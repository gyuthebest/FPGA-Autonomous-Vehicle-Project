#include "ps_carla_bridge.h"
#include <stdint.h>
#include "xparameters.h"
#include "xil_io.h"
#include "xil_printf.h"
#include "lwip/def.h"
#include "lwip/ip_addr.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"

#if defined(XPAR_TOP_CONTROLLER_0_S_AXI_BASEADDR)
#define SENSOR_IP_BASEADDR XPAR_TOP_CONTROLLER_0_S_AXI_BASEADDR
#elif defined(XPAR_TOP_CONTROLLER_0_S00_AXI_BASEADDR)
#define SENSOR_IP_BASEADDR XPAR_TOP_CONTROLLER_0_S00_AXI_BASEADDR
#elif defined(XPAR_TOP_CONTROLLER_0_BASEADDR)
#define SENSOR_IP_BASEADDR XPAR_TOP_CONTROLLER_0_BASEADDR
#else
#define SENSOR_IP_BASEADDR 0x80000000U
#warning "top_controller base-address macro not found; using 0x80000000"
#endif

#define CARLA_INPUT_PORT 5001U
#define INPUT_WORD_COUNT 10U
#define OUTPUT_WORD_COUNT 15U
#define INPUT_BYTES (INPUT_WORD_COUNT * 4U)
#define OUTPUT_BYTES (OUTPUT_WORD_COUNT * 4U)
#define REG_OFFSET(index) ((uint32_t)(index) * 4U)
#define RISK_SEQ_INDEX 11U
#define REL_SEQ_INDEX 12U
#define COMMIT_INDEX 9U

static struct udp_pcb *rx_pcb;
static struct udp_pcb *tx_pcb;
static ip_addr_t client_ip;
static u16_t client_port;
static volatile uint32_t pending_seq;
static volatile int response_pending;

static void receive_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                             const ip_addr_t *sender, u16_t sender_port)
{
    uint32_t net[INPUT_WORD_COUNT], host[INPUT_WORD_COUNT];
    unsigned int i;
    (void)arg; (void)pcb;
    if (p == NULL) return;
    if (p->tot_len != INPUT_BYTES ||
        pbuf_copy_partial(p, net, INPUT_BYTES, 0U) != INPUT_BYTES) {
        xil_printf("[FPGA] invalid input length: %u\r\n", p->tot_len);
        pbuf_free(p);
        return;
    }
    for (i = 0U; i < INPUT_WORD_COUNT; ++i) host[i] = lwip_ntohl(net[i]);
    ip_addr_copy(client_ip, *sender);
    client_port = sender_port;
    for (i = 0U; i < COMMIT_INDEX; ++i)
        Xil_Out32(SENSOR_IP_BASEADDR + REG_OFFSET(i), host[i]);
    Xil_Out32(SENSOR_IP_BASEADDR + REG_OFFSET(COMMIT_INDEX), host[COMMIT_INDEX]);
    pending_seq = host[COMMIT_INDEX];
    response_pending = 1;
    pbuf_free(p);
}

int carla_fpga_bridge_init(void)
{
    rx_pcb = udp_new();
    tx_pcb = udp_new();
    if (rx_pcb == NULL || tx_pcb == NULL) return -1;
    if (udp_bind(rx_pcb, IP_ADDR_ANY, CARLA_INPUT_PORT) != ERR_OK) return -1;
    response_pending = 0;
    client_port = 0U;
    udp_recv(rx_pcb, receive_callback, NULL);
    xil_printf("[FPGA] CARLA UDP bridge ready on port %u\r\n", CARLA_INPUT_PORT);
    return 0;
}

void carla_fpga_bridge_service(void)
{
    uint32_t net[OUTPUT_WORD_COUNT];
    struct pbuf *p;
    err_t status;
    unsigned int i;
    if (!response_pending || client_port == 0U) return;
    if (Xil_In32(SENSOR_IP_BASEADDR + REG_OFFSET(RISK_SEQ_INDEX)) != pending_seq ||
        Xil_In32(SENSOR_IP_BASEADDR + REG_OFFSET(REL_SEQ_INDEX)) != pending_seq) return;
    for (i = 0U; i < OUTPUT_WORD_COUNT; ++i)
        net[i] = lwip_htonl(Xil_In32(SENSOR_IP_BASEADDR + REG_OFFSET(i)));
    p = pbuf_alloc(PBUF_TRANSPORT, OUTPUT_BYTES, PBUF_RAM);
    if (p == NULL) return;
    status = pbuf_take(p, net, OUTPUT_BYTES);
    if (status == ERR_OK) status = udp_sendto(tx_pcb, p, &client_ip, client_port);
    pbuf_free(p);
    if (status == ERR_OK) response_pending = 0;
}
