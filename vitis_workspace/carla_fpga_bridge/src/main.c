/* Standalone/lwIP application for CARLA laptop <-> Zynq PS <-> PL. */
#include "platform.h"
#include "platform_config.h"
#include "ps_carla_bridge.h"
#include "netif/xadapter.h"
#include "lwip/init.h"
#include "lwip/ip_addr.h"
#include "lwip/tcp.h"
#include "lwip/priv/tcp_priv.h"
#if LWIP_DHCP
#include "lwip/dhcp.h"
#endif
#include "xil_printf.h"

extern volatile int TcpFastTmrFlag;
extern volatile int TcpSlowTmrFlag;
#if LWIP_DHCP
extern volatile int dhcp_timoutcntr;
#endif

static struct netif server_netif;

static void print_ip(const char *label, const ip_addr_t *address)
{
    xil_printf("%s%d.%d.%d.%d\r\n", label,
               ip4_addr1(address), ip4_addr2(address),
               ip4_addr3(address), ip4_addr4(address));
}

int main(void)
{
    struct netif *netif = &server_netif;
    ip_addr_t ipaddr, netmask, gateway;
    unsigned char mac[] = {0x00, 0x0A, 0x35, 0x00, 0x01, 0x02};

    init_platform();
    lwip_init();

#if LWIP_DHCP
    ip_addr_set_zero(&ipaddr);
    ip_addr_set_zero(&netmask);
    ip_addr_set_zero(&gateway);
#else
    IP4_ADDR(&ipaddr, 192, 168, 1, 10);
    IP4_ADDR(&netmask, 255, 255, 255, 0);
    IP4_ADDR(&gateway, 192, 168, 1, 1);
#endif

    if (xemac_add(netif, &ipaddr, &netmask, &gateway, mac,
                  PLATFORM_EMAC_BASEADDR) == NULL) {
        xil_printf("Error adding network interface\r\n");
        return -1;
    }
    netif_set_default(netif);
    platform_enable_interrupts();
    netif_set_up(netif);

#if LWIP_DHCP
    dhcp_start(netif);
    dhcp_timoutcntr = 24;
    while (ip4_addr_isany_val(*netif_ip4_addr(netif)) && dhcp_timoutcntr > 0)
        xemacif_input(netif);
    if (ip4_addr_isany_val(*netif_ip4_addr(netif))) {
        IP4_ADDR(netif_ip4_addr(netif), 192, 168, 1, 10);
        IP4_ADDR(netif_ip4_netmask(netif), 255, 255, 255, 0);
        IP4_ADDR(netif_ip4_gw(netif), 192, 168, 1, 1);
    }
#endif

    print_ip("Board IP: ", netif_ip_addr4(netif));
    if (carla_fpga_bridge_init() != 0) {
        xil_printf("CARLA FPGA bridge init failed\r\n");
        return -1;
    }

    for (;;) {
        if (TcpFastTmrFlag) {
            tcp_fasttmr();
            TcpFastTmrFlag = 0;
        }
        if (TcpSlowTmrFlag) {
            tcp_slowtmr();
            TcpSlowTmrFlag = 0;
        }
        xemacif_input(netif);
        carla_fpga_bridge_service();
    }
}
