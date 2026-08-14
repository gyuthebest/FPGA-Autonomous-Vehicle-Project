#include "platform.h"
#include "platform_config.h"
#include "xparameters.h"
#include "xparameters_ps.h"
#include "xil_cache.h"
#include "xil_exception.h"
#include "xscugic.h"
#include "xttcps.h"
#include "lwip/tcp.h"

#define INTC_DEVICE_ID XPAR_SCUGIC_SINGLE_DEVICE_ID
#define TIMER_DEVICE_ID XPAR_XTTCPS_0_DEVICE_ID
#define TIMER_IRPT_INTR XPAR_XTTCPS_0_INTR
#define INTC_BASE_ADDR XPAR_SCUGIC_0_CPU_BASEADDR
#define INTC_DIST_BASE_ADDR XPAR_SCUGIC_0_DIST_BASEADDR
#define PLATFORM_TIMER_INTR_RATE_HZ 4

static XTtcPs timer_instance;
static XInterval timer_interval;
static u8 timer_prescaler;

volatile int TcpFastTmrFlag = 0;
volatile int TcpSlowTmrFlag = 0;

#if LWIP_DHCP == 1
volatile int dhcp_timoutcntr = 24;
void dhcp_fine_tmr(void);
void dhcp_coarse_tmr(void);
#endif

static void platform_clear_interrupt(XTtcPs *timer)
{
    u32 status = XTtcPs_GetInterruptStatus(timer);
    XTtcPs_ClearInterruptStatus(timer, status);
}

static void timer_callback(XTtcPs *timer)
{
    static int odd = 1;
#if LWIP_DHCP == 1
    static int dhcp_timer = 0;
#endif

    TcpFastTmrFlag = 1;
    odd = !odd;
    if (odd) {
        TcpSlowTmrFlag = 1;
#if LWIP_DHCP == 1
        dhcp_timer++;
        dhcp_timoutcntr--;
        dhcp_fine_tmr();
        if (dhcp_timer >= 120) {
            dhcp_coarse_tmr();
            dhcp_timer = 0;
        }
#endif
    }
    platform_clear_interrupt(timer);
}

void platform_setup_timer(void)
{
    XTtcPs_Config *config = XTtcPs_LookupConfig(TIMER_DEVICE_ID);
    if (config == NULL) return;
    if (XTtcPs_CfgInitialize(&timer_instance, config, config->BaseAddress) != XST_SUCCESS)
        return;
    XTtcPs_SetOptions(&timer_instance,
                      XTTCPS_OPTION_INTERVAL_MODE | XTTCPS_OPTION_WAVE_DISABLE);
    XTtcPs_CalcIntervalFromFreq(&timer_instance, PLATFORM_TIMER_INTR_RATE_HZ,
                                &timer_interval, &timer_prescaler);
    XTtcPs_SetInterval(&timer_instance, timer_interval);
    XTtcPs_SetPrescaler(&timer_instance, timer_prescaler);
}

static void platform_setup_interrupts(void)
{
    Xil_ExceptionInit();
    XScuGic_DeviceInitialize(INTC_DEVICE_ID);
    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_IRQ_INT,
                                 (Xil_ExceptionHandler)XScuGic_DeviceInterruptHandler,
                                 (void *)INTC_DEVICE_ID);
    XScuGic_RegisterHandler(INTC_BASE_ADDR, TIMER_IRPT_INTR,
                            (Xil_ExceptionHandler)timer_callback,
                            (void *)&timer_instance);
    XScuGic_EnableIntr(INTC_DIST_BASE_ADDR, TIMER_IRPT_INTR);
}

void platform_enable_interrupts(void)
{
    Xil_ExceptionEnableMask(XIL_EXCEPTION_IRQ);
    XScuGic_EnableIntr(INTC_DIST_BASE_ADDR, TIMER_IRPT_INTR);
    XTtcPs_EnableInterrupts(&timer_instance, XTTCPS_IXR_INTERVAL_MASK);
    XTtcPs_Start(&timer_instance);
}

void init_platform(void)
{
    platform_setup_timer();
    platform_setup_interrupts();
}

void cleanup_platform(void)
{
    Xil_ICacheDisable();
    Xil_DCacheDisable();
}
