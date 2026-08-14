#ifndef CARLA_FPGA_PLATFORM_H
#define CARLA_FPGA_PLATFORM_H

void init_platform(void);
void cleanup_platform(void);
void platform_setup_timer(void);
void platform_enable_interrupts(void);

#endif
