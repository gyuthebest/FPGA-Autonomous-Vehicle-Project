################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
LD_SRCS += \
../src/lscript.ld 

C_SRCS += \
../src/main.c \
../src/platform_zynqmp.c \
../src/ps_carla_bridge.c 

OBJS += \
./src/main.o \
./src/platform_zynqmp.o \
./src/ps_carla_bridge.o 

C_DEPS += \
./src/main.d \
./src/platform_zynqmp.d \
./src/ps_carla_bridge.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM v8 gcc compiler'
	aarch64-none-elf-gcc -Wall -O0 -g3 -c -fmessage-length=0 -MT"$@" -IC:/Users/jiho0/GitProjects/FPGA-Autonomous-Vehicle-Project/vitis_workspace/carla_fpga_platform/export/carla_fpga_platform/sw/carla_fpga_platform/standalone_domain/bspinclude/include -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


