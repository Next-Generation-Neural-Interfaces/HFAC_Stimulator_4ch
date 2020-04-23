################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../SDK/devices/MK64F12/drivers/fsl_clock.c \
../SDK/devices/MK64F12/drivers/fsl_common.c \
../SDK/devices/MK64F12/drivers/fsl_dmamux.c \
../SDK/devices/MK64F12/drivers/fsl_dspi.c \
../SDK/devices/MK64F12/drivers/fsl_edma.c \
../SDK/devices/MK64F12/drivers/fsl_ftm.c \
../SDK/devices/MK64F12/drivers/fsl_gpio.c \
../SDK/devices/MK64F12/drivers/fsl_pit.c \
../SDK/devices/MK64F12/drivers/fsl_uart.c 

OBJS += \
./SDK/devices/MK64F12/drivers/fsl_clock.o \
./SDK/devices/MK64F12/drivers/fsl_common.o \
./SDK/devices/MK64F12/drivers/fsl_dmamux.o \
./SDK/devices/MK64F12/drivers/fsl_dspi.o \
./SDK/devices/MK64F12/drivers/fsl_edma.o \
./SDK/devices/MK64F12/drivers/fsl_ftm.o \
./SDK/devices/MK64F12/drivers/fsl_gpio.o \
./SDK/devices/MK64F12/drivers/fsl_pit.o \
./SDK/devices/MK64F12/drivers/fsl_uart.o 

C_DEPS += \
./SDK/devices/MK64F12/drivers/fsl_clock.d \
./SDK/devices/MK64F12/drivers/fsl_common.d \
./SDK/devices/MK64F12/drivers/fsl_dmamux.d \
./SDK/devices/MK64F12/drivers/fsl_dspi.d \
./SDK/devices/MK64F12/drivers/fsl_edma.d \
./SDK/devices/MK64F12/drivers/fsl_ftm.d \
./SDK/devices/MK64F12/drivers/fsl_gpio.d \
./SDK/devices/MK64F12/drivers/fsl_pit.d \
./SDK/devices/MK64F12/drivers/fsl_uart.d 


# Each subdirectory must supply rules for building sources it contributes
SDK/devices/MK64F12/drivers/%.o: ../SDK/devices/MK64F12/drivers/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g -DCPU_MK64FN1M0VLL12 -DFREEDOM -DFRDM_K64F -I"C:\Users\adrien\MCU Programming\K64F_SDK_NoDebugConsole/devices/MK64F12" -I"C:\Users\adrien\MCU Programming\K64F_SDK_NoDebugConsole/devices/MK64F12/utilities" -I"C:\Users\adrien\MCU Programming\K64F_SDK_NoDebugConsole/devices/MK64F12/drivers" -I"C:\Users\adrien\MCU Programming\K64F_SDK_NoDebugConsole/CMSIS/Include" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


