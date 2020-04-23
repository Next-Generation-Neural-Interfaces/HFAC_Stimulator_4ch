################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../SDK/devices/MK64F12/system_MK64F12.c 

OBJS += \
./SDK/devices/MK64F12/system_MK64F12.o 

C_DEPS += \
./SDK/devices/MK64F12/system_MK64F12.d 


# Each subdirectory must supply rules for building sources it contributes
SDK/devices/MK64F12/%.o: ../SDK/devices/MK64F12/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -DCPU_MK64FN1M0VLL12 -DSEMIHOSTING -DFREEDOM -DFRDM_K64F -I"C:\Users\adrien\MCU Programming\K64F_SDK_NoDebugConsole/devices/MK64F12" -I"C:\Users\adrien\MCU Programming\K64F_SDK_NoDebugConsole/devices/MK64F12/utilities" -I"C:\Users\adrien\MCU Programming\K64F_SDK_NoDebugConsole/devices/MK64F12/drivers" -I"C:\Users\adrien\MCU Programming\K64F_SDK_NoDebugConsole/CMSIS/Include" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


