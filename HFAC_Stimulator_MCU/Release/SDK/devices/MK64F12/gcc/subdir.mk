################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_UPPER_SRCS += \
C:/Users/adrien/MCU\ Programming/K64F_SDK_NoDebugConsole/devices/MK64F12/gcc/startup_MK64F12.S 

OBJS += \
./SDK/devices/MK64F12/gcc/startup_MK64F12.o 

S_UPPER_DEPS += \
./SDK/devices/MK64F12/gcc/startup_MK64F12.d 


# Each subdirectory must supply rules for building sources it contributes
SDK/devices/MK64F12/gcc/startup_MK64F12.o: C:/Users/adrien/MCU\ Programming/K64F_SDK_NoDebugConsole/devices/MK64F12/gcc/startup_MK64F12.S
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross Assembler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


