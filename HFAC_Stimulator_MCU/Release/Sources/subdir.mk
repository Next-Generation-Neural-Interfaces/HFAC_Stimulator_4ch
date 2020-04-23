################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Sources/K64F_HFAC_Stimulator.c \
../Sources/clock_config.c \
../Sources/peripherals.c \
../Sources/pin_mux.c 

OBJS += \
./Sources/K64F_HFAC_Stimulator.o \
./Sources/clock_config.o \
./Sources/peripherals.o \
./Sources/pin_mux.o 

C_DEPS += \
./Sources/K64F_HFAC_Stimulator.d \
./Sources/clock_config.d \
./Sources/peripherals.d \
./Sources/pin_mux.d 


# Each subdirectory must supply rules for building sources it contributes
Sources/%.o: ../Sources/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g -DCPU_MK64FN1M0VLL12 -DFREEDOM -DFRDM_K64F -I"C:\Users\adrien\MCU Programming\K64F_SDK_NoDebugConsole/devices/MK64F12" -I"C:\Users\adrien\MCU Programming\K64F_SDK_NoDebugConsole/devices/MK64F12/utilities" -I"C:\Users\adrien\MCU Programming\K64F_SDK_NoDebugConsole/devices/MK64F12/drivers" -I"C:\Users\adrien\MCU Programming\K64F_SDK_NoDebugConsole/CMSIS/Include" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


