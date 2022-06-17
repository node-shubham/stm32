################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/001-led_toggle.c 

OBJS += \
./Src/001-led_toggle.o 

C_DEPS += \
./Src/001-led_toggle.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F401CCYx -DSTM32F4 -c -I../Inc -I"E:/ARM/INDIVIDUAL/FRESH_DEVELOPEMENT/stm32f401xx_driver_development/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/001-led_toggle.d ./Src/001-led_toggle.o ./Src/001-led_toggle.su

.PHONY: clean-Src

