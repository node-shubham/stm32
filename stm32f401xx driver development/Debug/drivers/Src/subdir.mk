################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/f401_gpio.c \
../drivers/Src/f401_i2c.c \
../drivers/Src/f401_rcc.c \
../drivers/Src/f401_spi.c \
../drivers/Src/f401_usart.c 

OBJS += \
./drivers/Src/f401_gpio.o \
./drivers/Src/f401_i2c.o \
./drivers/Src/f401_rcc.o \
./drivers/Src/f401_spi.o \
./drivers/Src/f401_usart.o 

C_DEPS += \
./drivers/Src/f401_gpio.d \
./drivers/Src/f401_i2c.d \
./drivers/Src/f401_rcc.d \
./drivers/Src/f401_spi.d \
./drivers/Src/f401_usart.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/%.o drivers/Src/%.su: ../drivers/Src/%.c drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F401CCYx -DSTM32F4 -c -I../Inc -I"E:/ARM/INDIVIDUAL/FRESH_DEVELOPEMENT/stm32f401xx_driver_development/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-drivers-2f-Src

clean-drivers-2f-Src:
	-$(RM) ./drivers/Src/f401_gpio.d ./drivers/Src/f401_gpio.o ./drivers/Src/f401_gpio.su ./drivers/Src/f401_i2c.d ./drivers/Src/f401_i2c.o ./drivers/Src/f401_i2c.su ./drivers/Src/f401_rcc.d ./drivers/Src/f401_rcc.o ./drivers/Src/f401_rcc.su ./drivers/Src/f401_spi.d ./drivers/Src/f401_spi.o ./drivers/Src/f401_spi.su ./drivers/Src/f401_usart.d ./drivers/Src/f401_usart.o ./drivers/Src/f401_usart.su

.PHONY: clean-drivers-2f-Src

