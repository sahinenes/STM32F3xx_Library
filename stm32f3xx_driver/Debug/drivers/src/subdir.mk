################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/src/stm32f3xx_cortex.c \
../drivers/src/stm32f3xx_gpio.c \
../drivers/src/stm32f3xx_i2c.c \
../drivers/src/stm32f3xx_spi.c \
../drivers/src/stm32f3xx_usart.c 

OBJS += \
./drivers/src/stm32f3xx_cortex.o \
./drivers/src/stm32f3xx_gpio.o \
./drivers/src/stm32f3xx_i2c.o \
./drivers/src/stm32f3xx_spi.o \
./drivers/src/stm32f3xx_usart.o 

C_DEPS += \
./drivers/src/stm32f3xx_cortex.d \
./drivers/src/stm32f3xx_gpio.d \
./drivers/src/stm32f3xx_i2c.d \
./drivers/src/stm32f3xx_spi.d \
./drivers/src/stm32f3xx_usart.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/src/%.o: ../drivers/src/%.c drivers/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F3 -DSTM32F303RETx -DNUCLEO_F303RE -c -I"C:/Users/enes_/STM32CubeIDE/workspace_1.7.0/stm32f3xx_driver/drivers/inc" -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

