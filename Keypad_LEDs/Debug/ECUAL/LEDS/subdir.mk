################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ECUAL/LEDS/LEDS.c \
../ECUAL/LEDS/LEDS_cfg.c 

OBJS += \
./ECUAL/LEDS/LEDS.o \
./ECUAL/LEDS/LEDS_cfg.o 

C_DEPS += \
./ECUAL/LEDS/LEDS.d \
./ECUAL/LEDS/LEDS_cfg.d 


# Each subdirectory must supply rules for building sources it contributes
ECUAL/LEDS/LEDS.o: ../ECUAL/LEDS/LEDS.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -c -I../Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"ECUAL/LEDS/LEDS.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
ECUAL/LEDS/LEDS_cfg.o: ../ECUAL/LEDS/LEDS_cfg.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -c -I../Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"ECUAL/LEDS/LEDS_cfg.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

