################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ECUAL/KEYPAD/KEYPAD.c \
../ECUAL/KEYPAD/KEYPAD_cfg.c 

OBJS += \
./ECUAL/KEYPAD/KEYPAD.o \
./ECUAL/KEYPAD/KEYPAD_cfg.o 

C_DEPS += \
./ECUAL/KEYPAD/KEYPAD.d \
./ECUAL/KEYPAD/KEYPAD_cfg.d 


# Each subdirectory must supply rules for building sources it contributes
ECUAL/KEYPAD/KEYPAD.o: ../ECUAL/KEYPAD/KEYPAD.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -c -I../Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"ECUAL/KEYPAD/KEYPAD.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
ECUAL/KEYPAD/KEYPAD_cfg.o: ../ECUAL/KEYPAD/KEYPAD_cfg.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -c -I../Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"ECUAL/KEYPAD/KEYPAD_cfg.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

