################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include ECUAL/LEDS/subdir.mk
-include ECUAL/KEYPAD/subdir.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Keypad_LEDs.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Keypad_LEDs.list \

OBJCOPY_BIN += \
Keypad_LEDs.bin \


# All Target
all: Keypad_LEDs.elf secondary-outputs

# Tool invocations
Keypad_LEDs.elf: $(OBJS) $(USER_OBJS) C:\Users\Khaled\ Magdy\Desktop\Keypad_LEDs\STM32F103C8TX_FLASH.ld
	arm-none-eabi-gcc -o "Keypad_LEDs.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\Khaled Magdy\Desktop\Keypad_LEDs\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Keypad_LEDs.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Keypad_LEDs.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Keypad_LEDs.list"
	@echo 'Finished building: $@'
	@echo ' '

Keypad_LEDs.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Keypad_LEDs.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
