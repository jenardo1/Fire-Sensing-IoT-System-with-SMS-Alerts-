################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := IR Flame Sensor x Actuator Node
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
IR\ Flame\ Sensor\ x\ Actuator\ Node.elf \

MAP_FILES += \
IR\ Flame\ Sensor\ x\ Actuator\ Node.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
IR\ Flame\ Sensor\ x\ Actuator\ Node.list \

OBJCOPY_HEX += \
IR\ Flame\ Sensor\ x\ Actuator\ Node.hex \


# All Target
all: main-build

# Main-build Target
main-build: IR\ Flame\ Sensor\ x\ Actuator\ Node.elf secondary-outputs

# Tool invocations
IR\ Flame\ Sensor\ x\ Actuator\ Node.elf IR\ Flame\ Sensor\ x\ Actuator\ Node.map: $(OBJS) $(USER_OBJS) C:\Users\me\Documents\uvle\year3\eee158\IR\ Flame\ Sensor\ x\ Actuator\ Node\STM32F411RETX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "IR Flame Sensor x Actuator Node.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\me\Documents\uvle\year3\eee158\IR Flame Sensor x Actuator Node\STM32F411RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="IR Flame Sensor x Actuator Node.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

IR\ Flame\ Sensor\ x\ Actuator\ Node.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "IR Flame Sensor x Actuator Node.list"
	@echo 'Finished building: $@'
	@echo ' '

IR\ Flame\ Sensor\ x\ Actuator\ Node.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "IR Flame Sensor x Actuator Node.hex"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) IR\ Flame\ Sensor\ x\ Actuator\ Node.elf IR\ Flame\ Sensor\ x\ Actuator\ Node.hex IR\ Flame\ Sensor\ x\ Actuator\ Node.list IR\ Flame\ Sensor\ x\ Actuator\ Node.map IR\\ Flame\\ Sensor\\ x\\ Actuator\\ Node.elf default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
