################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/Third_Party/FatFs/src/option/subdir.mk
-include Middlewares/Third_Party/FatFs/src/subdir.mk
-include FC/Src/subdir.mk
-include FATFS/Target/subdir.mk
-include FATFS/App/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Drivers/MRT_Xtend/Src/subdir.mk
-include Drivers/MRT_VideoRecorder/Src/subdir.mk
-include Drivers/MRT_SX1262/Src/subdir.mk
-include Drivers/MRT_SD/Src/subdir.mk
-include Drivers/MRT_MAX31855/Src/subdir.mk
-include Drivers/MRT_LSM6DSR/Src/subdir.mk
-include Drivers/MRT_LPS22HH/Src/subdir.mk
-include Drivers/MRT_Helpers/Src/subdir.mk
-include Drivers/MRT_GPS_NEOM8/Src/subdir.mk
-include Drivers/MRT_FLASH_W25QXX/Src/subdir.mk
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


BUILD_ARTIFACT_NAME := f437_fc_v4.3_megaloop_with_ejection
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
f437_fc_v4.3_megaloop_with_ejection.elf \

MAP_FILES += \
f437_fc_v4.3_megaloop_with_ejection.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
f437_fc_v4.3_megaloop_with_ejection.list \


# All Target
all: main-build

# Main-build Target
main-build: f437_fc_v4.3_megaloop_with_ejection.elf secondary-outputs

# Tool invocations
f437_fc_v4.3_megaloop_with_ejection.elf f437_fc_v4.3_megaloop_with_ejection.map: $(OBJS) $(USER_OBJS) /Users/thomasjarvis/avionics-2022/Flight\ Computer/Testing/f437_fc_v4.3_megaloop_with_ejection/STM32F437ZITX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "f437_fc_v4.3_megaloop_with_ejection.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/Users/thomasjarvis/avionics-2022/Flight Computer/Testing/f437_fc_v4.3_megaloop_with_ejection/STM32F437ZITX_FLASH.ld" --specs=nosys.specs -Wl,-Map="f437_fc_v4.3_megaloop_with_ejection.map" -Wl,--gc-sections -static -u _printf_float -u _scanf_float --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

f437_fc_v4.3_megaloop_with_ejection.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "f437_fc_v4.3_megaloop_with_ejection.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) default.size.stdout f437_fc_v4.3_megaloop_with_ejection.elf f437_fc_v4.3_megaloop_with_ejection.list f437_fc_v4.3_megaloop_with_ejection.map
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
