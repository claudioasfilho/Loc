################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include kit_flashpwr/BRD4302A/subdir.mk
-include emlib/subdir.mk
-include device/EFR32_B_1_1_B/gcc/subdir.mk
-include bgapi/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: Locoroll-2.axf

# Tool invocations
Locoroll-2.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM C Linker'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -T "/Users/clfilho/SimplicityStudio/v4_workspace/Locoroll-2/linker/GCC/efr32bg1b232f256gm56.ld" -Xlinker -no-enum-size-warning -Xlinker -no-wchar-size-warning -Xlinker --gc-sections -Xlinker -Map="Locoroll-2.map" -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -o Locoroll-2.axf "./application_properties.o" "./ble-callback-stubs.o" "./ble-callbacks.o" "./em_timer.o" "./gatt_db.o" "./main.o" "./peripherals.o" "./bgapi/dmadrv.o" "./bgapi/em_adc.o" "./bgapi/em_ldma.o" "./bgapi/em_leuart.o" "./device/EFR32_B_1_1_B/gcc/startup_efr32bg1b.o" "./emlib/em_cmu.o" "./emlib/em_cryotimer.o" "./emlib/em_emu.o" "./emlib/em_gpio.o" "./emlib/em_i2c.o" "./emlib/em_letimer.o" "./emlib/em_msc.o" "./emlib/em_rtcc.o" "./emlib/em_system.o" "./emlib/em_usart.o" "./kit_flashpwr/BRD4302A/mx25flash_spi.o" "./src/InitDevice.o" "/Applications/Simplicity Studio.app/Contents/Eclipse/developer/stacks/blehomekit/v1.1.0//protocol/bluetooth_2.3/lib/EFR32BG1B/EABI/stack.a" "/Applications/Simplicity Studio.app/Contents/Eclipse/developer/stacks/blehomekit/v1.1.0//protocol/bluetooth_2.3/lib/EFR32BG1B/EABI/binstack.o" "/Applications/Simplicity Studio.app/Contents/Eclipse/developer/stacks/blehomekit/v1.1.0//protocol/bluetooth_2.3/lib/EFR32BG1B/GCC/bgapi.a" "/Applications/Simplicity Studio.app/Contents/Eclipse/developer/stacks/blehomekit/v1.1.0//protocol/bluetooth_2.3/lib/EFR32BG1B/EABI/binbootloader.o" -lm -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

	@echo 'Building hex file: Locoroll-2.hex'
	arm-none-eabi-objcopy -O ihex "Locoroll-2.axf" "Locoroll-2.hex"
	@echo ' '

	@echo 'Building bin file: Locoroll-2.bin'
	arm-none-eabi-objcopy -O binary "Locoroll-2.axf" "Locoroll-2.bin"
	@echo ' '

	@echo 'Building s37 file: Locoroll-2.s37'
	arm-none-eabi-objcopy -O srec "Locoroll-2.axf" "Locoroll-2.s37"
	@echo ' '

	@echo 'Running size tool'
	arm-none-eabi-size "Locoroll-2.axf"
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) Locoroll-2.axf
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
