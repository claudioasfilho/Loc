################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/InitDevice.c 

OBJS += \
./src/InitDevice.o 

C_DEPS += \
./src/InitDevice.d 


# Each subdirectory must supply rules for building sources it contributes
src/InitDevice.o: ../src/InitDevice.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DGENERATION_DONE=1' '-DSILABS_AF_USE_HWCONF=1' '-D__NO_SYSTEM_INIT=1' '-DEFR32BG1B232F256GM56=1' -I"/Users/clfilho/SimplicityStudio/v4_workspace/Locoroll-2/inc" -I"/Users/clfilho/SimplicityStudio/v4_workspace/Locoroll-2" -I"/Applications/Simplicity Studio.app/Contents/Eclipse/developer/stacks/blehomekit/v1.1.0//protocol/bluetooth_2.3/ble_stack/inc/common" -I"/Applications/Simplicity Studio.app/Contents/Eclipse/developer/stacks/blehomekit/v1.1.0//protocol/bluetooth_2.3/ble_stack/inc/soc" -I"/Applications/Simplicity Studio.app/Contents/Eclipse/developer/stacks/blehomekit/v1.1.0//platform/bootloader/api" -I"/Applications/Simplicity Studio.app/Contents/Eclipse/developer/stacks/blehomekit/v1.1.0//platform/emdrv/dmadrv/inc" -I"/Applications/Simplicity Studio.app/Contents/Eclipse/developer/stacks/blehomekit/v1.1.0//platform/emlib/inc" -I"/Applications/Simplicity Studio.app/Contents/Eclipse/developer/stacks/blehomekit/v1.1.0//platform/CMSIS/Include" -I"/Applications/Simplicity Studio.app/Contents/Eclipse/developer/stacks/blehomekit/v1.1.0//platform/Device/SiliconLabs/EFR32BG1B/Include" -I"/Applications/Simplicity Studio.app/Contents/Eclipse/developer/stacks/blehomekit/v1.1.0//platform/emdrv/common/inc" -I"/Applications/Simplicity Studio.app/Contents/Eclipse/developer/stacks/blehomekit/v1.1.0//platform/emdrv/dmadrv/config" -I"/Applications/Simplicity Studio.app/Contents/Eclipse/developer/stacks/blehomekit/v1.1.0//platform/emdrv/gpiointerrupt/inc" -I"/Applications/Simplicity Studio.app/Contents/Eclipse/developer/stacks/blehomekit/v1.1.0//platform/emdrv/nvm/config" -I"/Applications/Simplicity Studio.app/Contents/Eclipse/developer/stacks/blehomekit/v1.1.0//platform/emdrv/nvm/inc" -I"/Applications/Simplicity Studio.app/Contents/Eclipse/developer/stacks/blehomekit/v1.1.0//platform/emdrv/rtcdrv/config" -I"/Applications/Simplicity Studio.app/Contents/Eclipse/developer/stacks/blehomekit/v1.1.0//platform/emdrv/rtcdrv/inc" -I"/Applications/Simplicity Studio.app/Contents/Eclipse/developer/stacks/blehomekit/v1.1.0//platform/emdrv/sleep/inc" -I"/Applications/Simplicity Studio.app/Contents/Eclipse/developer/stacks/blehomekit/v1.1.0//platform/emdrv/spidrv/config" -I"/Applications/Simplicity Studio.app/Contents/Eclipse/developer/stacks/blehomekit/v1.1.0//platform/emdrv/spidrv/inc" -I"/Applications/Simplicity Studio.app/Contents/Eclipse/developer/stacks/blehomekit/v1.1.0//platform/emdrv/tempdrv/config" -I"/Applications/Simplicity Studio.app/Contents/Eclipse/developer/stacks/blehomekit/v1.1.0//platform/emdrv/tempdrv/inc" -I"/Applications/Simplicity Studio.app/Contents/Eclipse/developer/stacks/blehomekit/v1.1.0//platform/emdrv/uartdrv/config" -I"/Applications/Simplicity Studio.app/Contents/Eclipse/developer/stacks/blehomekit/v1.1.0//platform/emdrv/uartdrv/inc" -I"/Applications/Simplicity Studio.app/Contents/Eclipse/developer/stacks/blehomekit/v1.1.0//platform/emdrv/ustimer/config" -I"/Applications/Simplicity Studio.app/Contents/Eclipse/developer/stacks/blehomekit/v1.1.0//platform/emdrv/ustimer/inc" -I"/Applications/Simplicity Studio.app/Contents/Eclipse/developer/stacks/blehomekit/v1.1.0//hardware/kit/EFR32BG1_BRD4302A/config" -I"/Applications/Simplicity Studio.app/Contents/Eclipse/developer/stacks/blehomekit/v1.1.0//hardware/kit/common/bsp" -I"/Applications/Simplicity Studio.app/Contents/Eclipse/developer/stacks/blehomekit/v1.1.0//hardware/kit/common/drivers" -I"/Applications/Simplicity Studio.app/Contents/Eclipse/developer/stacks/blehomekit/v1.1.0//platform/radio/rail_lib/chip/efr32/rf/common/cortex" -I"/Applications/Simplicity Studio.app/Contents/Eclipse/developer/stacks/blehomekit/v1.1.0//platform/radio/rail_lib/common" -I"/Applications/Simplicity Studio.app/Contents/Eclipse/developer/stacks/blehomekit/v1.1.0//platform/radio/rail_lib/chip/efr32" -I"/Users/clfilho/SimplicityStudio/v4_workspace/Locoroll-2/src" -O0 -fno-short-enums -Wall -c -fmessage-length=0 -ffunction-sections -fdata-sections -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -MMD -MP -MF"src/InitDevice.d" -MT"src/InitDevice.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


