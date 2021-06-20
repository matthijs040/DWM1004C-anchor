################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/deca_driver/deca_device.c \
../Drivers/deca_driver/deca_params_init.c 

OBJS += \
./Drivers/deca_driver/deca_device.o \
./Drivers/deca_driver/deca_params_init.o 

C_DEPS += \
./Drivers/deca_driver/deca_device.d \
./Drivers/deca_driver/deca_params_init.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/deca_driver/deca_device.o: ../Drivers/deca_driver/deca_device.c Drivers/deca_driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DUSE_FULL_LL_DRIVER '-DHSE_VALUE=8000000' '-DHSE_STARTUP_TIMEOUT=100' '-DLSE_STARTUP_TIMEOUT=5000' '-DLSE_VALUE=32768' '-DMSI_VALUE=131072' '-DHSI_VALUE=16000000' '-DLSI_VALUE=37000' '-DVDD_VALUE=3300' '-DPREFETCH_ENABLE=0' '-DINSTRUCTION_CACHE_ENABLE=1' '-DDATA_CACHE_ENABLE=1' '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DSTM32L041xx -c -I../Inc -I../Src -I../Src/instance -I../Src/mems -I../Src/Port -I../Src/utils -I../Src/cmd -I../Src/config -I../Drivers/deca_driver -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/CMSIS/Include -Og -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/deca_driver/deca_device.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Drivers/deca_driver/deca_params_init.o: ../Drivers/deca_driver/deca_params_init.c Drivers/deca_driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DUSE_FULL_LL_DRIVER '-DHSE_VALUE=8000000' '-DHSE_STARTUP_TIMEOUT=100' '-DLSE_STARTUP_TIMEOUT=5000' '-DLSE_VALUE=32768' '-DMSI_VALUE=131072' '-DHSI_VALUE=16000000' '-DLSI_VALUE=37000' '-DVDD_VALUE=3300' '-DPREFETCH_ENABLE=0' '-DINSTRUCTION_CACHE_ENABLE=1' '-DDATA_CACHE_ENABLE=1' '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DSTM32L041xx -c -I../Inc -I../Src -I../Src/instance -I../Src/mems -I../Src/Port -I../Src/utils -I../Src/cmd -I../Src/config -I../Drivers/deca_driver -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/CMSIS/Include -Og -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/deca_driver/deca_params_init.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

