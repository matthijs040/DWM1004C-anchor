################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../startup/startup_stm32l041xx.s 

OBJS += \
./startup/startup_stm32l041xx.o 

S_DEPS += \
./startup/startup_stm32l041xx.d 


# Each subdirectory must supply rules for building sources it contributes
startup/startup_stm32l041xx.o: ../startup/startup_stm32l041xx.s startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m0plus -g3 -c -x assembler-with-cpp -MMD -MP -MF"startup/startup_stm32l041xx.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

