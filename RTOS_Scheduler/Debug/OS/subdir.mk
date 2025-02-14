################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../OS/Kernel_program.c 

OBJS += \
./OS/Kernel_program.o 

C_DEPS += \
./OS/Kernel_program.d 


# Each subdirectory must supply rules for building sources it contributes
OS/%.o OS/%.su OS/%.cyclo: ../OS/%.c OS/subdir.mk
	arm-none-eabi-gcc -gdwarf-2 "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=soft -mthumb -o "$@"

clean: clean-OS

clean-OS:
	-$(RM) ./OS/Kernel_program.cyclo ./OS/Kernel_program.d ./OS/Kernel_program.o ./OS/Kernel_program.su

.PHONY: clean-OS

