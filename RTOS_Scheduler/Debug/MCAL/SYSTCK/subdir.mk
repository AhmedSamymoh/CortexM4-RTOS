################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MCAL/SYSTCK/SYSTICK_program.c 

OBJS += \
./MCAL/SYSTCK/SYSTICK_program.o 

C_DEPS += \
./MCAL/SYSTCK/SYSTICK_program.d 


# Each subdirectory must supply rules for building sources it contributes
MCAL/SYSTCK/%.o MCAL/SYSTCK/%.su MCAL/SYSTCK/%.cyclo: ../MCAL/SYSTCK/%.c MCAL/SYSTCK/subdir.mk
	arm-none-eabi-gcc -gdwarf-2 "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=soft -mthumb -o "$@"

clean: clean-MCAL-2f-SYSTCK

clean-MCAL-2f-SYSTCK:
	-$(RM) ./MCAL/SYSTCK/SYSTICK_program.cyclo ./MCAL/SYSTCK/SYSTICK_program.d ./MCAL/SYSTCK/SYSTICK_program.o ./MCAL/SYSTCK/SYSTICK_program.su

.PHONY: clean-MCAL-2f-SYSTCK

