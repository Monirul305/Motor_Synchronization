################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Motor/src/BLDC_PID_Tuning_GA.c \
../Motor/src/BLDC_PID_Tuning_QPS.c \
../Motor/src/motor1.c \
../Motor/src/motor2.c 

OBJS += \
./Motor/src/BLDC_PID_Tuning_GA.o \
./Motor/src/BLDC_PID_Tuning_QPS.o \
./Motor/src/motor1.o \
./Motor/src/motor2.o 

C_DEPS += \
./Motor/src/BLDC_PID_Tuning_GA.d \
./Motor/src/BLDC_PID_Tuning_QPS.d \
./Motor/src/motor1.d \
./Motor/src/motor2.d 


# Each subdirectory must supply rules for building sources it contributes
Motor/src/%.o Motor/src/%.su Motor/src/%.cyclo: ../Motor/src/%.c Motor/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_NUCLEO_64 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/BSP/STM32G4xx_Nucleo -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/User/Documents/PID/STM32_Codes/Motor_Synchronization/Motor_Synchronization/Motor/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Motor-2f-src

clean-Motor-2f-src:
	-$(RM) ./Motor/src/BLDC_PID_Tuning_GA.cyclo ./Motor/src/BLDC_PID_Tuning_GA.d ./Motor/src/BLDC_PID_Tuning_GA.o ./Motor/src/BLDC_PID_Tuning_GA.su ./Motor/src/BLDC_PID_Tuning_QPS.cyclo ./Motor/src/BLDC_PID_Tuning_QPS.d ./Motor/src/BLDC_PID_Tuning_QPS.o ./Motor/src/BLDC_PID_Tuning_QPS.su ./Motor/src/motor1.cyclo ./Motor/src/motor1.d ./Motor/src/motor1.o ./Motor/src/motor1.su ./Motor/src/motor2.cyclo ./Motor/src/motor2.d ./Motor/src/motor2.o ./Motor/src/motor2.su

.PHONY: clean-Motor-2f-src

