################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MEMS/Target/app_mems_int_pin_a_interface.c \
../MEMS/Target/custom_motion_sensors.c \
../MEMS/Target/custom_motion_sensors_ex.c 

C_DEPS += \
./MEMS/Target/app_mems_int_pin_a_interface.d \
./MEMS/Target/custom_motion_sensors.d \
./MEMS/Target/custom_motion_sensors_ex.d 

OBJS += \
./MEMS/Target/app_mems_int_pin_a_interface.o \
./MEMS/Target/custom_motion_sensors.o \
./MEMS/Target/custom_motion_sensors_ex.o 


# Each subdirectory must supply rules for building sources it contributes
MEMS/Target/%.o MEMS/Target/%.su MEMS/Target/%.cyclo: ../MEMS/Target/%.c MEMS/Target/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../MEMS/App -I../MEMS/Target -I../Drivers/BSP/STM32G4xx_Nucleo -I../Drivers/BSP/Components/lsm6dso -I../Drivers/BSP/Components/Common -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-MEMS-2f-Target

clean-MEMS-2f-Target:
	-$(RM) ./MEMS/Target/app_mems_int_pin_a_interface.cyclo ./MEMS/Target/app_mems_int_pin_a_interface.d ./MEMS/Target/app_mems_int_pin_a_interface.o ./MEMS/Target/app_mems_int_pin_a_interface.su ./MEMS/Target/custom_motion_sensors.cyclo ./MEMS/Target/custom_motion_sensors.d ./MEMS/Target/custom_motion_sensors.o ./MEMS/Target/custom_motion_sensors.su ./MEMS/Target/custom_motion_sensors_ex.cyclo ./MEMS/Target/custom_motion_sensors_ex.d ./MEMS/Target/custom_motion_sensors_ex.o ./MEMS/Target/custom_motion_sensors_ex.su

.PHONY: clean-MEMS-2f-Target

