################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Inc/SwerveDrive/SwerveDrive.cpp \
../Core/Inc/SwerveDrive/SwerveModule.cpp 

OBJS += \
./Core/Inc/SwerveDrive/SwerveDrive.o \
./Core/Inc/SwerveDrive/SwerveModule.o 

CPP_DEPS += \
./Core/Inc/SwerveDrive/SwerveDrive.d \
./Core/Inc/SwerveDrive/SwerveModule.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/SwerveDrive/%.o Core/Inc/SwerveDrive/%.su Core/Inc/SwerveDrive/%.cyclo: ../Core/Inc/SwerveDrive/%.cpp Core/Inc/SwerveDrive/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32F105xC -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/ARM/DSP/Inc -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-SwerveDrive

clean-Core-2f-Inc-2f-SwerveDrive:
	-$(RM) ./Core/Inc/SwerveDrive/SwerveDrive.cyclo ./Core/Inc/SwerveDrive/SwerveDrive.d ./Core/Inc/SwerveDrive/SwerveDrive.o ./Core/Inc/SwerveDrive/SwerveDrive.su ./Core/Inc/SwerveDrive/SwerveModule.cyclo ./Core/Inc/SwerveDrive/SwerveModule.d ./Core/Inc/SwerveDrive/SwerveModule.o ./Core/Inc/SwerveDrive/SwerveModule.su

.PHONY: clean-Core-2f-Inc-2f-SwerveDrive

