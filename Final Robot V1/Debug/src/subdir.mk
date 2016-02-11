################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Robot.cpp \
../src/adjustValues.cpp \
../src/drivePower.cpp 

OBJS += \
./src/Robot.o \
./src/adjustValues.o \
./src/drivePower.o 

CPP_DEPS += \
./src/Robot.d \
./src/adjustValues.d \
./src/drivePower.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-frc-linux-gnueabi-g++ -std=c++1y -I"C:\Users\Connor/wpilib/cpp/current/include" -I"C:\Users\Connor\Documents\GitHub\Final Robot V1\src" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


