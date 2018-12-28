################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/cost.cpp \
../src/main.cpp \
../src/road.cpp \
../src/vehicle.cpp 

OBJS += \
./src/cost.o \
./src/main.o \
./src/road.o \
./src/vehicle.o 

CPP_DEPS += \
./src/cost.d \
./src/main.d \
./src/road.d \
./src/vehicle.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -std=c++11 -D__GXX_EXPERIMENTAL_CXX0X__ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


