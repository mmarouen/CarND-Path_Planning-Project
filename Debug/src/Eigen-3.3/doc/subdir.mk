################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Eigen-3.3/doc/tutorial.cpp 

OBJS += \
./src/Eigen-3.3/doc/tutorial.o 

CPP_DEPS += \
./src/Eigen-3.3/doc/tutorial.d 


# Each subdirectory must supply rules for building sources it contributes
src/Eigen-3.3/doc/%.o: ../src/Eigen-3.3/doc/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -std=c++11 -D__GXX_EXPERIMENTAL_CXX0X__ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


