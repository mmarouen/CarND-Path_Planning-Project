################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Eigen-3.3/bench/btl/libs/eigen2/btl_tiny_eigen2.cpp \
../src/Eigen-3.3/bench/btl/libs/eigen2/main_adv.cpp \
../src/Eigen-3.3/bench/btl/libs/eigen2/main_linear.cpp \
../src/Eigen-3.3/bench/btl/libs/eigen2/main_matmat.cpp \
../src/Eigen-3.3/bench/btl/libs/eigen2/main_vecmat.cpp 

OBJS += \
./src/Eigen-3.3/bench/btl/libs/eigen2/btl_tiny_eigen2.o \
./src/Eigen-3.3/bench/btl/libs/eigen2/main_adv.o \
./src/Eigen-3.3/bench/btl/libs/eigen2/main_linear.o \
./src/Eigen-3.3/bench/btl/libs/eigen2/main_matmat.o \
./src/Eigen-3.3/bench/btl/libs/eigen2/main_vecmat.o 

CPP_DEPS += \
./src/Eigen-3.3/bench/btl/libs/eigen2/btl_tiny_eigen2.d \
./src/Eigen-3.3/bench/btl/libs/eigen2/main_adv.d \
./src/Eigen-3.3/bench/btl/libs/eigen2/main_linear.d \
./src/Eigen-3.3/bench/btl/libs/eigen2/main_matmat.d \
./src/Eigen-3.3/bench/btl/libs/eigen2/main_vecmat.d 


# Each subdirectory must supply rules for building sources it contributes
src/Eigen-3.3/bench/btl/libs/eigen2/%.o: ../src/Eigen-3.3/bench/btl/libs/eigen2/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -std=c++11 -D__GXX_EXPERIMENTAL_CXX0X__ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


