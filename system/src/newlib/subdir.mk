CPP_SRCS += \
./system/src/newlib/cxx.cpp

C_SRCS += \
./system/src/newlib/assert.c \
./system/src/newlib/exit.c \
./system/src/newlib/sbrk.c \
./system/src/newlib/startup.c \
./system/src/newlib/syscalls.c

C_DEPS += \
./system/src/newlib/assert.d \
./system/src/newlib/exit.d \
./system/src/newlib/sbrk.d \
./system/src/newlib/startup.d \
./system/src/newlib/syscalls.d

OBJS += \
./system/src/newlib/assert.o \
./system/src/newlib/cxx.o \
./system/src/newlib/exit.o \
./system/src/newlib/sbrk.o \
./system/src/newlib/startup.o \
./system/src/newlib/syscalls.o

CPP_DEPS += \
./system/src/newlib/cxx.d

system/src/newlib/%.o: ./system/src/newlib/%.c system/src/newlib/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU Arm Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -flto -fno-move-loop-invariants -g3 -DDEBUG -DUSE_FULL_ASSERT -DSTM32G031xx -DUSE_HAL_DRIVER -DHSE_VALUE=8000000 -I"./include" -I"./system/include" -I"./system/include/cmsis" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

system/src/newlib/%.o: ./system/src/newlib/%.cpp system/src/newlib/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU Arm Cross C++ Compiler'
	arm-none-eabi-g++ -mcpu=cortex-m0plus -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -flto -fno-move-loop-invariants -g3 -DDEBUG -DUSE_FULL_ASSERT -DSTM32G031xx -DUSE_HAL_DRIVER -DHSE_VALUE=8000000 -I"./include" -I"./system/include" -I"./system/include/cmsis" -I"./system/include/stm32f0-hal" -std=gnu++11 -fabi-version=0 -fno-exceptions -fno-rtti -fno-use-cxa-atexit -fno-threadsafe-statics -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '
