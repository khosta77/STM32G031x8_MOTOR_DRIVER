C_SRCS += \
./system/src/cmsis/system_stm32g0xx.c \
./system/src/cmsis/vectors_stm32g031x8.c

C_DEPS += \
./system/src/cmsis/system_stm32g0xx.d \
./system/src/cmsis/vectors_stm32g031x8.d

OBJS += \
./system/src/cmsis/system_stm32g0xx.o \
./system/src/cmsis/vectors_stm32g031x8.o

system/src/cmsis/%.o: ./system/src/cmsis/%.c system/src/cmsis/subdir.mk
	@echo 'In process: ./system/src/cmsis/'
	@echo 'Building file: $<'
	@echo 'Invoking: GNU Arm Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -flto -fno-move-loop-invariants -g3 -DDEBUG -DUSE_FULL_ASSERT -DSTM32G031xx -DUSE_HAL_DRIVER -DHSE_VALUE=8000000 -I"./include" -I"./system/include" -I"./system/include/cmsis" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '
