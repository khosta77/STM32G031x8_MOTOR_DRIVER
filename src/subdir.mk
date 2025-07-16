C_SRCS += ./src/main.cpp

C_DEPS += ./src/main.d

OBJS += ./src/main.o

src/%.o: ./src/%.cpp src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU Arm Cross C Compiler'
	arm-none-eabi-g++ -mcpu=cortex-m0plus -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -g3 -DDEBUG -DUSE_FULL_ASSERT -DSTM32G031xx -DUSE_HAL_DRIVER -DHSE_VALUE=8000000 -I"./include" -I"./system/include" -I"./system/include/cmsis" -std=gnu++17 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '
