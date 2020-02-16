# path to STM32F407 standard peripheral library
DEV_CAPS ?= STM32F4xx
# STD_PERIPH_LIBS ?= ./STM32L1xx_StdPeriph_Lib_V1.3.1
STD_PERIPH_DRIV ?= ./$(DEV_CAPS)_StdPeriph_Driver
DEV ?= stm32f4xx


# list of source files
SOURCES = $(wildcard usr_src/*.c)
SOURCES += ./Libraries/CMSIS/Device/ST/$(DEV_CAPS)/Source/Templates/system_$(DEV).c
SOURCES += ./Libraries/$(STD_PERIPH_DRIV)/src/$(DEV)_rcc.c
SOURCES += ./Libraries/$(STD_PERIPH_DRIV)/src/$(DEV)_gpio.c
SOURCES += ./Libraries/CMSIS/Device/ST/$(DEV_CAPS)/Source/Templates/TrueSTUDIO/startup_stm32f40xx.s

# name for output binary files
PROJECT ?= embedded_project

# compiler, objcopy (should be in PATH)
CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy

# path to st-flash (or should be specified in PATH)
ST_FLASH ?= st-flash

# specify compiler flags
CFLAGS = -g -O2 -Wall
CFLAGS += -T"./Project/STM32F4xx_StdPeriph_Templates/TrueSTUDIO/STM32F40_41xxx/STM32F417IG_FLASH.ld"
CFLAGS += -mlittle-endian -mthumb -specs=nano.specs -specs=nosys.specs -mcpu=cortex-m4 -mthumb-interwork -mfloat-abi=hard
CFLAGS += -DUSE_STDPERIPH_DRIVER -DSTM32F10X_MD
CFLAGS += -Wl,--gc-sections
CFLAGS += -I./usr_src
CFLAGS += -I./Libraries/CMSIS/Device/ST/$(DEV_CAPS)/Include
CFLAGS += -I./Libraries/Include
CFLAGS += -I./Libraries/$(STD_PERIPH_DRIV)/inc
CFLAGS += -I./Libraries/CMSIS/Include
CFLAGS += -I./Project/$(DEV_CAPS)_StdPeriph_Templates

OBJS = $(SOURCES:.c=.o)

all: $(PROJECT).elf

# compile
$(PROJECT).elf: $(SOURCES)
	$(CC) $(CFLAGS) $^ -o $@
	$(OBJCOPY) -O ihex $(PROJECT).elf $(PROJECT).hex
	$(OBJCOPY) -O binary $(PROJECT).elf $(PROJECT).bin

# remove binary files
clean:
	rm -f *.o *.elf *.hex *.bin

# flash
burn:
	sudo $(ST_FLASH) write $(PROJECT).bin 0x8000000

# Debug
gdb:
	arm-none-eabi-gdb $(PROJECT).elf
