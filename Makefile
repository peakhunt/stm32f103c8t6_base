# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ChangeLog :
#	2017-02-10 - Several enhancements + project update mode
#   2015-07-22 - first version
# ------------------------------------------------
#
######################################
# host os
######################################
OS := $(shell uname)

######################################
# target
######################################
TARGET = stm32f103c8t6_base


######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Os
#OPT = -O2


#######################################
# paths
#######################################
# source path
SOURCES_DIR =  \
Application/User \
Drivers \
Application \
Application/MAKEFILE \
Middlewares/USB_Device_Library \
Drivers/STM32F1xx_HAL_Driver \
Drivers/CMSIS \
Middlewares

# firmware library path
PERIFLIB_PATH = 

# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources
C_SOURCES =  \
Src/usbd_conf.c \
Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_i2c.c \
Src/stm32f1xx_hal_msp.c \
Src/usb_device.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c \
Src/main.c \
Src/usbd_desc.c \
Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pcd.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pcd_ex.c \
Src/tim.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c \
Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_spi_ex.c \
Src/i2c.c \
Src/stm32f1xx_it.c \
Src/usart.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_usb.c \
Src/usbd_cdc_if.c \
Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c \
Src/spi.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_spi.c \
Src/system_stm32f1xx.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c \
Src/gpio.c \
Src/dma.c

APP_DIR=app
APP_SRC=														\
$(APP_DIR)/circ_buffer.c						\
$(APP_DIR)/bhash.c									\
$(APP_DIR)/event_dispatcher.c				\
$(APP_DIR)/soft_timer.c							\
$(APP_DIR)/mainloop_timer.c					\
$(APP_DIR)/stm32f1xx_callbacks.c		\
$(APP_DIR)/blinky.c									\
$(APP_DIR)/shell.c									\
$(APP_DIR)/shell_if_usart.c					\
$(APP_DIR)/shell_if_usb.c						\
$(APP_DIR)/micros.c									\
$(APP_DIR)/pwm_out.c								\
$(APP_DIR)/pwm_in.c								  \
$(APP_DIR)/i2c_bus.c								\
$(APP_DIR)/spi_bus.c								\
$(APP_DIR)/ws2812b.c								\
$(APP_DIR)/hmc5883.c								\
$(APP_DIR)/qmc5883.c								\
$(APP_DIR)/bmp180.c									\
$(APP_DIR)/bmp180_support.c					\
$(APP_DIR)/mpu6050.c								\
$(APP_DIR)/mpu6500_spi.c						\
$(APP_DIR)/imu.c										\
$(APP_DIR)/barometer.c							\
$(APP_DIR)/mahony.c									\
$(APP_DIR)/madgwick.c								\
$(APP_DIR)/sensor_calib.c						\
$(APP_DIR)/gyro_calibration.c				\
$(APP_DIR)/accel_calibration.c			\
$(APP_DIR)/mag_calibration.c			  \
$(APP_DIR)/config.c			            \
$(APP_DIR)/app.c


C_SOURCES += $(APP_SRC)

# ASM sources
ASM_SOURCES =  \
startup_stm32f103xb.s


######################################
# firmware library
######################################
PERIFLIB_SOURCES = 


#######################################
# binaries
#######################################
ifeq ($(OS),Darwin)
BINPATH = /Users/hawk/toolchains/gcc-arm-none-eabi-5_4-2016q3/bin
STLINKPATH= /opt/local/bin
else
BINPATH = /home/hawk/tools/toolchains/cortex-m/gcc-arm-none-eabi-4_9-2014q4/bin
STLINKPATH= ~/stlink
endif

PREFIX = arm-none-eabi-
CC = $(BINPATH)/$(PREFIX)gcc
AS = $(BINPATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(BINPATH)/$(PREFIX)objcopy
AR = $(BINPATH)/$(PREFIX)ar
SZ = $(BINPATH)/$(PREFIX)size
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m3

# fpu
# NONE for Cortex-M0/M0+/M3

# float-abi


# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =                    \
-DUSE_HAL_DRIVER            \
-DSTM32F103xB               \
-D__ENABLE_IMU              \
-DBMP180_API								


# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES =  \
-IInc \
-IDrivers/STM32F1xx_HAL_Driver/Inc \
-IDrivers/STM32F1xx_HAL_Driver/Inc/Legacy \
-IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc \
-IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc \
-IDrivers/CMSIS/Device/ST/STM32F1xx/Include \
-IDrivers/CMSIS/Include \
-I$(APP_DIR)


# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -Werror -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -Werror -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
#CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"
CFLAGS += -MMD -MF .dep/$(*F).d


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32F103C8Tx_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys
LIBDIR =
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections -u _printf_float

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@		

#######################################
# clean up
#######################################
clean:
	-rm -fR .dep $(BUILD_DIR)

#######################################
# flash
#######################################
flash: $(BUILD_DIR)/$(TARGET).bin
	$(STLINKPATH)/st-flash write $< 0x08000000

#######################################
# openocd
#######################################
openocd: $(BUILD_DIR)/$(TARGET).bin
	openocd -f interface/stlink-v2.cfg -f target/stm32f1x_stlink.cfg
  
#######################################
# dependencies
#######################################
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)

# *** EOF ***
