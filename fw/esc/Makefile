PRETTY = 1

######################################
# target
######################################
TARGETS = blinky

#######################################
# paths
#######################################
# Build path
OUTPUT_DIRECTORY = _build

# link script
$(OUTPUT_DIRECTORY)/blinky.out: \
  LINKER_SCRIPT  := STM32F031C6Tx_FLASH.ld

######################################
# source
######################################
# C sources
SRC_FILES += \
startup_stm32f031x6.s \
src/main_32F0.c \
src/stspin32f0_hal_msp.c \
src/stspin32f0_it.c \
Drivers/BSP/Components/stspin32f0/stspin32f0.c \
Drivers/BSP/STEVAL-ESC002V1/STEVAL-ESC002V1.c \
Drivers/CMSIS/Device/ST/STM32F0xx/Source/Templates/system_stm32f0xx.c \
Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal.c \
Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_adc.c \
Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_adc_ex.c \
Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_cortex.c \
Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_dma.c \
Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_tim.c \
Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_tim_ex.c \
Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_gpio.c \
Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_rcc.c \
Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_rcc_ex.c \
Drivers/SEGGER_RTT/SEGGER_RTT_printf.c \
Drivers/SEGGER_RTT/SEGGER_RTT.c \
Middlewares/ST/MC_6Step_Lib/Src/6Step_Lib.c

# Include folders common to all targets
INC_FOLDERS +=  \
Inc \
Drivers/STM32F0xx_HAL_Driver/Inc \
Drivers/BSP/Components/stspin32f0 \
Drivers/BSP/Components/common \
Drivers/BSP/STEVAL-ESC002V1 \
Drivers/CMSIS/Include \
Drivers/CMSIS/Device/ST/STM32F0xx/Include \
Drivers/SEGGER_RTT \
Middlewares/ST/MC_6Step_Lib/Inc \

LIB_FILES = 

ASM_SOURCES =  \


# debug build?
DEBUG = 0
# optimization
OPT = -O2

#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m0

# fpu
# NONE for Cortex-M0/M0+/M3

# float-abi

MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DSTEVAL_ESC002V1 \
-DUSE_HAL_DRIVER \
-DSTM32F031x6 \
-DNO_UART_COMM \
-DST_PWM_INTERFACE \
-DARR_FILTER \
-DSENSE_COMPARATORS \
-DNO_OPEN_LOOP_RAMP \
-DNO_VARIABLE_ADVANCE \
-DPID_V2 \
-DMC_7PP_3S_PROPELLER \
-DDELTA_6STEP_TABLE \
-DCOMPLEMENTARY_DRIVE \
-DVOLTAGE_MODE \
-DNO_SPEED_SENDING \
-DSPEED_RAMP \
-DOPEN_LOOP \
-DFAST_DEMAG




CXXFLAGS += $(OPT)

# compile gcc flags
CFLAGS += $(OPT)
CFLAGS += $(C_DEFS)
CFLAGS += $(MCU)
CLFAGS += -Wall
CFLAGS += -fdata-sections
CFLAGS += -ffunction-sections
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"
ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif

#assembler flags
ASFLAGS += $(OPT)
ASFLAGS += $(C_DEFS)
ASFLAGS += $(MCU)
ASFLAGS += -Wall
ASFLAGS += -fdata-sections
ASFLAGS += -ffunction-sections
ASFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"
ifeq ($(DEBUG), 1)
ASFLAGS += -g -gdwarf-2
endif

# linker flags
LDFLAGS = $(MCU) -specs=nano.specs -T$(LINKER_SCRIPT) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# Add standard libraries at the very end of the linker input, after all objects
# that may need symbols provided by these libraries.
LIB_FILES += -lc -lnosys -lm

# #######################################
# # build the application
# #######################################

.PHONY: default help

default: blinky

# Print all targets that can be built
help:
	@echo following targets are available:
	@echo		clean
	@echo		erase
	@echo		flash
	@echo		reset

include Makefile.common

$(foreach target, $(TARGETS), $(call define_target, $(target)))

.PHONY: flash 

#######################################
# clean up
#######################################  
erase:
	jlink -SelectEmuBySN 801020691 -device STM32F031C6 -if SWD -speed 8000 -autoconnect 1 -ExitOnError -CommanderScript ./erase.jlink

flash: default
	jlink -SelectEmuBySN 801020691 -device STM32F031C6 -if SWD -speed 4000 -autoconnect 1 -ExitOnError -CommanderScript ./flash.jlink

reset:
	jlink -SelectEmuBySN 801020691 -device STM32F031C6 -if SWD -speed 8000 -autoconnect 1 -ExitOnError -CommanderScript ./jlink/reset.jlink

#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***