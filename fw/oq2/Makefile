SHELL := /bin/bash
PRETTY = 1
######################################
# target
######################################
TARGETS = oq2
TARGET = $(TARGETS)

#######################################
# paths
#######################################
# Build path
OUTPUT_DIRECTORY = _build
BUILD_DIR = $(OUTPUT_DIRECTORY)

# link script
$(OUTPUT_DIRECTORY)/$(TARGET).out: \
  LINKER_SCRIPT  := STM32H735ZGTx_FLASH.ld

######################################
# source
######################################
# C sources
SRC_FILES +=  \
application/sys/startup/startup_stm32h735zgtx.s \
application/main.c \
application/debug_log/debug_log.c \
application/flight_app/flight_app.c \
application/flight_app/esc_dfu/esc_dfu.c \
application/flight_app/esc_dfu/stm32_uart_dfu/stm32_uart_dfu.c \
application/freertos/freertos.c \
application/led_blinky/led_blinky.c \
application/location/location.c \
application/network/net_if/winc_netif.c \
application/network/net_init/net_init.c \
application/network/os_hook/os_hook.c \
application/network/wifi_ex/m2m_wifi_ex.c \
application/network/network.c \
application/network/oq2_protocol/oq2_protocol.c \
application/network/oq2_protocol/oq2p_packet_builders.c \
application/pid/pid.c \
application/stability_control/stability.c \
application/stability_control/platform/mems_platform.c \
application/stability_control/kinematics/kinematics.c \
application/stability_control/motors/motors.c \
application/task_manager/task_manager.c \
application/timer/timer.c \
application/sys/stm32h7xx_it.c \
application/sys/stm32h7xx_hal_msp.c \
application/sys/stm32h7xx_hal_timebase_tim.c \
application/sys/system_stm32h7xx.c \
application/utility/utility.c \
FATFS/Target/user_diskio.c \
FATFS/App/fatfs.c \
Drivers/wifi_drv_winc3400/bsp/source/nm_bsp_stm32h7xx.c \
Drivers/wifi_drv_winc3400/bus_wrapper/source/nm_bus_wrapper_stm32.c \
Drivers/wifi_drv_winc3400/common/source/efuse.c \
Drivers/wifi_drv_winc3400/common/source/nm_common.c \
Drivers/wifi_drv_winc3400/driver/source/m2m_ate_mode.c \
Drivers/wifi_drv_winc3400/driver/source/m2m_crypto.c \
Drivers/wifi_drv_winc3400/driver/source/m2m_hif.c \
Drivers/wifi_drv_winc3400/driver/source/m2m_ota.c \
Drivers/wifi_drv_winc3400/driver/source/m2m_periph.c \
Drivers/wifi_drv_winc3400/driver/source/m2m_ssl.c \
Drivers/wifi_drv_winc3400/driver/source/m2m_wifi.c \
Drivers/wifi_drv_winc3400/driver/source/nmasic.c \
Drivers/wifi_drv_winc3400/driver/source/nmbus.c \
Drivers/wifi_drv_winc3400/driver/source/nmdrv.c \
Drivers/wifi_drv_winc3400/driver/source/nmi2c.c \
Drivers/wifi_drv_winc3400/driver/source/nmspi.c \
Drivers/wifi_drv_winc3400/driver/source/nmuart.c \
Drivers/wifi_drv_winc3400/socket/source/socket.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd_ex.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_usb.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_hsem.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_mdma.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c_ex.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_exti.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_adc.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_adc_ex.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_crc.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_crc_ex.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cryp.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cryp_ex.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_hash.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_hash_ex.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_ospi.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rng.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rng_ex.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_sdmmc.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_delayblock.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_sd.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_sd_ex.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi_ex.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_lptim.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart_ex.c \
Drivers/Mems/lsm9ds1_STdC/driver/lsm9ds1_reg.c \
Drivers/Mems/lps22hh_STdC/driver/lps22hh_reg.c \
Middlewares/Third_Party/FatFs/src/diskio.c \
Middlewares/Third_Party/FatFs/src/ff.c \
Middlewares/Third_Party/FatFs/src/ff_gen_drv.c \
Middlewares/Third_Party/FatFs/src/option/syscall.c \
Middlewares/Third_Party/FreeRTOS/Source/croutine.c \
Middlewares/Third_Party/FreeRTOS/Source/event_groups.c \
Middlewares/Third_Party/FreeRTOS/Source/list.c \
Middlewares/Third_Party/FreeRTOS/Source/queue.c \
Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c \
Middlewares/Third_Party/FreeRTOS/Source/tasks.c \
Middlewares/Third_Party/FreeRTOS/Source/timers.c \
Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c \
Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c \
Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1/port.c \
Middlewares/Third_Party/LwIP/src/netif/ppp/auth.c \
Middlewares/Third_Party/LwIP/src/netif/ppp/ccp.c \
Middlewares/Third_Party/LwIP/src/netif/ppp/chap_ms.c \
Middlewares/Third_Party/LwIP/src/netif/ppp/chap-md5.c \
Middlewares/Third_Party/LwIP/src/netif/ppp/chap-new.c \
Middlewares/Third_Party/LwIP/src/netif/ppp/demand.c \
Middlewares/Third_Party/LwIP/src/netif/ppp/eap.c \
Middlewares/Third_Party/LwIP/src/netif/ppp/eui64.c \
Middlewares/Third_Party/LwIP/src/netif/ppp/fsm.c \
Middlewares/Third_Party/LwIP/src/netif/ppp/ipcp.c \
Middlewares/Third_Party/LwIP/src/netif/ppp/ipv6cp.c \
Middlewares/Third_Party/LwIP/src/netif/ppp/lcp.c \
Middlewares/Third_Party/LwIP/src/netif/ppp/magic.c \
Middlewares/Third_Party/LwIP/src/netif/ppp/mppe.c \
Middlewares/Third_Party/LwIP/src/netif/ppp/multilink.c \
Middlewares/Third_Party/LwIP/src/netif/ppp/ppp.c \
Middlewares/Third_Party/LwIP/src/netif/ppp/pppapi.c \
Middlewares/Third_Party/LwIP/src/netif/ppp/pppcrypt.c \
Middlewares/Third_Party/LwIP/src/netif/ppp/pppoe.c \
Middlewares/Third_Party/LwIP/src/netif/ppp/pppol2tp.c \
Middlewares/Third_Party/LwIP/src/netif/ppp/pppos.c \
Middlewares/Third_Party/LwIP/src/netif/ppp/upap.c \
Middlewares/Third_Party/LwIP/src/netif/ppp/utils.c \
Middlewares/Third_Party/LwIP/src/netif/ppp/vj.c \
Middlewares/Third_Party/LwIP/src/netif/bridgeif.c \
Middlewares/Third_Party/LwIP/src/netif/bridgeif_fdb.c \
Middlewares/Third_Party/LwIP/src/netif/ethernet.c \
Middlewares/Third_Party/LwIP/src/netif/lowpan6.c \
Middlewares/Third_Party/LwIP/src/netif/lowpan6_ble.c \
Middlewares/Third_Party/LwIP/src/netif/lowpan6_common.c \
Middlewares/Third_Party/LwIP/src/netif/slipif.c \
Middlewares/Third_Party/LwIP/src/netif/zepif.c \
Middlewares/Third_Party/LwIP/src/netif/ppp/ecp.c \
Middlewares/Third_Party/LwIP/src/api/api_lib.c \
Middlewares/Third_Party/LwIP/src/api/api_msg.c \
Middlewares/Third_Party/LwIP/src/api/err.c \
Middlewares/Third_Party/LwIP/src/api/if_api.c \
Middlewares/Third_Party/LwIP/src/api/netbuf.c \
Middlewares/Third_Party/LwIP/src/api/netdb.c \
Middlewares/Third_Party/LwIP/src/api/netifapi.c \
Middlewares/Third_Party/LwIP/src/api/sockets.c \
Middlewares/Third_Party/LwIP/src/api/tcpip.c \
Middlewares/Third_Party/LwIP/src/core/altcp.c \
Middlewares/Third_Party/LwIP/src/core/altcp_alloc.c \
Middlewares/Third_Party/LwIP/src/core/altcp_tcp.c \
Middlewares/Third_Party/LwIP/src/core/def.c \
Middlewares/Third_Party/LwIP/src/core/dns.c \
Middlewares/Third_Party/LwIP/src/core/inet_chksum.c \
Middlewares/Third_Party/LwIP/src/core/init.c \
Middlewares/Third_Party/LwIP/src/core/ip.c \
Middlewares/Third_Party/LwIP/src/core/mem.c \
Middlewares/Third_Party/LwIP/src/core/memp.c \
Middlewares/Third_Party/LwIP/src/core/netif.c \
Middlewares/Third_Party/LwIP/src/core/pbuf.c \
Middlewares/Third_Party/LwIP/src/core/raw.c \
Middlewares/Third_Party/LwIP/src/core/stats.c \
Middlewares/Third_Party/LwIP/src/core/sys.c \
Middlewares/Third_Party/LwIP/src/core/tcp.c \
Middlewares/Third_Party/LwIP/src/core/tcp_in.c \
Middlewares/Third_Party/LwIP/src/core/tcp_out.c \
Middlewares/Third_Party/LwIP/src/core/timeouts.c \
Middlewares/Third_Party/LwIP/src/core/udp.c \
Middlewares/Third_Party/LwIP/src/core/ipv4/autoip.c \
Middlewares/Third_Party/LwIP/src/core/ipv4/dhcp.c \
Middlewares/Third_Party/LwIP/src/core/ipv4/etharp.c \
Middlewares/Third_Party/LwIP/src/core/ipv4/icmp.c \
Middlewares/Third_Party/LwIP/src/core/ipv4/igmp.c \
Middlewares/Third_Party/LwIP/src/core/ipv4/ip4.c \
Middlewares/Third_Party/LwIP/src/core/ipv4/ip4_addr.c \
Middlewares/Third_Party/LwIP/src/core/ipv4/ip4_frag.c \
Middlewares/Third_Party/LwIP/src/core/ipv6/dhcp6.c \
Middlewares/Third_Party/LwIP/src/core/ipv6/ethip6.c \
Middlewares/Third_Party/LwIP/src/core/ipv6/icmp6.c \
Middlewares/Third_Party/LwIP/src/core/ipv6/inet6.c \
Middlewares/Third_Party/LwIP/src/core/ipv6/ip6.c \
Middlewares/Third_Party/LwIP/src/core/ipv6/ip6_addr.c \
Middlewares/Third_Party/LwIP/src/core/ipv6/ip6_frag.c \
Middlewares/Third_Party/LwIP/src/core/ipv6/mld6.c \
Middlewares/Third_Party/LwIP/src/core/ipv6/nd6.c \
Middlewares/Third_Party/LwIP/system/OS/sys_arch.c 
# Middlewares/Third_Party/LwIP/src/apps/mqtt/mqtt.c \
# Middlewares/Third_Party/LwIP/src/apps/snmp/snmpv3.c \
# Middlewares/Third_Party/LwIP/src/apps/snmp/snmpv3_mbedtls.c \
# Middlewares/Third_Party/LwIP/src/apps/snmp/snmp_asn1.c \
# Middlewares/Third_Party/LwIP/src/apps/snmp/snmp_core.c \
# Middlewares/Third_Party/LwIP/src/apps/snmp/snmp_mib2.c \
# Middlewares/Third_Party/LwIP/src/apps/snmp/snmp_mib2_icmp.c \
# Middlewares/Third_Party/LwIP/src/apps/snmp/snmp_mib2_interfaces.c \
# Middlewares/Third_Party/LwIP/src/apps/snmp/snmp_mib2_ip.c \
# Middlewares/Third_Party/LwIP/src/apps/snmp/snmp_mib2_snmp.c \
# Middlewares/Third_Party/LwIP/src/apps/snmp/snmp_mib2_system.c \
# Middlewares/Third_Party/LwIP/src/apps/snmp/snmp_mib2_tcp.c \
# Middlewares/Third_Party/LwIP/src/apps/snmp/snmp_mib2_udp.c \
# Middlewares/Third_Party/LwIP/src/apps/snmp/snmp_msg.c \
# Middlewares/Third_Party/LwIP/src/apps/snmp/snmp_netconn.c \
# Middlewares/Third_Party/LwIP/src/apps/snmp/snmp_pbuf_stream.c \
# Middlewares/Third_Party/LwIP/src/apps/snmp/snmp_raw.c \
# Middlewares/Third_Party/LwIP/src/apps/snmp/snmp_scalar.c \
# Middlewares/Third_Party/LwIP/src/apps/snmp/snmp_snmpv2_framework.c \
# Middlewares/Third_Party/LwIP/src/apps/snmp/snmp_snmpv2_usm.c \
# Middlewares/Third_Party/LwIP/src/apps/snmp/snmp_table.c \
# Middlewares/Third_Party/LwIP/src/apps/snmp/snmp_threadsync.c \
# Middlewares/Third_Party/LwIP/src/apps/snmp/snmp_traps.c \
# Middlewares/Third_Party/LwIP/src/apps/sntp/sntp.c \
# Middlewares/Third_Party/LwIP/src/apps/smtp/smtp.c

# Include folders common to all targets
INC_FOLDERS +=  \
  application \
  application/config \
  application/debug_log \
  application/flight_app \
  application/flight_app/esc_dfu \
  application/flight_app/esc_dfu/stm32_uart_dfu \
  application/freertos \
  application/location \
  application/led_blinky \
  application/network \
  application/network/net_if \
  application/network/net_init \
  application/network/os_hook \
  application/network/wifi_ex \
  application/network/oq2_protocol \
  application/pid \
  application/stability_control \
  application/stability_control/kinematics \
  application/stability_control/platform \
  application/stability_control/motors \
  application/sys \
  application/task_manager \
  application/timer \
  application/utility \
  FATFS/Target \
  FATFS/App \
  USB_DEVICE/App \
  USB_DEVICE/Target \
  Middlewares/Third_Party/FreeRTOS/Source/include \
  Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 \
  Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 \
  Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F \
  Middlewares/Third_Party/FatFs/src \
  Middlewares/ST/STM32_USB_Device_Library/Core/Inc \
  Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Inc \
  Drivers/CMSIS/Device/ST/STM32H7xx/Include \
  Drivers/CMSIS/Include \
  Drivers/Mems/lps22hh_STdC/driver \
  Drivers/Mems/lsm9ds1_STdC/driver \
  Drivers/STM32H7xx_HAL_Driver/Inc \
  Drivers/STM32H7xx_HAL_Driver/Inc/Legacy \
  Middlewares/ST/STM32_MotionFX_Library/Inc \
  Middlewares/ARM/DSP/Inc \
  Drivers/wifi_drv_winc3400/driver/include \
  Drivers/wifi_drv_winc3400/driver/source \
  Drivers/wifi_drv_winc3400/bsp/include \
  Drivers/wifi_drv_winc3400/bus_wrapper/include \
  Drivers/wifi_drv_winc3400/common/include \
  Drivers/wifi_drv_winc3400/socket/include \
  Drivers/wifi_drv_winc3400/socket/source \
  Middlewares/Third_Party/LwIP/src/include \
  Middlewares/Third_Party/LwIP/system \
  Middlewares/Third_Party/LwIP/src/include/netif/ppp \
  Middlewares/Third_Party/LwIP/src/include/lwip \
  Middlewares/Third_Party/LwIP/src/include/lwip/apps \
  Middlewares/Third_Party/LwIP/src/include/lwip/priv \
  Middlewares/Third_Party/LwIP/src/include/lwip/prot \
  Middlewares/Third_Party/LwIP/src/include \
  Middlewares/Third_Party/LwIP/src/include/netif \
  Middlewares/Third_Party/LwIP/src/include/compat/posix \
  Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa \
  Middlewares/Third_Party/LwIP/src/include/compat/posix/net \
  Middlewares/Third_Party/LwIP/src/include/compat/posix/sys \
  Middlewares/Third_Party/LwIP/src/include/compat/stdc \
  Middlewares/Third_Party/LwIP/system/arch

LIB_FILES += Middlewares/ST/STM32_MotionFX_Library/Lib/MotionFX_CM7F_wc32_ot_hard.a
LIB_FILES += Middlewares/ARM/DSP/Lib/libarm_cortexM7lfsp_math.a

# debug build?
DEBUG = 1
# optimization
OPT = -O0
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m7

# fpu
FPU = -mfpu=fpv5-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DUSE_HAL_DRIVER \
-DSTM32H735xx \
-DFREERTOS \
-DARM_MATH_CM7 \
-D_FLASH_STORAGE_=0x080C0000 \
-D_FLASH_STORAGE_SIZE_=0x40000

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
CFLAGS += $(C_DEFS)
CFLAGS += $(MCU)
CLFAGS += -Wall
CFLAGS += -fdata-sections
CFLAGS += -ffunction-sections
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"
ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif

# linker flags
LDFLAGS = $(MCU) -specs=nano.specs -T$(LINKER_SCRIPT) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# Add standard libraries at the very end of the linker input, after all objects
# that may need symbols provided by these libraries.
LIB_FILES += -lc -lnosys -lm

.PHONY: default help

# Default target - first one defined
default: $(TARGET)

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
	jlink -SelectEmuBySN 801020691 -device STM32H735ZGT -if SWD -speed 16000 -autoconnect 1 -ExitOnError -CommanderScript ./jlink/erase.jlink

flash: default
	jlink -SelectEmuBySN 801020691 -device STM32H735ZGT -if SWD -speed 16000 -autoconnect 1 -ExitOnError -CommanderScript ./jlink/flash.jlink

flash_wholedevice:
	@echo Done Compiling
	@echo Merging
	@python merger.py
	@echo Flashing
	jlink -SelectEmuBySN 801020691 -device STM32H735ZGT -if SWD -speed 16000 -autoconnect 1 -ExitOnError -CommanderScript ./jlink/flash_esc_fw.jlink


reset:
	jlink -SelectEmuBySN 801020691 -device STM32H735ZGT -if SWD -speed 16000 -autoconnect 1 -ExitOnError -CommanderScript ./jlink/reset.jlink

#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***
