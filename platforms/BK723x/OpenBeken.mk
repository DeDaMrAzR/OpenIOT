OBK_DIR = ./app

INCLUDES += -I$(OBK_DIR)/../include
INCLUDES += -I./fixes

CCFLAGS += -DPLATFORM_BEKEN -DPLATFORM_BEKEN_NEW -D__FILE__=\"\" -Wno-builtin-macro-redefined

ifeq ($(CFG_SOC_NAME), 1)
CCFLAGS += -DPLATFORM_BK7231T
else ifeq ($(CFG_SOC_NAME), 2)
ifeq ($(SOC_BK7231T), 1)
CCFLAGS += -DPLATFORM_BK7231T
else
CCFLAGS += -DPLATFORM_BK7231U
CCFLAGS += -DENABLE_DRIVER_MDNS=1 -DLWIP_MDNS_RESPONDER=1 -DLWIP_NUM_NETIF_CLIENT_DATA=1
endif
else ifeq ($(CFG_SOC_NAME), 3)
CCFLAGS += -DPLATFORM_BK7252
CCFLAGS += -DENABLE_DRIVER_MDNS=1 -DLWIP_MDNS_RESPONDER=1 -DLWIP_NUM_NETIF_CLIENT_DATA=1
else ifeq ($(CFG_SOC_NAME), 5)
CCFLAGS += -DPLATFORM_BK7231N
else ifeq ($(CFG_SOC_NAME), 7)
CCFLAGS += -DPLATFORM_BK7238
else ifeq ($(CFG_SOC_NAME), 8)
CCFLAGS += -DPLATFORM_BK7252N
CCFLAGS += -DENABLE_DRIVER_MDNS=1 -DLWIP_MDNS_RESPONDER=1 -DLWIP_NUM_NETIF_CLIENT_DATA=1
endif

SRC_C += ./fixes/blank.c
APP_C += $(OBK_DIR)/../platforms/BK723x/ps.c

APP_C += $(OBK_DIR)/hal/bk7231/hal_adc_bk7231.c
APP_C += $(OBK_DIR)/hal/bk7231/hal_flashConfig_bk7231.c
APP_C += $(OBK_DIR)/hal/bk7231/hal_flashVars_bk7231.c
APP_C += $(OBK_DIR)/hal/bk7231/hal_generic_bk7231.c
APP_C += $(OBK_DIR)/hal/bk7231/hal_hwtimer_bk7231.c
APP_C += $(OBK_DIR)/hal/bk7231/hal_main_bk7231.c
APP_C += $(OBK_DIR)/hal/bk7231/hal_pins_bk7231.c
APP_C += $(OBK_DIR)/hal/bk7231/hal_wifi_bk7231.c
APP_C += $(OBK_DIR)/hal/bk7231/hal_uart_bk7231.c
APP_C += $(OBK_DIR)/hal/bk7231/hal_ota_bk7231.c
APP_C += $(OBK_DIR)/hal/bk7231/hal_bt_proxy_bk7231.c

OBK_SRCS = $(OBK_DIR)/
include $(OBK_DIR)/../platforms/obk_main.mk
APP_C += $(OBKM_SRC)
APP_CXX += $(OBKM_SRC_CXX)
CCFLAGS += $(OBK_CFLAGS)


APP_C += $(OBK_DIR)/../libraries/easyflash/ports/ef_port.c
APP_C += $(OBK_DIR)/../libraries/easyflash/src/easyflash.c
APP_C += $(OBK_DIR)/../libraries/easyflash/src/ef_env.c
APP_C += $(OBK_DIR)/../libraries/easyflash/src/ef_utils.c
INCLUDES += -I$(OBK_DIR)/../libraries/easyflash/inc

BERRY_MODULEPATH = $(OBK_DIR)/berry/modules
BERRY_SRCPATH = $(OBK_DIR)/../libraries/berry/src
include $(OBK_DIR)/../libraries/berry.mk

APP_C += $(BERRY_SRC_C)

# Compiler flags are not prerequisites in the SDK. Track the effective version
# after all application sources are collected, including C++ and Berry sources.
OBK_APP_VERSION_STAMP := $(OBJ_DIR)/.obk_app_version
ifdef USER_SW_VER
export OBK_APP_VERSION_STAMP_VALUE := $(USER_SW_VER)
else
export OBK_APP_VERSION_STAMP_VALUE := $(APP_VERSION)
endif

# Snapshot before the stamp recipe updates it. A conditional phony prerequisite
# handles equal/coarse timestamps without deleting objects whose mtimes make may
# already have cached. Reading the prior stamp has no parse-time side effects.
OBK_APP_VERSION_CHANGED := $(shell if [ ! -f "$(OBK_APP_VERSION_STAMP)" ] || ! printf '%s\n' '$(subst ','"'"',$(OBK_APP_VERSION_STAMP_VALUE))' | cmp -s - "$(OBK_APP_VERSION_STAMP)"; then printf '%s' 1; fi)

.PHONY: obk_app_version_check
obk_app_version_check:

$(OBK_APP_VERSION_STAMP): obk_app_version_check
	@set -eu; \
	mkdir -p "$(OBJ_DIR)"; \
	stamp_tmp="$@.tmp.$$$$"; \
	trap 'rm -f "$$stamp_tmp"' 0; \
	trap 'exit 129' HUP; trap 'exit 130' INT; trap 'exit 143' TERM; \
	printf '%s\n' "$$OBK_APP_VERSION_STAMP_VALUE" > "$$stamp_tmp"; \
	if [ ! -f "$@" ] || ! cmp -s "$$stamp_tmp" "$@"; then \
		mv -f "$$stamp_tmp" "$@"; \
	else \
		rm -f "$$stamp_tmp"; \
	fi

# The shared stamp executes once before app compiles under parallel make. Only
# a changed/missing version adds a force dependency for this invocation; SDK
# libraries and final firmware artifacts are not included in this rule.
$(OBJ_APP_LIST) $(OBJ_CXX_LIST): $(OBK_APP_VERSION_STAMP)
ifeq ($(OBK_APP_VERSION_CHANGED),1)
.PHONY: obk_app_version_changed
obk_app_version_changed:
$(OBJ_APP_LIST) $(OBJ_CXX_LIST): obk_app_version_changed
endif
