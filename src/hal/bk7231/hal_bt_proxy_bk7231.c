#if PLATFORM_BEKEN_NEW

#include "../../new_common.h"
#include "../../new_pins.h"
#include "../../new_cfg.h"
#include "../hal_bt_proxy.h"
#include "../../logging/logging.h"
#include "../../driver/drv_esphome_api.h"
#include <string.h>
#include <stdio.h>

#include "include.h"

#if ENABLE_BT_PROXY || ENABLE_DRIVER_BTHOME

#include "ble_api.h"
#if ENABLE_DRIVER_BTHOME
#include "app_ble.h"
#endif

#if !PLATFORM_BK7238
#include "common_bt_defines.h"
extern struct bd_addr common_default_bdaddr;
#endif

typedef struct
{
	bool init_done;
	bool scan_active;
	int scan_total_packets;
	int scan_dropped_packets;

	//QueueHandle_t cmd_queue;
	TaskHandle_t task_handle;
} hal_bt_state_t;

extern void WiFI_GetMacAddress(char* mac);
extern unsigned int g_timeMs;
static hal_bt_state_t g_bt_proxy = { 0 };

static int g_bleStackReady = 0;
static uint8_t g_bleScanActvIdx = 0xFF;
static uint16_t g_bleScanInterval = 100;
static uint16_t g_bleScanWindow = 30;
static bool scan_mode = false; // passive - false, active - true

#if ENABLE_DRIVER_BTHOME
// The SDK has one global command callback. Serialize SDK submissions, and let
// only the platform tick consume requests; callbacks must never chain SDK calls.
static beken_mutex_t s_advMutex;
static bool s_advMutexReady;
static bool s_advMutexInitializing;
static volatile bool s_bleReadyNotice;
static struct adv_param s_advRequest;
static bool s_advWanted;
static bool s_advPending;
static int s_advState;
static int s_advError;
static uint8_t s_advIndex = 0xFF;
static unsigned int s_advGeneration;
static unsigned int s_advAppliedGeneration;
static unsigned int s_advSubmittedGeneration;
static unsigned int s_advCommandTime;
static unsigned int s_advWaitTime;
static unsigned int s_advCompleted;
static unsigned int s_advFailures;
static bool s_advBusy;

static bool HAL_BT_AdvEnsureMutex(void)
{
	OSStatus result;
	GLOBAL_INT_DECLARATION();
	GLOBAL_INT_DISABLE();
	if (s_advMutexReady || s_advMutexInitializing) {
		bool ready = s_advMutexReady;
		GLOBAL_INT_RESTORE();
		return ready;
	}
	s_advMutexInitializing = true;
	GLOBAL_INT_RESTORE();
	// Allocation stays in task context. A concurrent first caller fails closed
	// until the single construction completes, rather than using another mutex.
	result = rtos_init_mutex(&s_advMutex);
	GLOBAL_INT_DISABLE();
	s_advMutexReady = result == kNoErr;
	s_advMutexInitializing = false;
	GLOBAL_INT_RESTORE();
	if (result != kNoErr) {
		ADDLOG_ERROR(LOG_FEATURE_DRV, "BLE adv: mutex allocation failed error=%d", result);
	}
	return result == kNoErr;
}

static void HAL_BT_AdvCommandComplete(ble_cmd_t cmd, ble_cmd_param_t *param)
{
	if (!param || !s_advMutexReady) {
		return;
	}
	rtos_lock_mutex(&s_advMutex);
	if (!s_advPending || param->cmd_idx != s_advIndex) {
		rtos_unlock_mutex(&s_advMutex);
		return;
	}
	s_advPending = false;
	s_advBusy = false;
	if (param->status != ERR_SUCCESS) {
		s_advError = param->status;
		s_advState = -1;
		s_advWanted = false;
		s_advFailures++;
		ADDLOG_ERROR(LOG_FEATURE_DRV, "BLE adv: asynchronous failure=%d; fault latched, activity ownership retained, reboot required before reuse", param->status);
	} else if (cmd == BLE_INIT_ADV) {
		s_advState = 2;
		s_advAppliedGeneration = s_advSubmittedGeneration;
		s_advCompleted++;
	} else if (cmd == BLE_DEINIT_ADV) {
		s_advState = 0;
		s_advIndex = 0xFF;
	} else {
		s_advError = -2;
		s_advState = -1;
		s_advWanted = false;
		ADDLOG_ERROR(LOG_FEATURE_DRV, "BLE adv: unexpected completion=%d; fault latched, reboot required before reuse", cmd);
	}
	ADDLOG_DEBUG(LOG_FEATURE_DRV, "BLE adv: completion cmd=%d state=%d status=%d completed=%u errors=%u elapsed=%ums", cmd, s_advState, param->status, s_advCompleted, s_advFailures, g_timeMs - s_advCommandTime);
	rtos_unlock_mutex(&s_advMutex);
}

int HAL_BT_AdvSubmit(const uint8_t *data, int length, uint16_t interval)
{
	if (!data || length < 1 || length > 31 || interval < 160 || interval > 16384) {
		return -1;
	}
	if (!HAL_BT_AdvEnsureMutex()) {
		return -1;
	}
	rtos_lock_mutex(&s_advMutex);
	if (s_advState < 0 || s_advError == -3 || g_bt_proxy.scan_active) {
		int result = s_advState < 0 || s_advError == -3 ? -1 : 0;
		rtos_unlock_mutex(&s_advMutex);
		return result;
	}
	// The request remains owned here through stop/restart. SDK start copies it
	// synchronously, so the common driver's stack buffer can disappear immediately.
	memset(&s_advRequest, 0, sizeof(s_advRequest));
	memcpy(s_advRequest.advData, data, length);
	s_advRequest.advDataLen = length;
	s_advRequest.channel_map = 7;
	s_advRequest.interval_min = interval;
	s_advRequest.interval_max = interval;
	s_advRequest.duration = 0;
	s_advRequest.prop = 0; // Legacy, nonconnectable and nonscannable, public identity.
	if (!s_advWanted) {
		s_advWaitTime = g_timeMs;
	}
	s_advWanted = true;
	s_advGeneration++;
	rtos_unlock_mutex(&s_advMutex);
	return 1;
}

void HAL_BT_AdvStop(void)
{
	if (!s_advMutexReady) {
		return;
	}
	rtos_lock_mutex(&s_advMutex);
	s_advWanted = false;
	s_advWaitTime = g_timeMs;
	rtos_unlock_mutex(&s_advMutex);
}

int HAL_BT_AdvGetState(int *error)
{
	int state;
	if (!s_advMutexReady) {
		if (error) {
			*error = 0;
		}
		return 0;
	}
	rtos_lock_mutex(&s_advMutex);
	state = s_advState;
	if (error) {
		*error = s_advError;
	}
	rtos_unlock_mutex(&s_advMutex);
	return state;
}
#endif

static void hal_bt_gap_callback(ble_notice_t notice, void* param)
{
#if ENABLE_DRIVER_BTHOME
	// Stack-ready notices may have no parameter; process them before the old guard.
	if (notice == BLE_5_STACK_OK) {
		s_bleReadyNotice = true;
	}
#endif
	if(!param) return;
#if ENABLE_BT_PROXY && ENABLE_DRIVER_ESPHOME_API
	if(notice == BLE_5_REPORT_ADV)
	{
		recv_adv_t* info = (recv_adv_t*)param;
		g_bt_proxy.scan_total_packets++;
		if(!ESPHome_API_PassScanResult(info->adv_addr, (int)(int8_t)info->rssi, info->adv_addr_type, info->data, info->data_len))
		{
			g_bt_proxy.scan_dropped_packets++;
		}
	}
#endif
}

static void HAL_BTScan_EnsureStackReady()
{
	if(g_bleStackReady)
	{
		return;
	}

	extern void ble_entry(void);
	ble_set_notice_cb(hal_bt_gap_callback);
	ble_entry();
	g_bleStackReady = 1;
	ADDLOG_INFO(LOG_FEATURE_GENERAL, "BLE scan stack initialized");
}

void HAL_BTProxy_StartScan()
{
#if ENABLE_DRIVER_BTHOME
	if (!HAL_BT_AdvEnsureMutex()) {
		return;
	}
	rtos_lock_mutex(&s_advMutex);
	if (s_advWanted || s_advPending || s_advIndex != 0xFF) {
		rtos_unlock_mutex(&s_advMutex);
		ADDLOG_ERROR(LOG_FEATURE_GENERAL, "BLE scan: BTHome owns advertising activity; stop it first");
		return;
	}
#endif
	if(!g_bt_proxy.init_done) {
#if ENABLE_DRIVER_BTHOME
		if (s_advMutexReady) {
			rtos_unlock_mutex(&s_advMutex);
		}
#endif
		return;
	}
	struct scan_param scan;
	ble_err_t ret;
	if(g_bt_proxy.scan_active)
	{
		ADDLOG_INFO(LOG_FEATURE_GENERAL, "Scan is processing, please stop it first");
	}
	else
	{
		HAL_BTScan_EnsureStackReady();
		memset(&scan, 0, sizeof(scan));
		//scan.filter_en = SCAN_FILT_DUPLIC_EN;
		scan.channel_map = 7;
		scan.interval = g_bleScanInterval;
		scan.window = g_bleScanWindow;
		scan.active = (uint8_t)scan_mode;

		g_bleScanActvIdx = app_ble_get_idle_actv_idx_handle(SCAN_ACTV);
		if(g_bleScanActvIdx == 0xFF)
		{
			ADDLOG_ERROR(LOG_FEATURE_GENERAL, "BLE scan failed - no idle activity");
#if ENABLE_DRIVER_BTHOME
			if (s_advMutexReady) {
				rtos_unlock_mutex(&s_advMutex);
			}
#endif
			return;
		}

		ret = bk_ble_scan_start(g_bleScanActvIdx, &scan, NULL);
		if(ret != ERR_SUCCESS)
		{
			ADDLOG_ERROR(LOG_FEATURE_GENERAL, "BLE scan start failed (err %d)", ret);
			g_bleScanActvIdx = 0xFF;
#if ENABLE_DRIVER_BTHOME
			if (s_advMutexReady) {
				rtos_unlock_mutex(&s_advMutex);
			}
#endif
			return;
		}

		g_bt_proxy.scan_active = true;
	}
#if ENABLE_DRIVER_BTHOME
	if (s_advMutexReady) {
		rtos_unlock_mutex(&s_advMutex);
	}
#endif
}

void HAL_BTProxy_StopScan()
{
#if ENABLE_DRIVER_BTHOME
	if (!HAL_BT_AdvEnsureMutex()) {
		return;
	}
	rtos_lock_mutex(&s_advMutex);
#endif
	if(!g_bt_proxy.init_done) {
#if ENABLE_DRIVER_BTHOME
		if (s_advMutexReady) {
			rtos_unlock_mutex(&s_advMutex);
		}
#endif
		return;
	}
	ble_err_t ret;
	if(g_bt_proxy.scan_active)
	{
		ret = bk_ble_scan_stop(g_bleScanActvIdx, NULL);
		if(ret != ERR_SUCCESS)
		{
			ADDLOG_ERROR(LOG_FEATURE_GENERAL, "BLE scan stop failed (err %d)", ret);
#if ENABLE_DRIVER_BTHOME
			if (s_advMutexReady) {
				rtos_unlock_mutex(&s_advMutex);
			}
#endif
			return;
		}
		g_bleScanActvIdx = 0xFF;
		g_bt_proxy.scan_active = false;
	}
	else
	{
		ADDLOG_INFO(LOG_FEATURE_GENERAL, "There is no scan");
	}
#if ENABLE_DRIVER_BTHOME
	if (s_advMutexReady) {
		rtos_unlock_mutex(&s_advMutex);
	}
#endif
}

void HAL_BTProxy_Init(void)
{
#if ENABLE_DRIVER_BTHOME
	if (!HAL_BT_AdvEnsureMutex()) {
		return;
	}
	rtos_lock_mutex(&s_advMutex);
#endif
	if(g_bt_proxy.init_done)
	{
#if ENABLE_DRIVER_BTHOME
		rtos_unlock_mutex(&s_advMutex);
#endif
		return;
	}

	HAL_BTScan_EnsureStackReady();

	g_bt_proxy.init_done = true;
#if ENABLE_DRIVER_BTHOME
	rtos_unlock_mutex(&s_advMutex);
#endif
}

void HAL_BTProxy_Deinit(void)
{
	HAL_BTProxy_StopScan();

	//g_bt_proxy.init_done = false;
}

void HAL_BTProxy_GetMAC(uint8_t* mac)
{
	uint8_t* btmac = &common_default_bdaddr.addr[0];
	for(int i = 0; i < 6; i++)
	{
		mac[i] = btmac[5 - i];
	}
	uint8_t zero_mac[6] = { 0 };
	uint8_t ff_mac[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
	if(memcmp(mac, zero_mac, 6) == 0 || memcmp(mac, ff_mac, 6) == 0)
	{
		WiFI_GetMacAddress((char*)mac);
	}
}

void HAL_BTProxy_SetScanMode(bool isActive)
{
	scan_mode = isActive;
	HAL_BTProxy_StopScan();
	delay_ms(1);
	HAL_BTProxy_StartScan();
}

bool HAL_BTProxy_GetScanMode(void)
{
	return scan_mode;
}

void HAL_BTProxy_SetWindowInterval(uint16_t window, uint16_t interval)
{
	g_bleScanInterval = interval;
	g_bleScanWindow = window;
	HAL_BTProxy_StopScan();
	delay_ms(1);
	HAL_BTProxy_StartScan();
}

bool HAL_BTProxy_IsInit(void)
{
	return g_bt_proxy.init_done;
}

void HAL_BTProxy_OnEverySecond(void)
{
	// add check if there were no scan results in 60 seconds, if so - restart scan
#if ENABLE_DRIVER_BTHOME
	ble_err_t result;
	if (!s_advMutexReady) {
		return;
	}
	rtos_lock_mutex(&s_advMutex);
	if (s_advPending) {
		if (g_timeMs - s_advCommandTime >= 10000 && s_advError != -3) {
			// A timeout cannot release an SDK-owned activity or its late callback.
			s_advError = -3;
			s_advWanted = false;
			s_advFailures++;
			ADDLOG_ERROR(LOG_FEATURE_DRV, "BLE adv: command timeout state=%d; retaining pending ownership errors=%u", s_advState, s_advFailures);
		}
		rtos_unlock_mutex(&s_advMutex);
		return;
	}
	if (s_advState < 0 || (!s_advWanted && s_advState == 0)) {
		rtos_unlock_mutex(&s_advMutex);
		return;
	}
	HAL_BTScan_EnsureStackReady();
	// SDK startup may initialize BLE before this driver binds the notice callback.
	// READY is 1 (IDLE is 0), so querying it cannot confuse zeroed startup state.
	if (!s_bleReadyNotice && app_ble_env_state_get() == APP_BLE_READY) {
		s_bleReadyNotice = true;
	}
	if (!s_bleReadyNotice) {
		if (g_timeMs - s_advWaitTime >= 10000) {
			s_advError = -4;
			s_advState = -1;
			s_advWanted = false;
			ADDLOG_ERROR(LOG_FEATURE_DRV, "BLE adv: stack readiness timed out; fault latched, reboot required before reuse");
		}
		rtos_unlock_mutex(&s_advMutex);
		return;
	}
	if (s_advState == 2 && s_advWanted && s_advAppliedGeneration == s_advGeneration) {
		rtos_unlock_mutex(&s_advMutex);
		return;
	}
	if (s_advState == 2) {
		s_advState = 3;
		s_advPending = true;
		s_advCommandTime = g_timeMs;
		result = bk_ble_adv_stop(s_advIndex, HAL_BT_AdvCommandComplete);
	} else {
		s_advIndex = app_ble_get_idle_actv_idx_handle(ADV_ACTV);
		if (s_advIndex == 0xFF || s_advIndex >= BLE_ACTIVITY_MAX) {
			s_advError = -5;
			s_advState = -1;
			s_advWanted = false;
			ADDLOG_ERROR(LOG_FEATURE_DRV, "BLE adv: no idle advertising activity; fault latched, reboot required before reuse");
			rtos_unlock_mutex(&s_advMutex);
			return;
		}
		s_advState = 1;
		s_advPending = true;
		s_advSubmittedGeneration = s_advGeneration;
		s_advCommandTime = g_timeMs;
		if (app_ble_env_state_get() != APP_BLE_READY) {
			result = ERR_BLE_STATUS;
		} else {
			// bk_ble_adv_start always sets scan-response data, which the SDK rejects
			// for this nonscannable advertiser. Reuse its existing compound engine
			// and owned parameter copy, omitting only that invalid operation.
			uint32_t operations = (1U << BLE_OP_CREATE_ADV_POS)
				| (1U << BLE_OP_SET_ADV_DATA_POS)
				| (1U << BLE_OP_START_ADV_POS);
			app_ble_run(s_advIndex, BLE_INIT_ADV, operations, HAL_BT_AdvCommandComplete);
			memcpy(&app_ble_env.actvs[s_advIndex].param.adv, &s_advRequest, sizeof(s_advRequest));
			result = app_ble_create_advertising(s_advIndex, &s_advRequest);
			if (result != ERR_SUCCESS) {
				app_ble_reset();
			}
		}
	}
	if (result != ERR_SUCCESS) {
		s_advPending = false;
		s_advState = s_advState == 3 ? 2 : 0;
		if (s_advState == 0) {
			s_advIndex = 0xFF;
		}
		s_advError = result;
		if (!s_advBusy) {
			s_advWaitTime = g_timeMs;
			s_advBusy = true;
			ADDLOG_DEBUG(LOG_FEATURE_DRV, "BLE adv: waiting for SDK command ownership error=%d", result);
		}
		if (result != ERR_BLE_STATUS || g_timeMs - s_advWaitTime >= 10000) {
			s_advState = -1;
			s_advWanted = false;
			s_advFailures++;
			ADDLOG_ERROR(LOG_FEATURE_DRV, "BLE adv: submission failed error=%d errors=%u; fault latched, reboot required before reuse", result, s_advFailures);
		}
	} else {
		s_advBusy = false;
		s_advError = 0;
		s_advWaitTime = g_timeMs;
		ADDLOG_DEBUG(LOG_FEATURE_DRV, "BLE adv: submitted state=%d activity=%u generation=%u", s_advState, s_advIndex, s_advGeneration);
	}
	rtos_unlock_mutex(&s_advMutex);
#endif
}

int HAL_BTProxy_GetScanStats(int* init_done, int* scan_active, int* total_packets, int* dropped_packets)
{
	if(init_done) *init_done = g_bt_proxy.init_done;
	if(scan_active) *scan_active = g_bt_proxy.scan_active;
	if(total_packets) *total_packets = g_bt_proxy.scan_total_packets;
	if(dropped_packets) *dropped_packets = g_bt_proxy.scan_dropped_packets;
	return 1;
}

#endif

#endif
