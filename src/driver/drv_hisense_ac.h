#ifndef __DRV_HISENSE_AC_H__
#define __DRV_HISENSE_AC_H__

#include "../obk_config.h"

#if ENABLE_DRIVER_HISENSE_AC

#include "../new_common.h"
#include "../httpserver/new_http.h"

typedef enum {
	HISENSE_AC_PROFILE_UNKNOWN = 0,
	HISENSE_AC_PROFILE_009_104 = 104
} hisense_ac_profile_t;

typedef struct {
	int uart_index;
	int running;
	int profile_id;

	uint32_t last_rx_tick;
	uint32_t last_tx_tick;
	uint32_t last_quicktick_tick;
	uint32_t last_housekeeping_tick;

	uint32_t parser_frames_seen;
	uint32_t parser_frames_classified;
	uint32_t parser_frames_rejected;
	uint32_t parser_rx_bytes_seen;
	uint32_t parser_tx_frames_requested;

	int power_state;
	int mode_state;
	int fan_state;
	int swing_state;
	int sleep_state;
	int eco_state;
	int fan_mute_state;
	int super_state;
	int temp_unit_state;
	int fault_state;

	float target_temp;
	float current_temp;
	float pipe_temp;
	float daily_energy_kwh;
} hisense_ac_ctx_t;

void HisenseAC_Init(void);
void HisenseAC_Shutdown(void);
void HisenseAC_OnEverySecond(void);
void HisenseAC_RunFrame(void);
void HisenseAC_AppendInformationToHTTPIndexPage(http_request_t *request, int bPreState);
void HisenseAC_OnHassDiscovery(const char *topic);

#endif

#endif
