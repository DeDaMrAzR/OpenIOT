#include "../obk_config.h"

#if ENABLE_DRIVER_HISENSE_AC

#include "../new_common.h"
#include "../new_cfg.h"
#include "../cmnds/cmd_public.h"
#include "../logging/logging.h"
#include "../httpserver/new_http.h"
#include "drv_public.h"
#include "drv_local.h"
#include "drv_uart.h"
#include "drv_hisense_ac.h"

/*
	* HisenseAC driver shell for OpenBeken.
	*
	* Skeleton scope only:
	* - driver registration and lifecycle hooks
	* - UART ownership shell
	* - placeholder command surface
	* - HTTP status stub
	*
	* Confirmed 009-104 protocol families for future implementation:
	* - TX 0C / 66 / 00 : main status poll
	* - TX 13 / 1E / 00 : short probe / keepalive query
	* - TX 29 / 65 / 00 : control update
	* - RX 7B / 65 / 00 : control ack
	* - RX 97 / 66 / 00 : main status reply
	*
	* Climate-first channel plan for later v1 implementation:
	* - power
	* - mode
	* - target temp
	* - current temp
	* - fan
	* - swing
	* - sleep
	* - eco
	* - fan mute
	* - super
	* - temp unit
	* - energy
	* - faults
	*
	* Intentionally out of scope in this shell:
	* - real frame parsing
	* - checksum validation
	* - active poll loop
	* - state decoding
	* - write builders
	* - MQTT / HA discovery payloads
	*/

static hisense_ac_ctx_t g_hisense_ctx;
static int g_hisense_uart = 1;
static int g_hisense_commands_registered = 0;

static void HisenseAC_ReinitUart(void);
static void HisenseAC_SendStatusPollStub(void);
static void HisenseAC_SendShortProbeStub(void);
static void HisenseAC_ProcessRxByteStub(byte data);
static void HisenseAC_ClassifyFrameStub(const byte *frame, int len);
static void HisenseAC_PublishStateStub(void);
static void HisenseAC_RegisterCommands(void);
static commandResult_t HisenseAC_Cmd_NotImplemented(const void *context, const char *cmd, const char *args, int cmdFlags);
static commandResult_t HisenseAC_Cmd_Uart(const void *context, const char *cmd, const char *args, int cmdFlags);
static commandResult_t HisenseAC_Cmd_Status(const void *context, const char *cmd, const char *args, int cmdFlags);

static void HisenseAC_ReinitUart(void) {
	UART_InitUARTEx(g_hisense_uart, 9600, 0, false);
	UART_InitReceiveRingBufferEx(g_hisense_uart, 256);
	g_hisense_ctx.uart_index = g_hisense_uart;
	addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "HisenseAC: UART shell initialized on UART%i at 9600 8N1", g_hisense_uart);
}

static void HisenseAC_SendStatusPollStub(void) {
	g_hisense_ctx.parser_tx_frames_requested++;
	g_hisense_ctx.last_tx_tick = xTaskGetTickCount();
}

static void HisenseAC_SendShortProbeStub(void) {
	g_hisense_ctx.parser_tx_frames_requested++;
	g_hisense_ctx.last_tx_tick = xTaskGetTickCount();
}

static void HisenseAC_ProcessRxByteStub(byte data) {
	(void)data;
	g_hisense_ctx.parser_rx_bytes_seen++;
	g_hisense_ctx.last_rx_tick = xTaskGetTickCount();
}

static void HisenseAC_ClassifyFrameStub(const byte *frame, int len) {
	(void)frame;
	(void)len;
	g_hisense_ctx.parser_frames_seen++;
}

static void HisenseAC_PublishStateStub(void) {
	addLogAdv(LOG_INFO, LOG_FEATURE_CMD,
		"HisenseAC shell: profile=%d uart=%d running=%d rxFrames=%u txReq=%u",
		g_hisense_ctx.profile_id,
		g_hisense_ctx.uart_index,
		g_hisense_ctx.running,
		(unsigned int)g_hisense_ctx.parser_frames_seen,
		(unsigned int)g_hisense_ctx.parser_tx_frames_requested);
}

static void HisenseAC_RegisterCommands(void) {
	if (g_hisense_commands_registered) {
		return;
	}
	CMD_RegisterCommand("hisense_uart", HisenseAC_Cmd_Uart, NULL);
	CMD_RegisterCommand("hisense_status", HisenseAC_Cmd_Status, NULL);
	CMD_RegisterCommand("hisense_setPower", HisenseAC_Cmd_NotImplemented, NULL);
	CMD_RegisterCommand("hisense_setMode", HisenseAC_Cmd_NotImplemented, NULL);
	CMD_RegisterCommand("hisense_setTemp", HisenseAC_Cmd_NotImplemented, NULL);
	CMD_RegisterCommand("hisense_setFan", HisenseAC_Cmd_NotImplemented, NULL);
	CMD_RegisterCommand("hisense_setSwing", HisenseAC_Cmd_NotImplemented, NULL);
	CMD_RegisterCommand("hisense_setSleep", HisenseAC_Cmd_NotImplemented, NULL);
	CMD_RegisterCommand("hisense_setEco", HisenseAC_Cmd_NotImplemented, NULL);
	CMD_RegisterCommand("hisense_setFanMute", HisenseAC_Cmd_NotImplemented, NULL);
	CMD_RegisterCommand("hisense_setSuper", HisenseAC_Cmd_NotImplemented, NULL);
	CMD_RegisterCommand("hisense_setTempUnit", HisenseAC_Cmd_NotImplemented, NULL);
	g_hisense_commands_registered = 1;
}

static commandResult_t HisenseAC_Cmd_NotImplemented(const void *context, const char *cmd, const char *args, int cmdFlags) {
	(void)context;
	(void)args;
	(void)cmdFlags;
	addLogAdv(LOG_INFO, LOG_FEATURE_CMD,
		"HisenseAC shell: command %s is registered but not implemented yet", cmd);
	return CMD_RES_OK;
}

static commandResult_t HisenseAC_Cmd_Uart(const void *context, const char *cmd, const char *args, int cmdFlags) {
	int new_uart;
	(void)context;
	(void)cmd;
	(void)cmdFlags;

	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_GetArgsCount() < 1) {
		addLogAdv(LOG_INFO, LOG_FEATURE_CMD, "HisenseAC shell: current UART is %d", g_hisense_uart);
		return CMD_RES_OK;
	}

	new_uart = Tokenizer_GetArgInteger(0);
	if (new_uart < 0) {
		addLogAdv(LOG_INFO, LOG_FEATURE_CMD, "HisenseAC shell: invalid UART index %d", new_uart);
		return CMD_RES_BAD_ARGUMENT;
	}

	g_hisense_uart = new_uart;
	HisenseAC_ReinitUart();
	addLogAdv(LOG_INFO, LOG_FEATURE_CMD, "HisenseAC shell: UART changed to %d", g_hisense_uart);
	return CMD_RES_OK;
}

static commandResult_t HisenseAC_Cmd_Status(const void *context, const char *cmd, const char *args, int cmdFlags) {
	(void)context;
	(void)cmd;
	(void)cmdFlags;

	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_GetArgsCount() > 0) {
		const char *subcmd = Tokenizer_GetArg(0);
		if (!stricmp(subcmd, "poll")) {
			HisenseAC_SendStatusPollStub();
			addLogAdv(LOG_INFO, LOG_FEATURE_CMD, "HisenseAC shell: staged poll stub");
		}
		else if (!stricmp(subcmd, "probe")) {
			HisenseAC_SendShortProbeStub();
			addLogAdv(LOG_INFO, LOG_FEATURE_CMD, "HisenseAC shell: staged probe stub");
		}
		else if (!stricmp(subcmd, "rxbyte")) {
			byte fake_rx = 0;
			if (Tokenizer_GetArgsCount() > 1) {
				fake_rx = (byte)Tokenizer_GetArgInteger(1);
			}
			HisenseAC_ProcessRxByteStub(fake_rx);
			addLogAdv(LOG_INFO, LOG_FEATURE_CMD, "HisenseAC shell: staged rx-byte stub with 0x%02X", fake_rx);
		}
	}

	HisenseAC_PublishStateStub();
	addLogAdv(LOG_INFO, LOG_FEATURE_CMD,
		"HisenseAC shell: protocol families ready for later implementation -> 0C/66/00, 13/1E/00, 29/65/00, 7B/65/00, 97/66/00");
	return CMD_RES_OK;
}

void HisenseAC_Init(void) {
	memset(&g_hisense_ctx, 0, sizeof(g_hisense_ctx));

	g_hisense_uart = UART_GetSelectedPortIndex();
	if (g_hisense_uart < 0) {
		g_hisense_uart = 1;
	}

	g_hisense_ctx.uart_index = g_hisense_uart;
	g_hisense_ctx.running = 1;
	g_hisense_ctx.profile_id = HISENSE_AC_PROFILE_009_104;
	g_hisense_ctx.power_state = -1;
	g_hisense_ctx.mode_state = -1;
	g_hisense_ctx.fan_state = -1;
	g_hisense_ctx.swing_state = -1;
	g_hisense_ctx.sleep_state = -1;
	g_hisense_ctx.eco_state = -1;
	g_hisense_ctx.fan_mute_state = -1;
	g_hisense_ctx.super_state = -1;
	g_hisense_ctx.temp_unit_state = -1;
	g_hisense_ctx.fault_state = -1;
	g_hisense_ctx.target_temp = -1.0f;
	g_hisense_ctx.current_temp = -1.0f;
	g_hisense_ctx.pipe_temp = -1.0f;
	g_hisense_ctx.daily_energy_kwh = -1.0f;

	HisenseAC_RegisterCommands();
	HisenseAC_ReinitUart();

	addLogAdv(LOG_INFO, LOG_FEATURE_DRV,
		"HisenseAC shell initialized for profile 009-104 (protocol implementation intentionally deferred)");
}

void HisenseAC_Shutdown(void) {
	g_hisense_ctx.running = 0;
	addLogAdv(LOG_INFO, LOG_FEATURE_DRV, "HisenseAC shell stopped");
}

void HisenseAC_OnEverySecond(void) {
	if (!g_hisense_ctx.running) {
		return;
	}
	g_hisense_ctx.last_housekeeping_tick = xTaskGetTickCount();
}

void HisenseAC_RunFrame(void) {
	if (!g_hisense_ctx.running) {
		return;
	}
	g_hisense_ctx.last_quicktick_tick = xTaskGetTickCount();
	HisenseAC_ClassifyFrameStub(NULL, 0);
}

void HisenseAC_AppendInformationToHTTPIndexPage(http_request_t *request, int bPreState) {
	if (bPreState) {
		return;
	}

	poststr(request, "<hr><h4>HisenseAC</h4>");
	hprintf255(request, "<div>Shell stage: active</div>");
	hprintf255(request, "<div>Profile: %d</div>", g_hisense_ctx.profile_id);
	hprintf255(request, "<div>UART: %d</div>", g_hisense_ctx.uart_index);
	hprintf255(request, "<div>Running: %d</div>", g_hisense_ctx.running);
	hprintf255(request, "<div>RX frames seen: %u</div>", (unsigned int)g_hisense_ctx.parser_frames_seen);
	hprintf255(request, "<div>RX bytes seen: %u</div>", (unsigned int)g_hisense_ctx.parser_rx_bytes_seen);
	hprintf255(request, "<div>TX requests staged: %u</div>", (unsigned int)g_hisense_ctx.parser_tx_frames_requested);
	poststr(request, "<div>Stub protocol families: 0C/66/00, 13/1E/00, 29/65/00, 7B/65/00, 97/66/00</div>");
}

void HisenseAC_OnHassDiscovery(const char *topic) {
	(void)topic;
}

#endif
