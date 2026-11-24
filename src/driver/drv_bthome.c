#include "../new_common.h"
#include "../new_pins.h"
#include "../new_cfg.h"
#include "../cmnds/cmd_public.h"
#include "../logging/logging.h"
#include "../hal/hal_bt_proxy.h"
#include "drv_local.h"
#include "drv_public.h"
#include <math.h>

#if ENABLE_DRIVER_BTHOME

#define BTHOME_DEFAULT_PERIOD_SECONDS 5
#define BTHOME_DEFAULT_ADV_INTERVAL_MS 200
#define BTHOME_MIN_PERIOD_SECONDS 2
#define BTHOME_MAX_PERIOD_SECONDS 3600
#define BTHOME_MIN_ADV_INTERVAL_MS 100
#define BTHOME_MAX_ADV_INTERVAL_MS 10240
#define BTHOME_ADV_MAX_BYTES 31
#define BTHOME_FLAGS_BYTES 3
#define BTHOME_SERVICE_HEADER_BYTES 5
#define BTHOME_PACKET_ID_BYTES 2
#define BTHOME_NAME_HEADER_BYTES 2
#define BTHOME_LARGEST_OBJECT_BYTES 5
#define BTHOME_NAME_MAX_BYTES (BTHOME_ADV_MAX_BYTES - BTHOME_FLAGS_BYTES - BTHOME_SERVICE_HEADER_BYTES - BTHOME_PACKET_ID_BYTES - BTHOME_NAME_HEADER_BYTES - BTHOME_LARGEST_OBJECT_BYTES)
#define BTHOME_SERVICE_UUID_LOW 0xD2
#define BTHOME_SERVICE_UUID_HIGH 0xFC
#define BTHOME_V2_UNENCRYPTED 0x40
#define BTHOME_OBJECT_PACKET_ID 0x00
#define BTHOME_AD_FLAGS 0x01
#define BTHOME_AD_SHORT_NAME 0x08
#define BTHOME_AD_COMPLETE_NAME 0x09
#define BTHOME_AD_SERVICE_DATA 0x16

// Protocol descriptions stay local to this transport. Sensors continue to own
// acquisition, channel types, calibration and cached readings.
typedef struct {
	uint8_t id;
	uint8_t width;
	double multiplier;
	double minimum;
	double maximum;
	const char *name;
} bthome_object_t;

// ID order is also the packing order. Limits are encoded integer limits, so
// uint32 energy and signed power are checked before any narrowing conversion.
static const bthome_object_t s_objects[] = {
	{ 0x01, 1, 1, 0, 100, "battery_percent" },
	{ 0x02, 2, 100, -32768, 32767, "temperature_C" },
	{ 0x03, 2, 100, 0, 10000, "humidity_percent" },
	{ 0x04, 3, 100, 0, 16777215, "pressure_hPa" },
	{ 0x05, 3, 100, 0, 16777215, "illuminance_lux" },
	{ 0x11, 1, 1, 0, 1, "opening" },
	{ 0x21, 1, 1, 0, 1, "motion" },
	{ 0x43, 2, 1000, 0, 65535, "current_A" },
	{ 0x4A, 2, 10, 0, 65535, "voltage_V" },
	{ 0x4D, 4, 1000, 0, 4294967295.0, "energy_kWh" },
	{ 0x5C, 4, 100, -2147483648.0, 2147483647.0, "power_W" },
};

#define BTHOME_OBJECT_COUNT (sizeof(s_objects) / sizeof(s_objects[0]))

typedef struct {
	bool running;
	bool configured;
	bool layoutReady;
	bool nameValid;
	bool shortenedName;
	bool awaitingStart;
	bool awaitingStop;
	bool backendFault;
	bool sleepPaused;
	int periodSeconds;
	int advIntervalMs;
	int secondsRemaining;
	int channelTypes[CHANNEL_MAX];
	int counts[BTHOME_OBJECT_COUNT];
	int frames[BTHOME_OBJECT_COUNT];
	int invalidChannel[BTHOME_OBJECT_COUNT];
	int frameCount;
	int nextFrame;
	int unsupported;
	int oversized;
	int measurementCapacity;
	char configuredName[CGF_DEVICE_NAME_SIZE];
	char advertisedName[BTHOME_NAME_MAX_BYTES + 1];
	uint32_t invalidGroups;
	unsigned int packets;
	unsigned int errors;
	uint8_t packetId;
} bthome_state_t;

static bthome_state_t s_bthome;

static int BTHome_ObjectForType(int type) {
	switch (type) {
	case ChType_BatteryLevelPercent:
		return 0;
	case ChType_Temperature:
	case ChType_Temperature_div2:
	case ChType_Temperature_div10:
	case ChType_Temperature_div100:
		return 1;
	case ChType_Humidity:
	case ChType_Humidity_div10:
		return 2;
	case ChType_Pressure_div100:
		return 3;
	case ChType_Illuminance:
	case ChType_Illuminance_div10:
		return 4;
	case ChType_OpenClosed:
	case ChType_OpenClosed_Inv:
		return 5;
	case ChType_Motion:
	case ChType_Motion_n:
		return 6;
	case ChType_Current_div10:
	case ChType_Current_div100:
	case ChType_Current_div1000:
		return 7;
	case ChType_Voltage_div10:
	case ChType_Voltage_div100:
		return 8;
	case ChType_EnergyTotal_kWh_div100:
	case ChType_EnergyTotal_kWh_div1000:
	case ChType_EnergyExport_kWh_div1000:
	case ChType_EnergyToday_kWh_div1000:
	case ChType_EnergyImport_kWh_div1000:
		return 9;
	case ChType_Power:
	case ChType_Power_div10:
	case ChType_Power_div100:
		return 10;
	default:
		// Default, controls, unitless/custom values and physical types without an
		// exact supported BTHome representation must not acquire guessed units.
		return -1;
	}
}

static bool BTHome_ParseInteger(int argument, int maximum, int *result) {
	char *end;
	const char *text = Tokenizer_GetArg(argument);
	long value;

	if (!text) {
		return false;
	}
	value = strtol(text, &end, 10);

	if (end == text || *end || value < 0 || value > maximum) {
		return false;
	}
	*result = (int)value;
	return true;
}

// Validate the configured name and return the largest complete UTF-8 prefix
// within the legacy advertisement budget. Never split a code point or emit a
// malformed, overlong, surrogate or out-of-range UTF-8 sequence.
static int BTHome_NamePrefix(const char *text, int limit) {
	const unsigned char *bytes = (const unsigned char *)text;
	int length = strlen(text);
	int offset = 0;
	int prefix = 0;

	while (offset < length) {
		unsigned char first = bytes[offset];
		int width;
		int i;
		if (first < 0x80) {
			width = 1;
		} else if (first >= 0xC2 && first <= 0xDF) {
			width = 2;
		} else if (first >= 0xE0 && first <= 0xEF) {
			width = 3;
		} else if (first >= 0xF0 && first <= 0xF4) {
			width = 4;
		} else {
			return -1;
		}
		if (offset + width > length) {
			return -1;
		}
		for (i = 1; i < width; i++) {
			if ((bytes[offset + i] & 0xC0) != 0x80) {
				return -1;
			}
		}
		if ((first == 0xE0 && bytes[offset + 1] < 0xA0)
			|| (first == 0xED && bytes[offset + 1] >= 0xA0)
			|| (first == 0xF0 && bytes[offset + 1] < 0x90)
			|| (first == 0xF4 && bytes[offset + 1] >= 0x90)) {
			return -1;
		}
		offset += width;
		if (offset <= limit) {
			prefix = offset;
		}
	}
	return prefix;
}

// Called under the existing driver lifecycle lock. Type/name changes rebuild
// contiguous pages; each repeated-ID group stays intact in channel-index order.
static void BTHome_RefreshLayout(bool force) {
	const char *configuredName = CFG_GetOpenBekenHostName();
	bool changed = force || !s_bthome.layoutReady;
	bool nameChanged;
	int channel;
	unsigned int object;
	int nameLength;
	int frame = 0;
	int used = 0;
	bool any = false;

	if (!configuredName || !configuredName[0]) {
		configuredName = CFG_GetDeviceName();
		if (!configuredName || !configuredName[0]) {
			configuredName = CFG_GetShortDeviceName();
		}
	}
	if (!configuredName) {
		configuredName = "";
	}
	nameChanged = !s_bthome.layoutReady || strcmp(configuredName, s_bthome.configuredName) != 0;
	if (nameChanged) {
		changed = true;
	}
	for (channel = 0; channel < CHANNEL_MAX; channel++) {
		if (CHANNEL_GetType(channel) != s_bthome.channelTypes[channel]) {
			changed = true;
		}
	}
	if (!changed) {
		return;
	}
	strncpy(s_bthome.configuredName, configuredName, sizeof(s_bthome.configuredName) - 1);
	s_bthome.configuredName[sizeof(s_bthome.configuredName) - 1] = 0;
	nameLength = BTHome_NamePrefix(s_bthome.configuredName, BTHOME_NAME_MAX_BYTES);
	s_bthome.nameValid = nameLength > 0;
	s_bthome.advertisedName[0] = 0;
	if (s_bthome.nameValid) {
		memcpy(s_bthome.advertisedName, s_bthome.configuredName, nameLength);
		s_bthome.advertisedName[nameLength] = 0;
	} else {
		nameLength = 0;
		if (nameChanged) {
			ADDLOG_ERROR(LOG_FEATURE_DRV, "BTHome: configured device name is empty or invalid UTF-8; advertising disabled");
		}
	}
	s_bthome.shortenedName = s_bthome.nameValid && strlen(s_bthome.configuredName) != (unsigned int)nameLength;
	if (s_bthome.shortenedName && nameChanged) {
		ADDLOG_INFO(LOG_FEATURE_DRV, "BTHome: legacy name budget shortened configured name '%s' to '%s' (%d bytes)", s_bthome.configuredName, s_bthome.advertisedName, nameLength);
	}
	s_bthome.measurementCapacity = BTHOME_ADV_MAX_BYTES - BTHOME_FLAGS_BYTES - BTHOME_SERVICE_HEADER_BYTES - BTHOME_PACKET_ID_BYTES - BTHOME_NAME_HEADER_BYTES - nameLength;
	s_bthome.unsupported = 0;
	s_bthome.oversized = 0;
	s_bthome.invalidGroups = 0;
	memset(s_bthome.counts, 0, sizeof(s_bthome.counts));
	for (channel = 0; channel < CHANNEL_MAX; channel++) {
		int type = CHANNEL_GetType(channel);
		int index = BTHome_ObjectForType(type);
		s_bthome.channelTypes[channel] = type;
		if (index >= 0) {
			s_bthome.counts[index]++;
		} else if (type != ChType_Default) {
			s_bthome.unsupported++;
			ADDLOG_DEBUG(LOG_FEATURE_DRV, "BTHome: skip channel=%d type=%d; no unambiguous supported object", channel, type);
		}
	}
	for (object = 0; object < BTHOME_OBJECT_COUNT; object++) {
		int bytes = s_bthome.counts[object] * (s_objects[object].width + 1);
		s_bthome.frames[object] = -1;
		s_bthome.invalidChannel[object] = -1;
		if (bytes == 0) {
			continue;
		}
		if (bytes > s_bthome.measurementCapacity) {
			s_bthome.oversized++;
			ADDLOG_ERROR(LOG_FEATURE_DRV, "BTHome: reject whole object=0x%02X group fields=%d bytes=%d capacity=%d; repeated fields cannot be split", s_objects[object].id, s_bthome.counts[object], bytes, s_bthome.measurementCapacity);
			continue;
		}
		if (used + bytes > s_bthome.measurementCapacity) {
			frame++;
			used = 0;
		}
		s_bthome.frames[object] = frame;
		used += bytes;
		any = true;
	}
	s_bthome.frameCount = any ? frame + 1 : 0;
	s_bthome.nextFrame = 0;
	s_bthome.layoutReady = true;
	// Cancel an older page before replacing its layout/name. The common tick
	// waits for idle before allowing the first page of the replacement layout.
	HAL_BT_AdvStop();
	s_bthome.awaitingStart = false;
	s_bthome.awaitingStop = true;
	ADDLOG_INFO(LOG_FEATURE_DRV, "BTHome: discovered typed channels; unsupported=%d oversizedGroups=%d pages=%d nominalGroupRefresh=%ds capacity=%d name='%s'", s_bthome.unsupported, s_bthome.oversized, s_bthome.frameCount, s_bthome.frameCount * s_bthome.periodSeconds, s_bthome.measurementCapacity, s_bthome.advertisedName);
	if (!s_bthome.nameValid || s_bthome.frameCount == 0) {
		if (s_bthome.nameValid && s_bthome.frameCount == 0) {
			ADDLOG_ERROR(LOG_FEATURE_DRV, "BTHome: no encodable typed-channel group; advertising disabled");
		}
	}
}

static int BTHome_EncodeGroup(unsigned int object, uint8_t *data, int *invalidChannel) {
	const bthome_object_t *description = &s_objects[object];
	int channel;
	int length = 0;
	*invalidChannel = -1;

	for (channel = 0; channel < CHANNEL_MAX; channel++) {
		int type = s_bthome.channelTypes[channel];
		double value;
		double scaled;
		int64_t rounded;
		uint32_t encoded;
		unsigned int byte;
		if (BTHome_ObjectForType(type) != (int)object) {
			continue;
		}
		*invalidChannel = channel;
		if (!CHANNEL_HasValue(channel)) {
			return -2;
		}
		if (CHANNEL_GetType(channel) != type) {
			return -1;
		}
		value = CHANNEL_GetFinalValue(channel);
		if (CHANNEL_GetType(channel) != type || !isfinite(value) || (description->minimum == 0 && value < 0)) {
			return -1;
		}
		if (description->id == 0x11 || description->id == 0x21) {
			if (value != 0 && value != 1) {
				return -1;
			}
			// OpenClosed raw 1 means closed; Motion_n raw 1 means no motion.
			if (type == ChType_OpenClosed || type == ChType_Motion_n) {
				value = 1 - value;
			}
		}
		if ((description->id == 0x01 || description->id == 0x03) && (value < 0 || value > 100)) {
			return -1;
		}
		scaled = value * description->multiplier;
		if (!isfinite(scaled) || scaled < description->minimum - 0.5 || scaled > description->maximum + 0.5) {
			return -1;
		}
		// Round symmetrically in an int64 domain before narrowing. All accepted
		// values are near int32/uint32 limits, safely inside the int64 cast range.
		rounded = (int64_t)(scaled + (scaled < 0 ? -0.5 : 0.5));
		if (rounded < description->minimum || rounded > description->maximum
			|| length + description->width + 1 > BTHOME_ADV_MAX_BYTES) {
			return -1;
		}
		encoded = (uint32_t)rounded;
		data[length++] = description->id;
		for (byte = 0; byte < description->width; byte++) {
			data[length++] = (uint8_t)(encoded >> (byte * 8));
		}
	}
	*invalidChannel = -1;
	return length;
}

static commandResult_t BTHome_Setup(const void *context, const char *cmd, const char *args, int flags) {
	int period = BTHOME_DEFAULT_PERIOD_SECONDS;
	int interval = BTHOME_DEFAULT_ADV_INTERVAL_MS;
	int arguments;

	Tokenizer_TokenizeString(args, 0);
	arguments = Tokenizer_GetArgsCount();
	if (arguments > 2 || (arguments >= 1 && !BTHome_ParseInteger(0, BTHOME_MAX_PERIOD_SECONDS, &period))
		|| (arguments >= 2 && !BTHome_ParseInteger(1, BTHOME_MAX_ADV_INTERVAL_MS, &interval))
		|| period < BTHOME_MIN_PERIOD_SECONDS || interval < BTHOME_MIN_ADV_INTERVAL_MS) {
		ADDLOG_ERROR(LOG_FEATURE_DRV, "BTHome_Setup [periodSeconds=5] [advIntervalMs=200]; channel arguments are not supported");
		return CMD_RES_BAD_ARGUMENT;
	}
	// The periodic callback already holds this lifecycle lock. Commit timing and
	// the discovered layout atomically so a report cannot mix configurations.
	if (!DRV_Mutex_Take(100)) {
		return CMD_RES_ERROR;
	}
	if (!s_bthome.running) {
		DRV_Mutex_Free();
		return CMD_RES_ERROR;
	}
	s_bthome.periodSeconds = period;
	s_bthome.advIntervalMs = interval;
	s_bthome.secondsRemaining = period;
	s_bthome.configured = true;
	s_bthome.backendFault = false;
	BTHome_RefreshLayout(true);
	DRV_Mutex_Free();
	ADDLOG_INFO(LOG_FEATURE_DRV, "BTHome: typed-channel transport configured period=%ds interval=%dms; cached values, no sensor acquisition", period, interval);
	return CMD_RES_OK;
}

static commandResult_t BTHome_Status(const void *context, const char *cmd, const char *args, int flags) {
	bthome_state_t snapshot;
	int error = 0;
	int state;
	int channel;
	unsigned int object;

	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_GetArgsCount() != 0) {
		return CMD_RES_BAD_ARGUMENT;
	}
	if (!DRV_Mutex_Take(100)) {
		return CMD_RES_ERROR;
	}
	snapshot = s_bthome;
	DRV_Mutex_Free();
	// Never hold the lifecycle lock while acquiring the platform status mutex.
	state = HAL_BT_AdvGetState(&error);
	ADDLOG_INFO(LOG_FEATURE_DRV, "BTHome: running=%d configured=%d period=%ds interval=%dms next=%ds adv=%d error=%d submitted=%u errors=%u", snapshot.running, snapshot.configured, snapshot.periodSeconds, snapshot.advIntervalMs, snapshot.secondsRemaining, state, error, snapshot.packets, snapshot.errors);
	ADDLOG_INFO(LOG_FEATURE_DRV, "BTHome: awaitingStart=%d awaitingStop=%d backendFault=%d; accepted requests are not completed starts or received packets", snapshot.awaitingStart, snapshot.awaitingStop, snapshot.backendFault);
	ADDLOG_INFO(LOG_FEATURE_DRV, "BTHome: configuredName='%s' advertisedName='%s' complete=%d nameValid=%d", snapshot.configuredName, snapshot.advertisedName, snapshot.nameValid && !snapshot.shortenedName, snapshot.nameValid);
	ADDLOG_INFO(LOG_FEATURE_DRV, "BTHome: pages=%d nextPage=%d nominalGroupRefresh=%ds unsupportedChannels=%d oversizedGroups=%d invalidGroupMask=0x%X; cached values; channel API write required since boot/type change, not physical freshness", snapshot.frameCount, snapshot.frameCount ? snapshot.nextFrame + 1 : 0, snapshot.frameCount * snapshot.periodSeconds, snapshot.unsupported, snapshot.oversized, (unsigned int)snapshot.invalidGroups);
	for (object = 0; object < BTHOME_OBJECT_COUNT; object++) {
		if (snapshot.counts[object]) {
			ADDLOG_INFO(LOG_FEATURE_DRV, "BTHome: object=0x%02X %s fields=%d page=%d latestCheckedInvalid=%d invalidChannel=%d", s_objects[object].id, s_objects[object].name, snapshot.counts[object], snapshot.frames[object] + 1, (snapshot.invalidGroups & (1U << object)) != 0, snapshot.invalidChannel[object]);
		}
	}
	for (channel = 0; channel < CHANNEL_MAX; channel++) {
		int type = snapshot.channelTypes[channel];
		int index = BTHome_ObjectForType(type);
		if (type == ChType_Default) {
			continue;
		}
		if (index < 0) {
			ADDLOG_INFO(LOG_FEATURE_DRV, "BTHome: channel=%d type=%d skipped; unsupported or ambiguous physical meaning", channel, type);
		} else if (CHANNEL_GetType(channel) != type) {
			ADDLOG_INFO(LOG_FEATURE_DRV, "BTHome: channel=%d type changed since layout; rediscovery pending", channel);
		} else {
			ADDLOG_INFO(LOG_FEATURE_DRV, "BTHome: channel=%d type=%d object=0x%02X %s written=%d cached=%.3f", channel, type, s_objects[index].id, s_objects[index].name, CHANNEL_HasValue(channel), CHANNEL_GetFinalValue(channel));
		}
	}
	return CMD_RES_OK;
}

void BTHome_Init(void) {
	// Driver restarts reset driver counters, but never reset backend SDK ownership.
	memset(&s_bthome, 0, sizeof(s_bthome));
	s_bthome.running = true;
	s_bthome.periodSeconds = BTHOME_DEFAULT_PERIOD_SECONDS;
	s_bthome.advIntervalMs = BTHOME_DEFAULT_ADV_INTERVAL_MS;
	// Start unconfigured; setup discovers typed channels before any publication.
	//cmddetail:{"name":"BTHome_Setup","args":"[OptionalPeriodSeconds=5] [OptionalAdvIntervalMs=200]",
	//cmddetail:"descr":"Configure timing and automatically discover supported typed channels: battery percent, temperature, humidity, pressure, illuminance, opening, motion, current, voltage, energy and power, including their handled scaled channel types. The old temperature/humidity channel arguments are no longer accepted. PeriodSeconds is a payload page interval of 2..3600 seconds, default 5; AdvIntervalMs requests BLE repetition every 100..10240 ms while active, default 200, rounded to 0.625 ms units. Neither changes sensor acquisition. Every channel requires a channel API write after boot or type change before its entire repeated-ID group is advertised; an unchanged zero counts. Startup/default/remembered values alone are withheld. Explicit static/virtual values can use setChannel in early.bat or autoexec.bat. This is write readiness, not physical sample-age tracking; existing drivers own acquisition. Whole groups of repeated BTHome object IDs stay together in channel-index order; pages rotate when groups do not all fit, so each group's nominal refresh is page count times PeriodSeconds. Oversized or invalid groups are withheld with diagnostics, never split or partially published. <br />The existing configured hostname preference (flag 29) selects long or short device name, falling back to the other configured name if empty. Every primary advertisement includes this name; names beyond the fixed legacy name budget are shortened at a complete UTF-8 boundary and reported. Both names empty or invalid UTF-8 disables advertising. Unsupported/ambiguous channel types and native power-meter/Battery readings outside channels are not automatically published. Timing stays in RAM; rerun setup in autoexec.bat after each restart. Payload updates can briefly interrupt advertising.",
	//cmddetail:"fn":"BTHome_Setup","file":"driver/drv_bthome.c","requires":"ENABLE_DRIVER_BTHOME",
	//cmddetail:"examples":"BTHome_Setup 5 200 <br />Use timing defaults: BTHome_Setup <br />Startup example in autoexec.bat: <br />setChannelType 0 Temperature_div10 <br />setChannelType 1 Humidity <br />startDriver BTHome <br />BTHome_Setup 5 200"}
	CMD_RegisterCommand("BTHome_Setup", BTHome_Setup, NULL);
	//cmddetail:{"name":"BTHome_Status","args":"",
	//cmddetail:"descr":"Show timing, configured/advertised name, discovered channel/object mappings, whole-group pages and nominal group refresh time, unsupported/oversized groups, latest checked invalid-group mask and first invalid channel, backend state/error, awaiting-start/stop flags, accepted update requests and aggregate error events (invalid groups, missing layouts or rejected submissions). A page waits for a successful backend active state before another is submitted; accepted requests do not prove completed starts or transmitted/received packet counts. Backend states: 0 idle, 1 starting, 2 active, 3 stopping, -1 failed. Periodic updates may show idle/stopping; error 0 means no recorded backend error. Latched backend faults require reboot before reuse. Cached readings do not establish freshness. The Home Assistant BTHome integration may add its own identifier suffix and retain a previously stored name.",
	//cmddetail:"fn":"BTHome_Status","file":"driver/drv_bthome.c","requires":"ENABLE_DRIVER_BTHOME",
	//cmddetail:"examples":"BTHome_Status"}
	CMD_RegisterCommand("BTHome_Status", BTHome_Status, NULL);
	ADDLOG_INFO(LOG_FEATURE_DRV, "BTHome: started; configure physical channel types, then BTHome_Setup [periodSeconds=5] [advIntervalMs=200]");
}

void BTHome_OnEverySecond(void) {
	uint8_t packet[BTHOME_ADV_MAX_BYTES];
	uint8_t group[BTHOME_ADV_MAX_BYTES];
	char hex[BTHOME_ADV_MAX_BYTES * 2 + 1];
	int length;
	int service;
	int frame;
	int fields = 0;
	int nameLength;
	int result;
	unsigned int object;
	int i;

	if (Main_IsDeepSleepPending()) {
		s_bthome.sleepPaused = true;
		s_bthome.awaitingStart = false;
		s_bthome.awaitingStop = false;
		return;
	}
	if (s_bthome.sleepPaused) {
		s_bthome.sleepPaused = false;
		s_bthome.secondsRemaining = 0;
		s_bthome.awaitingStop = true;
	}
	if (!s_bthome.running || !s_bthome.configured) {
		return;
	}
	if (s_bthome.secondsRemaining > 0) {
		s_bthome.secondsRemaining--;
	}
	if (s_bthome.backendFault) {
		return;
	}
	if (s_bthome.awaitingStart || s_bthome.awaitingStop) {
		int error = 0;
		int state = HAL_BT_AdvGetState(&error);
		if (state < 0 || error < 0) {
			s_bthome.awaitingStart = false;
			s_bthome.awaitingStop = false;
			s_bthome.backendFault = true;
			s_bthome.errors++;
			ADDLOG_ERROR(LOG_FEATURE_DRV, "BTHome: backend page transition failed state=%d error=%d; no page completion claimed, reboot required", state, error);
			return;
		}
		if (s_bthome.awaitingStart && state == 2 && error == 0) {
			s_bthome.awaitingStart = false;
		} else if (s_bthome.awaitingStop && state == 0 && error == 0) {
			s_bthome.awaitingStop = false;
		} else {
			// In particular, idle can still mean an accepted page is waiting
			// for SDK command ownership. Never overwrite that queued page.
			return;
		}
	}
	if (s_bthome.secondsRemaining > 0) {
		return;
	}
	s_bthome.secondsRemaining = s_bthome.periodSeconds;
	BTHome_RefreshLayout(false);
	if (s_bthome.awaitingStop) {
		return;
	}
	if (!s_bthome.nameValid || s_bthome.frameCount == 0) {
		s_bthome.errors++;
		return;
	}
	// Existing sensors own acquisition and calibration. These are cached values,
	// not a successful-sample notification; elapsed startup time cannot prove freshness.
	frame = s_bthome.nextFrame;
	nameLength = strlen(s_bthome.advertisedName);
	packet[0] = 2;
	packet[1] = BTHOME_AD_FLAGS;
	packet[2] = 6;
	packet[3] = nameLength + 1;
	packet[4] = s_bthome.shortenedName ? BTHOME_AD_SHORT_NAME : BTHOME_AD_COMPLETE_NAME;
	memcpy(packet + BTHOME_FLAGS_BYTES + BTHOME_NAME_HEADER_BYTES, s_bthome.advertisedName, nameLength);
	length = BTHOME_FLAGS_BYTES + BTHOME_NAME_HEADER_BYTES + nameLength;
	service = length;
	packet[length++] = 0;
	packet[length++] = BTHOME_AD_SERVICE_DATA;
	packet[length++] = BTHOME_SERVICE_UUID_LOW;
	packet[length++] = BTHOME_SERVICE_UUID_HIGH;
	packet[length++] = BTHOME_V2_UNENCRYPTED;
	packet[length++] = BTHOME_OBJECT_PACKET_ID;
	packet[length++] = s_bthome.packetId;
	// Each page is a contiguous range of ascending object IDs. Wrapping happens
	// only between packets, never inside a packet or a repeated-ID group.
	for (object = 0; object < BTHOME_OBJECT_COUNT; object++) {
		int bytes;
		int invalidChannel;
		if (s_bthome.frames[object] != frame) {
			continue;
		}
		bytes = BTHome_EncodeGroup(object, group, &invalidChannel);
		if (bytes == -2) {
			ADDLOG_DEBUG(LOG_FEATURE_DRV, "BTHome: awaiting first channel write object=0x%02X channel=%d; whole group withheld", s_objects[object].id, invalidChannel);
			continue;
		}
		if (bytes <= 0 || bytes != s_bthome.counts[object] * (s_objects[object].width + 1)
			|| length + bytes > BTHOME_ADV_MAX_BYTES) {
			if (!(s_bthome.invalidGroups & (1U << object))) {
				ADDLOG_ERROR(LOG_FEATURE_DRV, "BTHome: withholding entire object=0x%02X group channel=%d; invalid/out-of-range cached value or channel type changed", s_objects[object].id, invalidChannel);
			}
			s_bthome.invalidChannel[object] = invalidChannel;
			s_bthome.invalidGroups |= 1U << object;
			s_bthome.errors++;
			continue;
		}
		s_bthome.invalidGroups &= ~(1U << object);
		s_bthome.invalidChannel[object] = -1;
		memcpy(packet + length, group, bytes);
		length += bytes;
		fields += s_bthome.counts[object];
	}
	if (fields == 0) {
		HAL_BT_AdvStop();
		s_bthome.awaitingStop = true;
		s_bthome.nextFrame = (frame + 1) % s_bthome.frameCount;
		return;
	}
	packet[service] = length - service - 1;
	// BLE intervals use 0.625 ms units. Round once at the platform boundary.
	result = HAL_BT_AdvSubmit(packet, length, (uint16_t)((s_bthome.advIntervalMs * 8 + 2) / 5));
	if (result <= 0) {
		s_bthome.errors++;
		ADDLOG_DEBUG(LOG_FEATURE_DRV, "BTHome: submit deferred/error=%d errors=%u", result, s_bthome.errors);
		return;
	}
	s_bthome.packetId++;
	s_bthome.packets++;
	s_bthome.awaitingStart = true;
	s_bthome.nextFrame = (frame + 1) % s_bthome.frameCount;
	for (i = 0; i < length; i++) {
		sprintf(hex + i * 2, "%02X", packet[i]);
	}
	ADDLOG_DEBUG(LOG_FEATURE_DRV, "BTHome: submitted=%u page=%d/%d fields=%d name='%s' interval=%dms payload=%s", s_bthome.packets, frame + 1, s_bthome.frameCount, fields, s_bthome.advertisedName, s_bthome.advIntervalMs, hex);
}

void BTHome_Stop(void) {
	s_bthome.running = false;
	s_bthome.awaitingStart = false;
	s_bthome.awaitingStop = false;
	HAL_BT_AdvStop();
	ADDLOG_INFO(LOG_FEATURE_DRV, "BTHome: stop requested; platform tick retains asynchronous cleanup ownership");
}

#endif
