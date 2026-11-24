#ifndef __HAL_WIFI_H__
#define __HAL_WIFI_H__

#include <limits.h>

typedef enum {
	HAL_WIFI_STATION_ENABLE,
	HAL_WIFI_STATION_OFF
} HALWiFiStationPowerMode;

#define HAL_WIFI_STATION_MAX_RESTORE_SECONDS (INT_MAX / 1000)

typedef struct {
	int supported;
	int stationPresent;
	int macClockGated;
	int staRfSleep;
	int bleRfOwned;
} HALWiFiStationPowerStatus;

// Only the newer BK7231N SDK has a reviewed station/MAC sleep and wake pair.
// Feature selection remains in its implementation: this header can precede obk_config.h.
#if defined(PLATFORM_BEKEN_NEW) && defined(PLATFORM_BK7231N)
int HAL_WiFi_StationPowerSupported(void);
// 0 success, -2 rejected before changing hardware, -1 operation/readback failed.
int HAL_WiFi_SetStationPower(HALWiFiStationPowerMode mode);
int HAL_WiFi_GetStationPowerStatus(HALWiFiStationPowerStatus *status);
// Nonblocking cleanup: start=1 fences/stops BLE, -1 cancels the fence; polls return 1 idle, 0 pending, -1 failed.
int HAL_WiFi_PrepareDeepSleep(int start);
int HAL_WiFi_EnterDeepSleep(unsigned int seconds);
#else
static inline int HAL_WiFi_StationPowerSupported(void) {
	return 0;
}
static inline int HAL_WiFi_SetStationPower(HALWiFiStationPowerMode mode) {
	(void)mode;
	return -2;
}
static inline int HAL_WiFi_GetStationPowerStatus(HALWiFiStationPowerStatus *status) {
	status->supported = 0;
	status->stationPresent = -1;
	status->macClockGated = -1;
	status->staRfSleep = -1;
	status->bleRfOwned = -1;
	return -2;
}
static inline int HAL_WiFi_PrepareDeepSleep(int start) {
	(void)start;
	return -1;
}
static inline int HAL_WiFi_EnterDeepSleep(unsigned int seconds) {
	(void)seconds;
	return -1;
}
#endif

#ifndef MAC2STR
#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#endif
#ifndef MACSTR
#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"
#endif

typedef enum HALWifiStatus {
	WIFI_UNDEFINED,
	WIFI_STA_CONNECTING,
	WIFI_STA_DISCONNECTED,
	WIFI_STA_AUTH_FAILED,
	WIFI_STA_CONNECTED,
	WIFI_AP_CONNECTED,
	WIFI_AP_FAILED,
} HALWifiStatus_t;

typedef struct obkStaticIP_s {
	unsigned char localIPAddr[4];
	unsigned char netMask[4];
	unsigned char gatewayIPAddr[4];
	unsigned char dnsServerIpAddr[4];
} obkStaticIP_t;

typedef struct
{
	char bssid[6];
	unsigned int channel;
	unsigned int security_type;
#if PLATFORM_REALTEK
	unsigned char wpa_global_PSK[40];
	char pwd[128 + 1];
#elif PLATFORM_LN882H
	unsigned char psk[40];
#else
	char psk[64];
#endif
#if PLATFORM_BL602 || PLATFORM_BL616
	char pwd[64 + 1];
#endif
} obkFastConnectData_t;

int HAL_SetupWiFiOpenAccessPoint(const char* ssid);
void HAL_ConnectToWiFi(const char* oob_ssid, const char* connect_key, obkStaticIP_t *ip);
void HAL_FastConnectToWiFi(const char* oob_ssid, const char* connect_key, obkStaticIP_t* ip);
void HAL_DisableEnhancedFastConnect();
void HAL_DisconnectFromWifi();
void HAL_WiFi_SetupStatusCallback(void (*cb)(int code));
// This must return correct IP for both SOFT_AP and STATION modes,
// because, for example, javascript control panel requires it
const char* HAL_GetMyIPString();
const char* HAL_GetMyGatewayString();
const char* HAL_GetMyDNSString();
const char* HAL_GetMyMaskString();
const char* HAL_GetMACStr(char* macstr);

// Get WiFi Information (SSID / BSSID) - e.g. to display on status page 
// ATM there is only one SSID, so need for this code
//char* HAL_GetWiFiSSID(char* ssid);

char* HAL_GetWiFiBSSID(char* bssid);
typedef unsigned char uint8_t ;
uint8_t HAL_GetWiFiChannel(uint8_t *chan);

void WiFI_GetMacAddress(char* mac);
int WiFI_SetMacAddress(char* mac);
void HAL_PrintNetworkInfo();
int HAL_GetWifiStrength();

#endif
