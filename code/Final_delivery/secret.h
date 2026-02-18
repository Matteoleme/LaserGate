#ifndef SECRET_H
#define SECRET_H

// ====== COMPILA QUI ======
#define WIFI_SSID        "..."  // SSID Wi-Fi
#define WIFI_PASSWORD    "..."  // Password Wi-Fi
#define THINGNAME        "Esp32_IoT"  // AWS Thing Name
#define AWS_IOT_ENDPOINT "...-ats.iot.us-east-1.amazonaws.com"  // AWS IoT Endpoint

#define AWS_IOT_PUBLISH_TOPIC "esp32/tof"
#define AWS_IOT_SUBSCRIBE_TOPIC "esp32/test"

static const char AWS_CERT_CA[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
... (CERTIFICATE CONTENT HERE) ...
-----END CERTIFICATE-----
)EOF";

static const char AWS_CERT_CRT[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
... (CERTIFICATE CONTENT HERE) ...
-----END CERTIFICATE-----
)EOF";

static const char AWS_CERT_PRIVATE[] PROGMEM = R"EOF(
-----BEGIN RSA PRIVATE KEY-----
... (PRIVATE KEY CONTENT HERE) ...
-----END RSA PRIVATE KEY-----
)EOF";
// ====== FINE PARAMETRI ======

#endif
