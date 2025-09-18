#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include "esp_wifi.h"

// ====== Wi-Fi ======
const char* WIFI_SSID = "C3PO-IoT";
const char* WIFI_PASS = "IoT@Wifi#2025";

// ====== Web server na porte 8080 ======
WebServer server(8080);

// ====== Piny kamery (AI Thinker) ======
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// Voliteľné: biela LED (flash) na GPIO 4
#define LED_PIN            4

// ---- Handler: aktuálny JPEG ----
void handleShotJpg() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    server.send(503, "text/plain", "Camera capture failed");
    return;
  }
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "0");

  server.setContentLength(fb->len);
  server.send(200, "image/jpeg", "");
  WiFiClient client = server.client();
  client.write(fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

// Jednoduchá info stránka
void handleRoot() {
  String ip = WiFi.localIP().toString();
  String html =
    "<html><body>"
    "<h3>ESP32-CAM snapshot server</h3>"
    "<p>Snapshot: <a href=\"/shot.jpg\">/shot.jpg</a></p>"
    "<p>IP: " + ip + "</p>"
    "</body></html>";
  server.send(200, "text/html", html);
}

void startCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.frame_size   = FRAMESIZE_SXGA;     // 1280x1024
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 15;                 // menšie číslo = lepšia kvalita
  config.fb_count     = 2;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    while (true) delay(1000);
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);
  delay(200);

  // --- Wi-Fi ---
  WiFi.mode(WIFI_STA);

  // Logy Wi-Fi udalostí (pomôžu zistiť, kde padá handshake)
  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info){
    if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
      Serial.printf("\n[WiFi] Disconnected, reason=%d\n", info.wifi_sta_disconnected.reason);
    } else if (event == ARDUINO_EVENT_WIFI_STA_CONNECTED) {
      Serial.printf("\n[WiFi] Associated to AP\n");
    } else if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
      Serial.printf("\n[WiFi] Got IP: %s\n", WiFi.localIP().toString().c_str());
    }
  });

  // Vyčisti staré uložené pripojenia/BSSID a nepíš do NVS
  WiFi.persistent(false);
  WiFi.disconnect(true, true);
  delay(200);

  // Stabilnejšia asociácia: vypni power-save, zvýš TX výkon
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_max_tx_power(78);   // ~19.5 dBm (max)

    // krajina a povolené protokoly
  wifi_country_t c = { .cc="CZ", .schan=1, .nchan=13, .max_tx_power=78, .policy=WIFI_COUNTRY_POLICY_MANUAL };
  esp_wifi_set_country(&c);
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);

  // ak máš viac AP, uzamkneme aj kanál z najlepšieho BSSID
  int n = WiFi.scanNetworks();
  int best = -1000, bestCh = 0; uint8_t bestBssid[6] = {0};
  for (int i = 0; i < n; i++) {
    if (WiFi.SSID(i) == WIFI_SSID && WiFi.RSSI(i) > best) {
      best = WiFi.RSSI(i); bestCh = WiFi.channel(i);
      memcpy(bestBssid, WiFi.BSSID(i), 6);
    }
  }
  if (best > -1000) {
    Serial.printf("[WiFi] Locking to ch=%d BSSID %02X:%02X:%02X:%02X:%02X:%02X RSSI=%d\n",
      bestCh,bestBssid[0],bestBssid[1],bestBssid[2],bestBssid[3],bestBssid[4],bestBssid[5],best);
    WiFi.begin(WIFI_SSID, WIFI_PASS, bestCh, bestBssid, true);
  } else {
    WiFi.begin(WIFI_SSID, WIFI_PASS); // fallback
  }

  Serial.printf("Connecting to SSID %s", WIFI_SSID);

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(500);
    Serial.print(".");
  }

  // Ak sa do 20 s nechytí, urob scan a skús najbližšie BSSID
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\n[WiFi] Connect timeout. Scanning…");
    int n = WiFi.scanNetworks();
    int best = -1000;
    uint8_t bestBssid[6] = {0};
    for (int i = 0; i < n; i++) {
      if (WiFi.SSID(i) == WIFI_SSID && WiFi.RSSI(i) > best) {
        best = WiFi.RSSI(i);
        memcpy(bestBssid, WiFi.BSSID(i), 6);
      }
    }

    if (best > -1000) {
      Serial.printf("[WiFi] Best BSSID %02X:%02X:%02X:%02X:%02X:%02X RSSI=%d. Retrying with BSSID…\n",
        bestBssid[0], bestBssid[1], bestBssid[2], bestBssid[3], bestBssid[4], bestBssid[5], best);

      WiFi.disconnect(true, true);
      delay(200);
      WiFi.begin(WIFI_SSID, WIFI_PASS, 0, bestBssid, true); // lock na najbližšie AP
      t0 = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
        delay(500);
        Serial.print(".");
      }
    } else {
      Serial.println("[WiFi] SSID not found in scan.");
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected, IP: " + WiFi.localIP().toString());
  } else {
    Serial.println("\n[WiFi] Still not connected.");
  }

  // --- Kamera ---
  startCamera();

  // --- HTTP routy ---
  server.on("/", handleRoot);
  server.on("/shot.jpg", HTTP_GET, handleShotJpg);

  server.begin();
  Serial.println("HTTP server started on port 8080");
}

void loop() {
  server.handleClient();
}
