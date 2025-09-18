#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include "esp_wifi.h"
#include <ArduinoJson.h>
#include <Preferences.h>
#include "esp_heap_caps.h"


// ====== Wi-Fi ======
const char *WIFI_SSID = "C3PO-IoT";
const char *WIFI_PASS = "IoT@Wifi#2025";

// ====== Web server na porte 8080 ======
WebServer server(8080);

// ====== Piny kamery (AI Thinker) ======
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

// Voliteľné: biela LED (flash) na GPIO 4
#define LED_PIN 4

// ====== ROI (NVS) ======
struct RoiConfig { int x; int y; int w; int h; };
Preferences prefs;
RoiConfig roi = {0, 0, 20, 20};
const char *NVS_NS = "roi_cfg";


uint32_t crc32_le(uint32_t crc, const uint8_t *buf, size_t len) {
  crc = ~crc;
  while (len--) {
    crc ^= *buf++;
    for (int i = 0; i < 8; i++) {
      if (crc & 1)
        crc = (crc >> 1) ^ 0xEDB88320UL;
      else
        crc = crc >> 1;
    }
  }
  return ~crc;
}

bool saveRoiToNvs(const RoiConfig &r) {
  if (!prefs.begin(NVS_NS, /*readOnly=*/false)) return false;
  prefs.putInt("x", r.x); prefs.putInt("y", r.y);
  prefs.putInt("w", r.w); prefs.putInt("h", r.h);
  prefs.end(); return true;
}
bool loadRoiFromNvs(RoiConfig &r) {
  if (!prefs.begin(NVS_NS, /*readOnly=*/true)) return false;
  bool has = prefs.isKey("x") && prefs.isKey("y") && prefs.isKey("w") && prefs.isKey("h");
  if (has) {
    r.x = prefs.getInt("x", r.x); r.y = prefs.getInt("y", r.y);
    r.w = prefs.getInt("w", r.w); r.h = prefs.getInt("h", r.h);
  }
  prefs.end(); return has;
}

// ====== Ping-pong JPEG buffers ======
static uint8_t *jpeg_rd = nullptr;     // buffer, z ktorého číta /shot.jpg
static size_t   jpeg_rd_len = 0;
static size_t   jpeg_rd_cap = 0;

static uint8_t *jpeg_wr = nullptr;     // buffer, do ktorého zapisuje capture task
static size_t   jpeg_wr_cap = 0;

static SemaphoreHandle_t jpegMtx = nullptr;  // chráni len flip/snapshot (krátko)
static volatile uint32_t last_frame_ms = 0;

static const int TARGET_FPS = 10;

static volatile uint32_t crc_fb   = 0;   // CRC priamo z camera_fb_t->buf
static volatile uint32_t crc_wr   = 0;   // CRC po memcpy do WR buffera
static volatile uint32_t crc_rd   = 0;   // CRC aktuálneho RD
static volatile uint32_t copy_mismatch = 0; // crc_wr != crc_fb
static volatile uint32_t send_crc_mismatch = 0; // CRC pred/po send-e

static volatile bool rd_in_use = false;

static volatile bool pause_capture = false;  // keď true, captureTask dočasne nič nechytá


static inline bool is_valid_jpeg(const uint8_t* p, size_t n) {
  return (n >= 4 && p[0]==0xFF && p[1]==0xD8 && p[n-2]==0xFF && p[n-1]==0xD9);
}
static inline uint16_t sum16(const uint8_t* p, size_t n) {
  uint32_t s = 0; for (size_t i=0;i<n;i++) s += p[i]; return (uint16_t)(s & 0xFFFF);
}
static volatile uint32_t send_bad_sum = 0;

// DEBUG
static volatile uint32_t cap_iter = 0, cap_ok = 0, cap_drop = 0, cap_bad_jpg = 0;
static volatile uint32_t send_bad_checksum = 0;
static volatile uint32_t cap_null = 0;     // fb == nullptr

// k výslednému RD si uložíme aj checksum
static volatile uint16_t jpeg_rd_sum = 0;
// static volatile bool rd_in_use = false;

static volatile uint32_t last_crc_wr = 0;
static volatile uint32_t last_crc_rd = 0;




// ---- Handler: aktuálny JPEG z posledného uloženého frame-u ----
void handleShotJpg() {
  // A) snapshot + zámok RD
  xSemaphoreTake(jpegMtx, portMAX_DELAY);
  if (!jpeg_rd || jpeg_rd_len == 0) {
    xSemaphoreGive(jpegMtx);
    server.send(503, "text/plain", "No frame available yet");
    return;
  }
  rd_in_use = true;
  uint8_t *ptr = jpeg_rd;
  size_t   len = jpeg_rd_len;
  uint32_t crc_before = crc32_le(0, ptr, len);


  // rýchla validácia
  if (!(len >= 4 && ptr[0]==0xFF && ptr[1]==0xD8 && ptr[len-2]==0xFF && ptr[len-1]==0xD9)) {
    rd_in_use = false;
    xSemaphoreGive(jpegMtx);
    server.send(503, "text/plain", "Invalid JPEG in buffer");
    return;
  }
  xSemaphoreGive(jpegMtx);

  // B) RAW HTTP odpoveď (bez WebServer::send)
  WiFiClient client = server.client();
  if (!client) {
    xSemaphoreTake(jpegMtx, portMAX_DELAY);
    rd_in_use = false;
    xSemaphoreGive(jpegMtx);
    server.send(503, "text/plain", "No client");
    return;
  }
  client.setTimeout(3000);

  // HEADERS
  client.print(F(
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: image/jpeg\r\n"
    "Access-Control-Allow-Origin: *\r\n"
    "Cache-Control: no-store, no-cache, must-revalidate, max-age=0\r\n"
    "Pragma: no-cache\r\n"
    "Expires: 0\r\n"
    "Connection: close\r\n"
    "Content-Length: "
  ));
  client.print(len);
  client.print(F("\r\n\r\n"));

  // BODY – pošli celé telo po menších blokoch
  size_t off = 0;
  while (off < len) {
    size_t chunk = len - off;
    if (chunk > 1460) chunk = 1460;     // MTU-friendly
    size_t n = client.write(ptr + off, chunk);
    if (n == 0) break;                  // klient sa odpojil
    off += n;
  }
  client.flush();
  client.stop();

  // C) odomkni RD
  xSemaphoreTake(jpegMtx, portMAX_DELAY);
  uint32_t crc_after = crc32_le(0, ptr, len);
  if (crc_after != crc_before) send_crc_mismatch++;

  rd_in_use = false;
  xSemaphoreGive(jpegMtx);
}



void handlePeek() {
  xSemaphoreTake(jpegMtx, portMAX_DELAY);
  uint8_t *ptr = jpeg_rd; size_t len = jpeg_rd_len;
  xSemaphoreGive(jpegMtx);

  StaticJsonDocument<256> j;
  if (!ptr || len < 4) {
    j["ok"] = false;
    j["reason"] = "empty";
  } else {
    char head[16], tail[16];
    snprintf(head, sizeof(head), "%02X %02X %02X %02X", ptr[0], ptr[1], ptr[2], ptr[3]);
    snprintf(tail, sizeof(tail), "%02X %02X %02X %02X", ptr[len-4], ptr[len-3], ptr[len-2], ptr[len-1]);
    j["ok"] = true;
    j["len"] = (uint32_t)len;
    j["head"] = head;
    j["tail"] = tail;
    j["is_jpeg"] = is_valid_jpeg(ptr, len);
  }
  String s; serializeJson(j, s);
  server.send(200, "application/json", s);
}




// Jednoduchá info stránka
void handleRoot() {
  String ip = WiFi.localIP().toString();
  String html =
      "<html><body>"
      "<h3>ESP32-CAM snapshot server</h3>"
      "<p>Snapshot: <a href=\"/shot.jpg\">/shot.jpg</a></p>"
      "<p>Health: <a href=\"/health\">/health</a></p>"
      "<p>IP: " + ip + "</p>"
      "</body></html>";
  server.send(200, "text/html", html);
}

void handleHealth() {
  StaticJsonDocument<512> res;
  res["last_frame_ms"] = last_frame_ms;
  res["jpeg_rd_len"]   = jpeg_rd_len;
  res["jpeg_rd_cap"]   = jpeg_rd_cap;
  res["jpeg_wr_cap"]   = jpeg_wr_cap;
  res["fps_target"]    = TARGET_FPS;
  res["cap_iter"]      = cap_iter;
  res["cap_ok"]        = cap_ok;
  res["cap_drop"]      = cap_drop;
  res["cap_bad_jpg"]   = cap_bad_jpg;
  res["send_bad_sum"]  = send_bad_checksum;
  res["cap_null"]      = cap_null;
  res["crc_fb"] = crc_fb;
  res["crc_wr"] = crc_wr;
  res["crc_rd"] = crc_rd;
  res["copy_mismatch"] = copy_mismatch;
  res["send_crc_mismatch"] = send_crc_mismatch;


  String s; serializeJson(res, s);
  server.send(200, "application/json", s);
}

void handleShotLiveOnce() {
  // Zastav capture, nech uvoľní framebuffer
  pause_capture = true;
  delay(5); // daj čas captureTasku dobehnúť

  camera_fb_t *fb = nullptr;
  // Skús si "vypýtať" frame s krátkym retry (do ~500 ms)
  unsigned long t0 = millis();
  while (!fb && (millis() - t0 < 500)) {
    fb = esp_camera_fb_get();
    if (!fb) delay(5);
  }

  if (!fb) {
    pause_capture = false;
    server.send(503, "text/plain", "No fb");
    return;
  }

  // Rýchla kontrola SOI/EOI
  const uint8_t* b = (const uint8_t*)fb->buf;
  size_t L = fb->len;
  bool ok = (L >= 4 && b[0]==0xFF && b[1]==0xD8 && b[L-2]==0xFF && b[L-1]==0xD9);
  if (!ok) {
    esp_camera_fb_return(fb);
    pause_capture = false;
    server.send(503, "text/plain", "Live fb invalid");
    return;
  }

  // RAW HTTP write (priame poslanie frame-u)
  WiFiClient client = server.client();
  client.print(F(
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: image/jpeg\r\n"
    "Access-Control-Allow-Origin: *\r\n"
    "Cache-Control: no-store, no-cache, must-revalidate, max-age=0\r\n"
    "Pragma: no-cache\r\n"
    "Expires: 0\r\n"
    "Connection: close\r\n"
    "Content-Length: "
  ));
  client.print(L);
  client.print(F("\r\n\r\n"));

  size_t off = 0;
  while (off < L) {
    size_t chunk = L - off; if (chunk > 1460) chunk = 1460;
    size_t n = client.write(b + off, chunk);
    if (n == 0) break;
    off += n;
  }
  client.flush();
  client.stop();

  esp_camera_fb_return(fb);
  pause_capture = false;
}




void handleRoi() {
  if (server.method() == HTTP_POST) {
    String body = server.arg("plain");
    if (body.length() == 0) { server.send(400, "application/json", "{\"status\":\"error\",\"reason\":\"empty body\"}"); return; }
    StaticJsonDocument<200> doc;
    auto err = deserializeJson(doc, body);
    if (err) { server.send(400, "application/json", "{\"status\":\"error\",\"reason\":\"invalid json\"}"); return; }
    int x = doc["x"] | -1, y = doc["y"] | -1, w = doc["w"] | -1, h = doc["h"] | -1;
    if (x < 0 || y < 0 || w <= 0 || h <= 0) { server.send(400, "application/json", "{\"status\":\"error\",\"reason\":\"invalid values\"}"); return; }
    roi = {x,y,w,h};
    if (!saveRoiToNvs(roi)) { server.send(500, "application/json", "{\"status\":\"error\",\"reason\":\"nvs save failed\"}"); return; }
  }
  StaticJsonDocument<200> res;
  res["roi_x"] = roi.x; res["roi_y"] = roi.y; res["roi_w"] = roi.w; res["roi_h"] = roi.h;
  res["status"] = "ok";
  String response; serializeJson(res, response);
  server.send(200, "application/json", response);
}

void startCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM; config.pin_d1 = Y3_GPIO_NUM; config.pin_d2 = Y4_GPIO_NUM; config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM; config.pin_d5 = Y7_GPIO_NUM; config.pin_d6 = Y8_GPIO_NUM; config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM; config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM; config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM; config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM; config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 10000000;
  config.frame_size = FRAMESIZE_SXGA; // 1280x1024
  config.pixel_format = PIXFORMAT_JPEG;
  // config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.grab_mode = CAMERA_GRAB_LATEST;
  // config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 15; // menšie číslo = lepšia kvalita
  // config.fb_count = 2;
  config.fb_count = 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    while (true) delay(1000);
  }
}

// ====== CAPTURE TASK ======
TaskHandle_t captureTaskHandle = nullptr;

// ====== CAPTURE TASK ======
void captureTask(void *param) {
  const TickType_t period = pdMS_TO_TICKS(1000 / TARGET_FPS);
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
  if (pause_capture) { vTaskDelay(pdMS_TO_TICKS(1)); continue; }  // ⬅️ pauza pre live test

  cap_iter++;
    
    camera_fb_t *fb = esp_camera_fb_get(); // JPEG, SXGA
    if (fb) {
      // 1) CRC priamo z DMA/framebufferu kamery
      uint32_t cfb = crc32_le(0, (const uint8_t*)fb->buf, fb->len);

      if (fb->len <= jpeg_wr_cap) {
        memcpy(jpeg_wr, fb->buf, fb->len);

        // 2) CRC po memcpy do WR
        uint32_t cwr = crc32_le(0, jpeg_wr, fb->len);

        xSemaphoreTake(jpegMtx, portMAX_DELAY);
        crc_fb = cfb;
        crc_wr = cwr;
        if (crc_wr != crc_fb) copy_mismatch++;   // ← ak toto rastie, kazí sa to pri kopírovaní/PSRAM

        if (!rd_in_use) {
          // flip WR -> RD
          uint8_t *old_rd = jpeg_rd; size_t old_cap = jpeg_rd_cap;

          jpeg_rd     = jpeg_wr;
          jpeg_rd_len = fb->len;
          jpeg_rd_cap = jpeg_wr_cap;

          jpeg_wr     = old_rd;
          jpeg_wr_cap = old_cap;

          crc_rd      = crc_wr;                 // 3) RD má rovnaký obsah ako WR v tomto momente
          last_frame_ms = millis();
          cap_ok++;
        }
        xSemaphoreGive(jpegMtx);
      } else {
        cap_drop++;
      }
      esp_camera_fb_return(fb);
    }

    // Ak dlhšie nič neprichádza, skús reinit kamery
    static uint32_t last_ok_ms = 0;
    if (cap_ok) last_ok_ms = last_frame_ms;
    if (!cap_ok && (millis() > 2000)) last_ok_ms = 1; // prvé 2 s po štarte ignoruj

    if (last_ok_ms && (int32_t)(millis() - last_ok_ms) > 2000) {
      Serial.println("[CAPTURE] No frames for 2s → reinit camera");
      esp_camera_deinit();
      delay(100);
      startCamera();
      last_ok_ms = 0;
    }

    vTaskDelay(pdMS_TO_TICKS(1000 / TARGET_FPS)); // ~10 FPS
  }

}



// -- nahrádza ensureCap() úplne vyhoď --

// veľkosť na predalokovanie (zvoľ si, napr. 512 kB)
static const size_t JPEG_BUF_SIZE = 512 * 1024;

void startCaptureLoop() {
  if (!jpegMtx) jpegMtx = xSemaphoreCreateMutex();
  if (!jpegMtx) { Serial.println("[ERR] Cannot create jpegMtx"); return; }

  // Predalokuj dva perzistentné buffery
  jpeg_rd = (uint8_t*) heap_caps_malloc(JPEG_BUF_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  jpeg_wr = (uint8_t*) heap_caps_malloc(JPEG_BUF_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!jpeg_rd || !jpeg_wr) {
    Serial.println("[ERR] JPEG prealloc failed");
    // skús fallback do interného heapu
    if (!jpeg_rd) jpeg_rd = (uint8_t*) heap_caps_malloc(JPEG_BUF_SIZE, MALLOC_CAP_8BIT);
    if (!jpeg_wr) jpeg_wr = (uint8_t*) heap_caps_malloc(JPEG_BUF_SIZE, MALLOC_CAP_8BIT);
  }
  if (!jpeg_rd || !jpeg_wr) {
    Serial.println("[ERR] JPEG prealloc failed (both) – giving up");
    return;
  }
  jpeg_rd_cap = JPEG_BUF_SIZE;
  jpeg_wr_cap = JPEG_BUF_SIZE;

  BaseType_t ok = xTaskCreatePinnedToCore(
  captureTask, "captureTask",
  12288, nullptr, 2, nullptr, 1);
  Serial.printf("[CAPTURE] start %s\n", ok == pdPASS ? "OK" : "FAIL");

}



void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);
  delay(200);

  // --- Wi-Fi ---
  WiFi.mode(WIFI_STA);
  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
    if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
      Serial.printf("\n[WiFi] Disconnected, reason=%d\n", info.wifi_sta_disconnected.reason);
    } else if (event == ARDUINO_EVENT_WIFI_STA_CONNECTED) {
      Serial.printf("\n[WiFi] Associated to AP\n");
    } else if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
      Serial.printf("\n[WiFi] Got IP: %s\n", WiFi.localIP().toString().c_str());
    }
  });

  WiFi.persistent(false);
  WiFi.disconnect(true, true);
  delay(200);
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_max_tx_power(78);

  wifi_country_t c = {.cc = "CZ", .schan = 1, .nchan = 13, .max_tx_power = 78, .policy = WIFI_COUNTRY_POLICY_MANUAL};
  esp_wifi_set_country(&c);
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);

  int n = WiFi.scanNetworks();
  int best = -1000, bestCh = 0; uint8_t bestBssid[6] = {0};
  for (int i = 0; i < n; i++) {
    if (WiFi.SSID(i) == WIFI_SSID && WiFi.RSSI(i) > best) {
      best = WiFi.RSSI(i); bestCh = WiFi.channel(i); memcpy(bestBssid, WiFi.BSSID(i), 6);
    }
  }
  if (best > -1000) {
    Serial.printf("[WiFi] Locking to ch=%d BSSID %02X:%02X:%02X:%02X:%02X:%02X RSSI=%d\n",
                  bestCh, bestBssid[0], bestBssid[1], bestBssid[2], bestBssid[3], bestBssid[4], bestBssid[5], best);
    WiFi.begin(WIFI_SSID, WIFI_PASS, bestCh, bestBssid, true);
  } else {
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  }

  Serial.printf("Connecting to SSID %s", WIFI_SSID);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(500); Serial.print(".");
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\n[WiFi] Connect timeout. Scanning…");
    int n = WiFi.scanNetworks();
    int best = -1000; uint8_t bestBssid[6] = {0};
    for (int i = 0; i < n; i++) {
      if (WiFi.SSID(i) == WIFI_SSID && WiFi.RSSI(i) > best) { best = WiFi.RSSI(i); memcpy(bestBssid, WiFi.BSSID(i), 6); }
    }
    if (best > -1000) {
      Serial.printf("[WiFi] Best BSSID %02X:%02X:%02X:%02X:%02X:%02X RSSI=%d. Retrying with BSSID…\n",
                    bestBssid[0], bestBssid[1], bestBssid[2], bestBssid[3], bestBssid[4], bestBssid[5], best);
      WiFi.disconnect(true, true); delay(200);
      WiFi.begin(WIFI_SSID, WIFI_PASS, 0, bestBssid, true);
      t0 = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) { delay(500); Serial.print("."); }
    } else {
      Serial.println("[WiFi] SSID not found in scan.");
    }
  }
  if (WiFi.status() == WL_CONNECTED) Serial.println("\nWiFi connected, IP: " + WiFi.localIP().toString());
  else Serial.println("\n[WiFi] Still not connected.");

  // --- Kamera ---
  startCamera();

  // --- ROI z NVS ---
  loadRoiFromNvs(roi);

  // --- Capture loop ---
  // jpegMtx = xSemaphoreCreateMutex();
  startCaptureLoop();

  // --- HTTP routy ---
  server.on("/", handleRoot);
  server.on("/shot.jpg", HTTP_GET, handleShotJpg);
  server.on("/health", HTTP_GET, handleHealth);
  server.on("/peek", HTTP_GET, handlePeek);
  server.on("/shot_live.jpg", HTTP_GET, handleShotLiveOnce);

  // server.on("/roi", handleRoi);

  server.begin();
  Serial.println("HTTP server started on port 8080");
}

void loop() {
  server.handleClient();
}
