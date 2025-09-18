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

// ====== Web server ======
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

#define LED_PIN 4

// ====== ROI (NVS) ======
struct RoiConfig
{
  int x;
  int y;
  int w;
  int h;
};
Preferences prefs;
RoiConfig roi = {0, 0, 20, 20};
const char *NVS_NS = "roi_cfg";

bool saveRoiToNvs(const RoiConfig &r)
{
  if (!prefs.begin(NVS_NS, false))
    return false;
  prefs.putInt("x", r.x);
  prefs.putInt("y", r.y);
  prefs.putInt("w", r.w);
  prefs.putInt("h", r.h);
  prefs.end();
  return true;
}
bool loadRoiFromNvs(RoiConfig &r)
{
  if (!prefs.begin(NVS_NS, true))
    return false;
  bool has = prefs.isKey("x") && prefs.isKey("y") && prefs.isKey("w") && prefs.isKey("h");
  if (has)
  {
    r.x = prefs.getInt("x", r.x);
    r.y = prefs.getInt("y", r.y);
    r.w = prefs.getInt("w", r.w);
    r.h = prefs.getInt("h", r.h);
  }
  prefs.end();
  return has;
}

// ====== Ping-pong JPEG buffery ======
// === Analysis ping-pong buffers (Variant B preparation) ===
// Independent copy for analysis task; keeps /shot.jpg path unchanged.
static uint8_t *ana_front = nullptr; // published snapshot for analysis (read-only for analysis)
static uint8_t *ana_back = nullptr;  // captureTask writes here then swaps to front
static size_t ana_front_len = 0;     // current valid length in ana_front
static size_t ana_back_cap = 0;      // allocated capacity of ana_back/front (equal sizes)

// Lightweight sequencing for lock-free publish/consume of analysis JPEG
static volatile uint32_t ana_seq = 0;      // increments on each publish
static volatile uint32_t ana_seq_seen = 0; // last sequence processed by analysis

// Optional: semaphore to wake the analysis task on new frame
static SemaphoreHandle_t anaSem = nullptr;

// ROI shared config (already persisted via /roi endpoints if enabled)
typedef struct
{
  uint16_t x, y, w, h;
} roi_t;
static roi_t g_roi = {0, 0, 0, 0}; // loaded from NVS if available

// Detector state placeholders (diagnostics + future /pulse)
static volatile uint32_t g_pulse_counter = 0;
static volatile uint32_t g_last_pulse_ms = 0;
static volatile uint32_t g_last_interpulse_ms = 0;
static volatile uint16_t g_last_R_mean = 0; // last computed mean of red channel in ROI (0..255)

// EMA / hysteresis config (will be tuned later; placeholders)
static float g_ema = 0.0f;
static float EMA_ALPHA = 0.30f;
static uint16_t TH_ON = 140;
static uint16_t TH_OFF = 110;
static uint32_t REFRACT_MS = 120;
static volatile bool g_led_on = false;

// Forward decl of the analysis task (implemented at end of file)
void analysisTask(void *);

static uint8_t *jpeg_rd = nullptr; // /shot.jpg číta z tohto
static size_t jpeg_rd_len = 0;
static size_t jpeg_rd_cap = 0;

static uint8_t *jpeg_wr = nullptr; // captureTask zapisuje sem
static size_t jpeg_wr_cap = 0;

static SemaphoreHandle_t jpegMtx = nullptr; // chráni len flip/snapshot
static volatile bool rd_in_use = false;

static volatile uint32_t last_frame_ms = 0;

static const int TARGET_FPS = 10;
static const size_t JPEG_BUF_SIZE = 512 * 1024; // rezerva pre SXGA JPEG

// ---- /shot.jpg (RAW HTTP odpoveď) ----
void handleShotJpg()
{
  // snapshot + lock RD počas odosielania (capture nebude flipovať)
  xSemaphoreTake(jpegMtx, portMAX_DELAY);
  if (!jpeg_rd || jpeg_rd_len == 0)
  {
    xSemaphoreGive(jpegMtx);
    server.send(503, "text/plain", "No frame available yet");
    return;
  }
  rd_in_use = true;
  uint8_t *ptr = jpeg_rd;
  size_t len = jpeg_rd_len;

  // rýchla validácia SOI/EOI
  bool ok = (len >= 4 && ptr[0] == 0xFF && ptr[1] == 0xD8 && ptr[len - 2] == 0xFF && ptr[len - 1] == 0xD9);
  if (!ok)
  {
    rd_in_use = false;
    xSemaphoreGive(jpegMtx);
    server.send(503, "text/plain", "Invalid JPEG in buffer");
    return;
  }
  xSemaphoreGive(jpegMtx);

  WiFiClient client = server.client();
  if (!client)
  {
    xSemaphoreTake(jpegMtx, portMAX_DELAY);
    rd_in_use = false;
    xSemaphoreGive(jpegMtx);
    server.send(503, "text/plain", "No client");
    return;
  }
  client.setTimeout(3000);

  // HEADERS
  client.print(
      "HTTP/1.1 200 OK\r\n"
      "Content-Type: image/jpeg\r\n"
      "Access-Control-Allow-Origin: *\r\n"
      "Cache-Control: no-store, no-cache, must-revalidate, max-age=0\r\n"
      "Pragma: no-cache\r\n"
      "Expires: 0\r\n"
      "Connection: close\r\n"
      "Content-Length: ");
  client.print(len);
  client.print("\r\n\r\n");

  // BODY – po MTU-friendly blokoch
  size_t off = 0;
  while (off < len)
  {
    size_t chunk = len - off;
    if (chunk > 1460)
      chunk = 1460;
    size_t n = client.write(ptr + off, chunk);
    if (n == 0)
      break;
    off += n;
  }
  client.flush();
  client.stop();

  // odomkni RD pre capture
  xSemaphoreTake(jpegMtx, portMAX_DELAY);
  rd_in_use = false;
  xSemaphoreGive(jpegMtx);
}

// ---- Jednoduchá info stránka ----
void handleRoot()
{
  String ip = WiFi.localIP().toString();
  String html =
      "<html><body>"
      "<h3>ESP32-CAM snapshot server</h3>"
      "<p>Snapshot: <a href=\"/shot.jpg\">/shot.jpg</a></p>"
      "<p>Health: <a href=\"/health\">/health</a></p>"
      "<p>IP: " +
      ip + "</p>"
           "</body></html>";
  server.send(200, "text/html", html);
}

// ---- Minimal /health ----
void handleHealth()
{
  StaticJsonDocument<200> res;
  res["last_frame_ms"] = last_frame_ms;
  res["jpeg_len"] = jpeg_rd_len;
  res["fps"] = TARGET_FPS;

  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "0");

  String s;
  serializeJson(res, s);
  server.send(200, "application/json", s);
}

// (voliteľné) ROI API – pripravené, zatiaľ nevystavujeme
void handleRoi()
{
  if (server.method() == HTTP_POST)
  {
    String body = server.arg("plain");
    if (body.length() == 0)
    {
      server.send(400, "application/json", "{\"status\":\"error\",\"reason\":\"empty body\"}");
      return;
    }
    StaticJsonDocument<200> doc;
    auto err = deserializeJson(doc, body);
    if (err)
    {
      server.send(400, "application/json", "{\"status\":\"error\",\"reason\":\"invalid json\"}");
      return;
    }
    int x = doc["x"] | -1, y = doc["y"] | -1, w = doc["w"] | -1, h = doc["h"] | -1;
    if (x < 0 || y < 0 || w <= 0 || h <= 0)
    {
      server.send(400, "application/json", "{\"status\":\"error\",\"reason\":\"invalid values\"}");
      return;
    }
    roi = {x, y, w, h};
    if (!saveRoiToNvs(roi))
    {
      server.send(500, "application/json", "{\"status\":\"error\",\"reason\":\"nvs save failed\"}");
      return;
    }
  }
  StaticJsonDocument<200> res;
  res["roi_x"] = roi.x;
  res["roi_y"] = roi.y;
  res["roi_w"] = roi.w;
  res["roi_h"] = roi.h;
  res["status"] = "ok";
  String response;
  serializeJson(res, response);
  server.send(200, "application/json", response);
}

// ---- Kamera ----
void startCamera()
{
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 10000000;     // stabilné
  config.frame_size = FRAMESIZE_SXGA; // 1280x1024
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 18; // 15..22 (menšie číslo = vyššia kvalita)
  config.fb_count = 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed: 0x%x\n", err);
    while (true)
      delay(1000);
  }
}

// ---- CAPTURE TASK ----
void captureTask(void *param)
{
  const TickType_t period = pdMS_TO_TICKS(1000 / TARGET_FPS);
  TickType_t lastWake = xTaskGetTickCount();

  for (;;)
  {
    camera_fb_t *fb = esp_camera_fb_get(); // JPEG, SXGA
    if (fb)
    {
      if (fb->len <= jpeg_wr_cap)
      {
        memcpy(jpeg_wr, fb->buf, fb->len);
        // === Variant B: publish a copy for analysis without blocking /shot.jpg ===
        if (ana_back && ana_front && fb->len <= ana_back_cap)
        {
          memcpy(ana_back, fb->buf, fb->len);
          // Pointer swap and sequence bump (single-writer, single-reader pattern)
          uint8_t *tmp = ana_front;
          ana_front = ana_back;
          ana_back = tmp;
          ana_front_len = fb->len;
          ana_seq = ana_seq + 1;
          if (anaSem)
            xSemaphoreGive(anaSem);
        }

        xSemaphoreTake(jpegMtx, portMAX_DELAY);
        if (!rd_in_use)
        {
          uint8_t *old_rd = jpeg_rd;
          size_t old_cap = jpeg_rd_cap;
          jpeg_rd = jpeg_wr;
          jpeg_rd_len = fb->len;
          jpeg_rd_cap = jpeg_wr_cap;
          jpeg_wr = old_rd;
          jpeg_wr_cap = old_cap;
          last_frame_ms = millis();
        }
        xSemaphoreGive(jpegMtx);
      }
      esp_camera_fb_return(fb);
    }
    vTaskDelayUntil(&lastWake, period);
  }
}

void startCaptureLoop()
{
  Serial.println("[boot] startCaptureLoop() enter");

  if (!jpegMtx) jpegMtx = xSemaphoreCreateMutex();
  if (!jpegMtx) { Serial.println("[ERR] Cannot create jpegMtx"); return; }

  // --- JPEG publish buffers ---
  jpeg_rd = (uint8_t *)heap_caps_malloc(JPEG_BUF_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  jpeg_wr = (uint8_t *)heap_caps_malloc(JPEG_BUF_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!jpeg_rd || !jpeg_wr) {
    Serial.println("[WARN] JPEG PSRAM alloc failed, retry DRAM");
    if (!jpeg_rd) jpeg_rd = (uint8_t *)heap_caps_malloc(JPEG_BUF_SIZE, MALLOC_CAP_8BIT);
    if (!jpeg_wr) jpeg_wr = (uint8_t *)heap_caps_malloc(JPEG_BUF_SIZE, MALLOC_CAP_8BIT);
  }
  if (!jpeg_rd || !jpeg_wr) { Serial.println("[ERR] JPEG prealloc failed (both)"); return; }
  jpeg_rd_cap = JPEG_BUF_SIZE; jpeg_wr_cap = JPEG_BUF_SIZE;

  // --- Analysis buffers (Variant B) ---
  if (!ana_front) {
    size_t cap = jpeg_wr_cap > 0 ? jpeg_wr_cap : JPEG_BUF_SIZE;
    ana_front = (uint8_t*)heap_caps_malloc(cap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    ana_back  = (uint8_t*)heap_caps_malloc(cap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!ana_front || !ana_back) {
      Serial.println("[WARN] Analysis PSRAM alloc failed, retry DRAM");
      if (!ana_front) ana_front = (uint8_t*)heap_caps_malloc(cap, MALLOC_CAP_8BIT);
      if (!ana_back)  ana_back  = (uint8_t*)heap_caps_malloc(cap, MALLOC_CAP_8BIT);
    }
    if (!ana_front || !ana_back) {
      Serial.println("[ERR] Analysis buffers alloc failed");
      // stále môžeme bežať bez analýzy, ale task potom nespúšťaj
    } else {
      memset(ana_front, 0, cap);
      memset(ana_back,  0, cap);
      ana_back_cap = cap;
    }
  }

  if (!anaSem) anaSem = xSemaphoreCreateBinary();

  Serial.printf("[boot] analysis buffers: front=%p back=%p cap=%u\n",
                ana_front, ana_back, (unsigned)ana_back_cap);

  // --- Create tasks ---
  TaskHandle_t hCap = nullptr;
  BaseType_t rcCap = xTaskCreatePinnedToCore(captureTask, "captureTask",
                                             12288, nullptr, 2, &hCap, 1);
  Serial.printf("[boot] captureTask rc=%d handle=%p\n", (int)rcCap, hCap);

  TaskHandle_t hAna = nullptr;
  if (ana_front && ana_back) {
    BaseType_t rcAna = xTaskCreatePinnedToCore(analysisTask, "analysisTask",
                                               12288, nullptr, 1, &hAna, 0);
    Serial.printf("[boot] analysisTask rc=%d handle=%p\n", (int)rcAna, hAna);
  } else {
    Serial.println("[boot] analysisTask not started (no buffers)");
  }

  Serial.printf("[boot] pre-create captureTask, jpeg_rd=%p, jpeg_wr=%p, caps rd=%u wr=%u\n",
                jpeg_rd, jpeg_wr, (unsigned)jpeg_rd_cap, (unsigned)jpeg_wr_cap);
  Serial.println("[boot] startCaptureLoop() ok");
}


// ---- Setup/Loop ----
void setup()
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);
  delay(200);

  // Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info)
               {
    if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
      Serial.printf("\n[WiFi] Disconnected, reason=%d\n", info.wifi_sta_disconnected.reason);
    } else if (event == ARDUINO_EVENT_WIFI_STA_CONNECTED) {
      Serial.printf("\n[WiFi] Associated to AP\n");
    } else if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
      Serial.printf("\n[WiFi] Got IP: %s\n", WiFi.localIP().toString().c_str());
    } });
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
  int best = -1000, bestCh = 0;
  uint8_t bestBssid[6] = {0};
  for (int i = 0; i < n; i++)
  {
    if (WiFi.SSID(i) == WIFI_SSID && WiFi.RSSI(i) > best)
    {
      best = WiFi.RSSI(i);
      bestCh = WiFi.channel(i);
      memcpy(bestBssid, WiFi.BSSID(i), 6);
    }
  }
  if (best > -1000)
  {
    Serial.printf("[WiFi] Locking to ch=%d BSSID %02X:%02X:%02X:%02X:%02X:%02X RSSI=%d\n",
                  bestCh, bestBssid[0], bestBssid[1], bestBssid[2], bestBssid[3], bestBssid[4], bestBssid[5], best);
    WiFi.begin(WIFI_SSID, WIFI_PASS, bestCh, bestBssid, true);
  }
  else
  {
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  }

  Serial.printf("Connecting to SSID %s", WIFI_SSID);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000)
  {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() != WL_CONNECTED)
    Serial.println("\n[WiFi] Connect timeout.");
  else
    Serial.println("\nWiFi connected, IP: " + WiFi.localIP().toString());

  // Kamera
  startCamera();

  // ROI z NVS
  loadRoiFromNvs(roi);

  // Capture loop
  startCaptureLoop();

  // HTTP routy
  server.on("/", handleRoot);
  server.on("/shot.jpg", HTTP_GET, handleShotJpg);
  server.on("/health", HTTP_GET, handleHealth);
  // server.on("/roi", handleRoi);

  server.begin();
  Serial.println("HTTP server started on port 8080");
}

void loop()
{
  server.handleClient();
}

// === Analysis task skeleton (Variant B) ===
// Consumes only the latest published JPEG (ana_front/ana_front_len).
// Does NOT block capture or /shot.jpg. TJpgDec-based ROI analysis will be added later.
void analysisTask(void *arg)
{
  Serial.println("[analysis] started");
  uint32_t t0 = millis();
  uint32_t local_seen = 0;
  for (;;)
  {
    // Wait for a new frame or timeout to avoid starving others
    if (anaSem)
    {
      xSemaphoreTake(anaSem, pdMS_TO_TICKS(100));
    }
    else
    {
      vTaskDelay(pdMS_TO_TICKS(30));
    }

    // Try to pick up the latest snapshot
    uint32_t s1 = ana_seq;
    size_t len = ana_front_len;
    uint8_t *ptr = ana_front;
    uint32_t s2 = ana_seq;
    if (s1 != s2)
    {
      // swapped during read; retry next loop
      continue;
    }
    if (s1 == local_seen || ptr == nullptr || len == 0)
    {
      vTaskDelay(pdMS_TO_TICKS(20)); // nežiň CPU/Serial
      continue;                      // nothing new to process
    }
    local_seen = s1;

    // --- Placeholder for ROI analysis ---
    // Here we'll run TJpgDec with scale=1/8 and accumulate red channel in ROI.
    // For now, we just set a dummy measurement to prove the pipeline.
    uint16_t fake_r_mean = 0; // TODO: compute from JPEG
    g_last_R_mean = fake_r_mean;

    // Basic EMA/hysteresis state update placeholder
    float ema = EMA_ALPHA * (float)fake_r_mean + (1.0f - EMA_ALPHA) * g_ema;
    g_ema = ema;
    uint32_t now = millis();
    if (!g_led_on)
    {
      if ((uint16_t)ema >= TH_ON && (now - g_last_pulse_ms) >= REFRACT_MS)
      {
        g_led_on = true;
        g_pulse_counter++;
        g_last_interpulse_ms = now - g_last_pulse_ms;
        g_last_pulse_ms = now;
      }
    }
    else
    {
      if ((uint16_t)ema <= TH_OFF)
      {
        g_led_on = false;
      }
    }
  }
}
