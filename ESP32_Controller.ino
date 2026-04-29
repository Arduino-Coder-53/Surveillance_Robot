#include <esp_now.h>
#include <WiFi.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <TJpg_Decoder.h>

TFT_eSPI tft = TFT_eSPI();

uint8_t robotAddress[] = {0x1C, 0xDB, 0xD4, 0x75, 0x0D, 0xB0}; //esp32s3 sense mac

#define BTN_UP 13
#define BTN_DOWN 27
#define BTN_LEFT 14
#define BTN_RIGHT 12
#define BTN_SPD_P 25 
#define BTN_SPD_M 33

typedef struct struct_video {
  uint8_t msg_type;
  uint16_t total_chunks;
  uint16_t current_chunk;
  uint16_t data_length;
  uint8_t image_data[230];
} struct_video;

typedef struct struct_cmd {
  uint8_t msg_type;
  uint8_t btn_data;
} struct_cmd;

struct_video videoData;
struct_cmd cmdData;
esp_now_peer_info_t peerInfo;

uint8_t jpeg_buffer[30000];
uint32_t current_jpeg_size = 0;
bool frame_ready = false;
unsigned long lastCmdSendTime = 0;
unsigned long lastFrameTime = 0;

bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t *bitmap) {
  if (y >= tft.height()) return 0;
  tft.pushImage(x, y, w, h, bitmap);
  return 1;
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  uint8_t type = incomingData[0];

  if (type == 0) {
    lastFrameTime = millis();
    memcpy(&videoData, incomingData, sizeof(videoData));
    uint16_t offset = videoData.current_chunk * 230;

    if (offset + videoData.data_length < sizeof(jpeg_buffer)) {
      memcpy(jpeg_buffer + offset, videoData.image_data, videoData.data_length);
    }

    if (videoData.current_chunk == videoData.total_chunks - 1) {
      current_jpeg_size = offset + videoData.data_length;
      frame_ready = true;
    }
  }
}


void drawHUD() {
  bool isConnected = (millis() - lastFrameTime < 1000);

  tft.setTextDatum(MC_DATUM);


  if (isConnected) {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.drawString("LINK: ACTIVE", 120, 30, 2);
  } else {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("LINK: LOST  ", 120, 30, 2);
  }
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.drawString("CTRL: READY", 120, 210, 2);

  tft.drawRect(38, 58, 164,124, TFT_WHITE);
}

void setup() {
  Serial.begin(115200);
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_LEFT, INPUT_PULLUP);
  pinMode(BTN_RIGHT, INPUT_PULLUP);
  pinMode(BTN_SPD_P, INPUT_PULLUP);
  pinMode(BTN_SPD_M, INPUT_PULLUP);

  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);
  tft.setSwapBytes(false);

  TJpgDec.setJpgScale(1);
  TJpgDec.setSwapBytes(true);
  TJpgDec.setCallback(tft_output);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) return;

  esp_now_register_recv_cb(OnDataRecv);

  memcpy(peerInfo.peer_addr, robotAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

void loop() {
  if (frame_ready) {
    TJpgDec.drawJpg(40, 60, jpeg_buffer, current_jpeg_size);
    frame_ready = false;
    current_jpeg_size = 0;
  }
  if (millis() - lastCmdSendTime > 50) {
    cmdData.msg_type = 1;
    cmdData.btn_data = 0;

    if (digitalRead(BTN_UP) == LOW) cmdData.btn_data |= (1 << 7);
    if (digitalRead(BTN_DOWN) == LOW) cmdData.btn_data |= (1 << 6);
    if (digitalRead(BTN_LEFT) == LOW) cmdData.btn_data |= (1 << 5);
    if (digitalRead(BTN_RIGHT) == LOW) cmdData.btn_data |= (1 << 4);
    if (digitalRead(BTN_SPD_P) == LOW) cmdData.btn_data |= (1 << 3);
    if (digitalRead(BTN_SPD_M) == LOW) cmdData.btn_data |= (1 << 2);

    esp_now_send(robotAddress, (uint8_t *)&cmdData, sizeof(cmdData));
    lastCmdSendTime = millis();
  }

  static unsigned long lastHudUpdate = 0;
  if (millis() - lastHudUpdate > 500) {
    drawHUD();
    lastHudUpdate = millis();
  }


  yield();
}