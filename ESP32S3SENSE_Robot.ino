#include <esp_now.h>
#include <WiFi.h>
#include "esp_camera.h"
uint8_t remoteAddress[] = {0x44, 0x1D, 0x64, 0xF5, 0x31, 0x58};
#define PWMB D10
#define BIN1 D9
#define BIN2 D8
#define STBY D7
#define AIN1 D6
#define AIN2 D5
#define PWMA D4

#define LED_PIN D0
#define SPEED_LED D2

int motorSpeed = 90;
int ledBrightness = 0; 

#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 10
#define SIOD_GPIO_NUM 40
#define SIOC_GPIO_NUM 39
#define Y9_GPIO_NUM 48
#define Y8_GPIO_NUM 11
#define Y7_GPIO_NUM 12
#define Y6_GPIO_NUM 14
#define Y5_GPIO_NUM 16
#define Y4_GPIO_NUM 18
#define Y3_GPIO_NUM 17
#define Y2_GPIO_NUM 15
#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM 47
#define PCLK_GPIO_NUM 13

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


void moveForward() {
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); analogWrite(PWMA, motorSpeed);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); analogWrite(PWMB, motorSpeed);
}

void moveBackward() {
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); analogWrite(PWMA, motorSpeed);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); analogWrite(PWMB, motorSpeed);
}

void turnLeft() {
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); analogWrite(PWMA, motorSpeed); 
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); analogWrite(PWMB, motorSpeed); 
}

void turnRight() {
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); analogWrite(PWMA, motorSpeed); 
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); analogWrite(PWMB, motorSpeed); 
}

void stopMotors() {
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW); analogWrite(PWMA, 0);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW); analogWrite(PWMB, 0);
  digitalWrite(STBY, LOW);
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  uint8_t type = incomingData[0]; 
  
  if (type == 1) { 
    memcpy(&cmdData, incomingData, sizeof(cmdData));

    bool isUp    = cmdData.btn_data & (1 << 7);
    bool isDown  = cmdData.btn_data & (1 << 6);
    bool isLeft  = cmdData.btn_data & (1 << 5);
    bool isRight = cmdData.btn_data & (1 << 4);

    static unsigned long lastAdjustTime = 0;
    bool isAdjusting = false;

    if (millis() - lastAdjustTime > 150) { 
      if (isUp && isLeft) {
        if (motorSpeed <= 245) motorSpeed += 10;
        analogWrite(SPEED_LED, motorSpeed);
        isAdjusting = true;
        lastAdjustTime = millis();
      }

      else if (isDown && isRight) { 
        if (motorSpeed >= 10) motorSpeed -= 10;
        analogWrite(SPEED_LED, motorSpeed);
        isAdjusting = true;
        lastAdjustTime = millis();
      }

      else if (isUp && isRight) {
        if (ledBrightness <= 245) ledBrightness += 10;
        analogWrite(LED_PIN, ledBrightness);
        isAdjusting = true;
        lastAdjustTime = millis();
      }

      else if (isDown && isLeft) {
        if (ledBrightness >= 10) ledBrightness -= 10;
        analogWrite(LED_PIN, ledBrightness);
        isAdjusting = true;
        lastAdjustTime = millis();
      }
    }

    if (isAdjusting) {
        stopMotors();
    } else {
        if (isUp && !isLeft && !isRight) {
          moveForward();
        } else if (isDown && !isLeft && !isRight) {
          moveBackward();
        } else if (isLeft && !isUp && !isDown) {
          turnLeft();
        } else if (isRight && !isUp && !isDown) {
          turnRight();
        } else {
          stopMotors(); 
        }
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PWMA, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(AIN1, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(BIN1, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  analogWrite(LED_PIN, ledBrightness);
  analogWrite(SPEED_LED, motorSpeed);

  stopMotors();

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  memcpy(peerInfo.peer_addr, remoteAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0; config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM; config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM; config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM; config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM; config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM; config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM; config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM; config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM; config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  
  config.pixel_format = PIXFORMAT_JPEG; 
  config.frame_size = FRAMESIZE_QQVGA; 
  config.jpeg_quality = 12; 
  config.fb_count = 2;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  
  sensor_t * s = esp_camera_sensor_get();
  s->set_vflip(s, 1);   
  s->set_hmirror(s, 1); 
}

void loop() {
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) return;

  uint16_t total_chunks = (fb->len + 229) / 230;
  videoData.msg_type = 0; 
  
  for (uint16_t i = 0; i < total_chunks; i++) {
    videoData.total_chunks = total_chunks;
    videoData.current_chunk = i;
    
    uint16_t offset = i * 230;
    videoData.data_length = (offset + 230 > fb->len) ? (fb->len - offset) : 230;
    
    memcpy(videoData.image_data, fb->buf + offset, videoData.data_length);
    esp_now_send(remoteAddress, (uint8_t *) &videoData, sizeof(videoData));
    delay(4); 
  }

  esp_camera_fb_return(fb);
  delay(20); 
}