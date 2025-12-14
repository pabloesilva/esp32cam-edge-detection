#include "esp_camera.h"
#include "camera_pins.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "FS.h"
#include "SD_MMC.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// --- configuracion de pines ---
#define I2C_SDA 1      
#define I2C_SCL 3      

#define POT_PIN 12      // pin 12 para el pot, el 2 se usa para la sd
#define BUTTON_PIN 13   
#define FLASH_PIN 4     

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- configuracion de camara ---
#define CAM_PIXEL_FORMAT PIXFORMAT_GRAYSCALE
#define FRAMESIZE_UNIFIED FRAMESIZE_QQVGA  // 160x120

// buffers de memoria estatica
static uint8_t crop_buf[128 * 64];       
static uint8_t edge_crop[128 * 64];      
static uint8_t frameBuf[(128 * 64) / 8]; 

// --- variables globales ---
bool sdReady = false; 

// modos de filtrado
// 0: sobel + threshold
// 1: sobel + bayer
// 2: real + bayer
// 3: real + threshold
int currentMode = 0; 
const int TOTAL_MODES = 4;
const char* MODE_NAMES[] = {"SOBEL-TR", "SOBEL-BY", "BAYER", "THRESH"};

int bin_threshold = 80;
float thresh_f = 80.0;
const float POT_ALPHA = 0.2f; 
int potTargetValue = 0;       
const int POT_DEADBAND = 6; 
bool showOverlay = false;
unsigned long overlayUntil = 0;

// boton
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY = 50;
const unsigned long LONG_PRESS_TIME = 600; 
unsigned long pressStartTime = 0;
bool buttonPressed = false;
bool actionTriggered = false; 

// kernels y matrices
static const int8_t sobel_x[3][3] = { { -1, 0, 1 }, { -2, 0, 2 }, { -1, 0, 1 } };
static const int8_t sobel_y[3][3] = { { -1, -2, -1 }, { 0, 0, 0 }, { 1, 2, 1 } };
const uint8_t BAYER4[4][4] = { {0, 8, 2,10}, {12,4,14,6}, {3,11,1, 9}, {15,7,13,5} };

// prototipos
void initCamera();
void initSD();
bool savePhoto(); 
void applySobel(const uint8_t* src, uint8_t* dst, int width, int height);
void crop_center(const uint8_t *src, int sw, int sh, uint8_t *dst, int tw, int th);
void pack_frame(const uint8_t *src, uint8_t *outBuf, int sw, int sh);

void setup() {
  
  // flash no utilizado en esta version
  pinMode(FLASH_PIN, OUTPUT);
  digitalWrite(FLASH_PIN, LOW);

  initSD();

  // iniciar i2c a 400khz
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000); 

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    while(true) delay(100); 
  }
  
  // splash screen simple
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  display.setTextSize(2);
  display.setCursor(34, 15);
  display.println("1-BIT");
  
  display.setCursor(28, 35);
  display.println("CAMERA");
  
  display.display();
  delay(2000); 
  
  // mostrar estado de la sd centrado
  display.clearDisplay();
  
  display.setTextSize(1);
  display.setCursor(0,0);
  display.println("Starting...");
  
  display.setTextSize(2); 
  
  String msg = sdReady ? "SD OK" : "NO SD";
  // centrar texto segun cantidad de caracteres
  int16_t textWidth = msg.length() * 12; 
  int16_t xCenter = (SCREEN_WIDTH - textWidth) / 2;

  display.setCursor(xCenter, 25);
  display.println(msg);
  
  display.display();
  delay(1500);
  
  initCamera();
  
  analogReadResolution(12);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  potTargetValue = map(analogRead(POT_PIN), 0, 4095, 0, 255);
  bin_threshold = potTargetValue;
  thresh_f = (float)bin_threshold;
}

void loop() {
  // 1. lectura del pot
  int raw = analogRead(POT_PIN);
  int currentMapped = map(raw, 0, 4095, 0, 255);
  if (abs(currentMapped - potTargetValue) > POT_DEADBAND) {
    potTargetValue = currentMapped;
    showOverlay = true;
    overlayUntil = millis() + 800;
  }
  thresh_f = (POT_ALPHA * (float)potTargetValue) + ((1.0f - POT_ALPHA) * thresh_f);
  bin_threshold = (int)(thresh_f + 0.5f);

  // 2. lectura del boton
  int reading = digitalRead(BUTTON_PIN);
  if (reading != lastButtonState) lastDebounceTime = millis();
  
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (reading == LOW && !buttonPressed) {
      buttonPressed = true;
      pressStartTime = millis();
      actionTriggered = false;
    }
    if (reading == HIGH && buttonPressed) {
      buttonPressed = false;
      if (!actionTriggered) {
        // click corto: cambiar modo
        currentMode++;
        if (currentMode >= TOTAL_MODES) currentMode = 0;
        
        showOverlay = true;
        overlayUntil = millis() + 1000;
      }
    }
  }
  lastButtonState = reading;

  // 3. pulsacion larga: guardar
  if (buttonPressed && !actionTriggered && (millis() - pressStartTime > LONG_PRESS_TIME)) {
    actionTriggered = true; 
    
    // aviso de guardado
    display.fillRect(0, 0, 128, 10, SSD1306_BLACK);
    display.setCursor(0,0);
    display.setTextSize(1);
    display.println(sdReady ? "SAVING..." : "NO SD");
    display.display();

    if (sdReady) {
      bool ok = savePhoto(); 
      display.setCursor(80,0);
      display.println(ok ? "OK" : "ERR");
    } 
    display.display();
    delay(300); 
  }

  // 4. captura y proceso
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) return;

  // recortar centro
  crop_center(fb->buf, fb->width, fb->height, crop_buf, SCREEN_WIDTH, SCREEN_HEIGHT);
  
  if (currentMode <= 1) {
    // modo sobel
    applySobel(crop_buf, edge_crop, SCREEN_WIDTH, SCREEN_HEIGHT);
  } else {
    // modo 'real', copia directa sin sobel
    memcpy(edge_crop, crop_buf, SCREEN_WIDTH * SCREEN_HEIGHT);
  }
  
  // empaquetar a 1-bit
  pack_frame(edge_crop, frameBuf, SCREEN_WIDTH, SCREEN_HEIGHT);

  display.clearDisplay();
  display.drawBitmap(0, 0, frameBuf, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
  
  // overlay de informacion
  if (showOverlay) {
    display.fillRect(0,0,128,10,SSD1306_BLACK); 
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    // modo a la izquierda
    display.setCursor(0, 1);
    display.print(MODE_NAMES[currentMode]);

    // valor t a la derecha
    String tText = "T:" + String(bin_threshold);
    int16_t tWidth = tText.length() * 6;
    display.setCursor(SCREEN_WIDTH - tWidth, 1);
    display.print(tText);

    if(millis() > overlayUntil) showOverlay = false;
  }
  display.display(); 
  
  esp_camera_fb_return(fb);
}

// guardar foto en sd
bool savePhoto() {
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) return false;

  int fileIndex = 0;
  while(SD_MMC.exists("/pic" + String(fileIndex) + ".bmp")) {
    fileIndex++;
  }
  String path = "/pic" + String(fileIndex) + ".bmp";

  File file = SD_MMC.open(path, FILE_WRITE);
  if(!file){
     esp_camera_fb_return(fb);
     return false;
  }

  int w = fb->width;  
  int h = fb->height; 

  // cabecera bmp estandar (54 bytes)
  uint32_t imageSize = w * h;
  uint32_t fileSize = 54 + 1024 + imageSize;
  uint8_t bmpHeader[54] = {
    0x42, 0x4D, (uint8_t)(fileSize), (uint8_t)(fileSize >> 8), (uint8_t)(fileSize >> 16), (uint8_t)(fileSize >> 24),
    0, 0, 0, 0, 54 + 4, 4, 0, 0, 
    40, 0, 0, 0, (uint8_t)(w), (uint8_t)(w >> 8), (uint8_t)(w >> 16), (uint8_t)(w >> 24),
    (uint8_t)(h), (uint8_t)(h >> 8), (uint8_t)(h >> 16), (uint8_t)(h >> 24),
    1, 0, 8, 0, 0, 0, 0, 0, 
    (uint8_t)(imageSize), (uint8_t)(imageSize >> 8), (uint8_t)(imageSize >> 16), (uint8_t)(imageSize >> 24),
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
  };
  uint32_t offset = 1078;
  bmpHeader[10] = (uint8_t)(offset); bmpHeader[11] = (uint8_t)(offset >> 8);
  uint32_t h_neg = -h; 
  bmpHeader[22] = (uint8_t)(h_neg); bmpHeader[23] = (uint8_t)(h_neg >> 8);
  bmpHeader[24] = (uint8_t)(h_neg >> 16); bmpHeader[25] = (uint8_t)(h_neg >> 24);
  file.write(bmpHeader, 54);

  // paleta de grises
  uint8_t paleta[1024];
  for(int i=0; i<256; i++){
    paleta[i*4] = i; paleta[i*4+1] = i; paleta[i*4+2] = i; paleta[i*4+3] = 0; 
  }
  file.write(paleta, 1024);

  // procesar y escribir pixel por pixel
  uint8_t* src = fb->buf;
  uint8_t lineBuffer[160]; 

  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      int pixelVal = 0;
      int p_idx = y * w + x;

      if (x == 0 || x == w - 1 || y == 0 || y == h - 1) {
         pixelVal = 0;
      } else {
         if (currentMode <= 1) { // sobel
             int gx = (src[p_idx - w + 1] + 2*src[p_idx + 1] + src[p_idx + w + 1]) - 
                      (src[p_idx - w - 1] + 2*src[p_idx - 1] + src[p_idx + w - 1]);
             int gy = (src[p_idx + w - 1] + 2*src[p_idx + w] + src[p_idx + w + 1]) - 
                      (src[p_idx - w - 1] + 2*src[p_idx - w] + src[p_idx - w + 1]);
             int mag = abs(gx) + abs(gy);
             pixelVal = (mag > 255) ? 255 : mag;
         } else { // real
             pixelVal = src[p_idx];
         }
      }

      // binarizar segun modo
      bool pixelOn = false;
      bool useBayer = (currentMode == 1 || currentMode == 2); 

      if (!useBayer) {
        pixelOn = (pixelVal > bin_threshold);
      } else {
        uint8_t b = BAYER4[y & 3][x & 3];
        int thresh = bin_threshold + (b * 16 - 120);
        pixelOn = (pixelVal > thresh);
      }

      lineBuffer[x] = pixelOn ? 255 : 0;
    }
    file.write(lineBuffer, w);
  }

  file.close();
  esp_camera_fb_return(fb);
  return true;
}

void initSD() {
  SD_MMC.setPins(14, 15, 2);
  if(SD_MMC.begin("/sdcard", true)){ 
    sdReady = (SD_MMC.cardType() != CARD_NONE);
  } else {
    sdReady = false;
  }
}

void initCamera() {
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
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = CAM_PIXEL_FORMAT;
  config.frame_size = FRAMESIZE_UNIFIED; 
  config.jpeg_quality = 12;
  // doble buffer para mejorar fps
  config.fb_count = 2; 

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) while(true) delay(1000);
}

void pack_frame(const uint8_t *src, uint8_t *outBuf, int sw, int sh) {
  memset(outBuf, 0, (sw*sh)/8);
  bool useBayer = (currentMode == 1 || currentMode == 2);
  for (int y = 0; y < sh; y++) {
    for (int x = 0; x < sw; x++) {
      uint8_t v = src[y * sw + x];
      bool pixelOn = false;
      if (!useBayer) {
        pixelOn = (v > bin_threshold);
      } else {
        uint8_t b = BAYER4[y & 3][x & 3];
        int thresh = bin_threshold + (b * 16 - 120);
        pixelOn = (v > thresh);
      }
      if (pixelOn) {
        int bi = y * (sw >> 3) + (x >> 3);
        outBuf[bi] |= (1 << (7 - (x & 7)));
      }
    }
  }
}

// filtro sobel
void applySobel(const uint8_t* src, uint8_t* dst, int width, int height) {
  memset(dst, 0, width * height);
  for (int y = 1; y < height - 1; y++) {
    for (int x = 1; x < width - 1; x++) {
      int16_t gx = 0, gy = 0;
      for (int ky = -1; ky <= 1; ky++) {
        for (int kx = -1; kx <= 1; kx++) {
          int v = src[(y + ky) * width + (x + kx)];
          gx += sobel_x[ky + 1][kx + 1] * v;
          gy += sobel_y[ky + 1][kx + 1] * v;
        }
      }
      int mag = abs(gx) + abs(gy);
      dst[y * width + x] = (mag > 255) ? 255 : mag;
    }
  }
}

// recortar imagen al centro
void crop_center(const uint8_t *src, int sw, int sh, uint8_t *dst, int tw, int th) {
  int sx0 = (sw - tw) / 2;
  int sy0 = (sh - th) / 2;
  for (int y = 0; y < th; y++) {
    memcpy(dst + y * tw, src + (sy0 + y) * sw + sx0, tw);
  }
}