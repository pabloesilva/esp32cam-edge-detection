#include "esp_camera.h"
#include "camera_pins.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------- config display OLED ----------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

// ---------- DEFINICION DE PINES ----------
#define I2C_SDA 14      
#define I2C_SCL 15      
// Potenciómetro -> pin ADC 
#define POT_PIN 2       
// Botón 
#define BUTTON_PIN 13  // GPIO13 (botón activo en BAJO)
// Flash LED del módulo ESP32-CAM
#define FLASH_PIN 4

// OLED I2C address
#ifndef OLED_ADDR
#define OLED_ADDR 0x3C
#endif

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Camera config
#define CAM_FRAME_SIZE FRAMESIZE_QQVGA // 160x120 px
#define CAM_PIXEL_FORMAT PIXFORMAT_GRAYSCALE

// kernels de Sobel
static const int8_t sobel_x[3][3] = {  // vertical
  { -1, 0, 1 },
  { -2, 0, 2 },
  { -1, 0, 1 }
};
static const int8_t sobel_y[3][3] = {  // horizontal
  { -1, -2, -1 },
  {  0,  0,  0 },
  {  1,  2,  1 }
};

// matriz Bayer 4x4 
const uint8_t BAYER4[4][4] = {
  {0, 8, 2,10},
  {12,4,14,6},
  {3,11,1, 9},
  {15,7,13,5}
};

// buffers estaticos
static uint8_t crop_buf[SCREEN_WIDTH * SCREEN_HEIGHT];       // 8192
static uint8_t edge_crop[SCREEN_WIDTH * SCREEN_HEIGHT];      // 8192
static uint8_t frameBuf[(SCREEN_WIDTH * SCREEN_HEIGHT) / 8]; // 1024

// parametros de binarización
int bin_threshold = 80; // 0..255 (valor inicial)
int mode_bin = 1;       // 0 = threshold, 1 = Bayer

// suavizado de potenciometro
float thresh_f = (float)bin_threshold;
const float POT_ALPHA = 0.14f; // mayor = más reactivo, menor = más suave
int potTargetValue = 0;        
const int POT_DEADBAND = 10; 

// antirebote botón
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 40; // ms

int lastButtonReading = HIGH;   // lectura anterior cruda
int buttonStableState = HIGH;   // estado estable actual

// pulsacion larga boton
unsigned long pressStartTime = 0;
bool buttonPressed = false;
bool longTriggerActive = false;
const unsigned long LONG_PRESS_MS = 3000; // 3 segundos

// texto temporal en display
bool showOverlay = false;
unsigned long overlayUntil = 0;

// nombres de modos en overlay 
const char* MODE_NAMES[] = {"THRESH","BAYER"};

// ---------- camera init ----------
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
  config.frame_size = CAM_FRAME_SIZE;
  config.jpeg_quality = 12;
  config.fb_count = 2;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed 0x%x\n", err);
    while (true) delay(1000);
  }
}

// ---------- Funcion Sobel ----------
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

// ---------- recortar centro ----------
void crop_center(const uint8_t *src, int sw, int sh, uint8_t *dst, int tw, int th) {
  int sx0 = (sw - tw) / 2;
  int sy0 = (sh - th) / 2;
  if (sx0 < 0) sx0 = 0;
  if (sy0 < 0) sy0 = 0;
  for (int y = 0; y < th; y++) {
    const uint8_t *srow = src + (sy0 + y) * sw + sx0;
    uint8_t *drow = dst + y * tw;
    memcpy(drow, srow, tw);
  }
}

// ---------- modos de filtrado para mostrar en display ----------
void pack_from_crop_to_frame(const uint8_t *src, uint8_t *outBuf, int sw, int sh) {
  int tw = SCREEN_WIDTH;
  int th = SCREEN_HEIGHT;
  int outBytes = (tw * th) / 8;
  memset(outBuf, 0, outBytes);

  for (int y = 0; y < th; y++) {
    for (int x = 0; x < tw; x++) {
      uint8_t v = src[y * sw + x]; // 0..255
      bool pixelOn = false;

      if (mode_bin == 0) {  // simple threshold
        pixelOn = (v > bin_threshold);
      } else {              // Bayer dithering
        float local = ((BAYER4[y & 3][x & 3] + 0.5f) / 16.0f) * 255.0f;
        float biased = local + (bin_threshold - 128);
        pixelOn = (v > biased);
      }

      if (pixelOn) {
        int bi = y * (tw >> 3) + (x >> 3);
        outBuf[bi] |= (1 << (7 - (x & 7)));
      }
    }
  }
}

// ---------- leer potenciometro ----------
void readPotAndUpdateThreshold() {
  int raw = analogRead(POT_PIN);
  int currentMapped = map(raw, 0, 4095, 0, 255);

  // 1. ¿Se movió el pot lo suficiente (fuera del ruido)?
  if (abs(currentMapped - potTargetValue) > POT_DEADBAND) {
    // SÍ: El usuario está girando el pot.
    // Actualizamos el valor "objetivo" al que el filtro debe ir.
    potTargetValue = currentMapped;
    
    // Y SÓLO AHORA mostramos el overlay
    showOverlay = true;
    overlayUntil = millis() + 900;
    Serial.printf("pot moved-> new target=%d\n", potTargetValue);
  }
  
  // 2. El filtro suave (thresh_f) SIEMPRE se mueve hacia el último "potTargetValue"
  thresh_f = (POT_ALPHA * (float)potTargetValue) + ((1.0f - POT_ALPHA) * thresh_f);
  
  // 3. Actualizamos el threshold final (redondeado)
  // Nota: Ya no necesitamos el "if" aquí, porque el overlay se activó arriba.
  bin_threshold = (int)(thresh_f + 0.5f);
}

// ---------- manejo del boton ----------
void handleButton() {                     // presion corta: cambiar modo, presion larga: encender led flash
  int reading = digitalRead(BUTTON_PIN);
  // registrar cambio 
  if (reading != lastButtonReading) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // si ha pasado el tiempo de debounce, considerar el cambio
    if (reading != buttonStableState) {
      buttonStableState = reading;
      if (buttonStableState == LOW) {
        // botón acaba de ser presionado 
        pressStartTime = millis();
        buttonPressed = true;
        longTriggerActive = false; // reset
      } else {
        // botón acaba de ser liberado 
        if (buttonPressed) {
          unsigned long held = millis() - pressStartTime;
          if (longTriggerActive) {
            // Liberación tras pulsación larga: apagar flash
            digitalWrite(FLASH_PIN, LOW);
            longTriggerActive = false;
            Serial.println("Long press released - flash OFF");
          } else {
            // Liberación tras pulsación corta: cambiar modo
            mode_bin = (mode_bin + 1) % 2;
            showOverlay = true;
            overlayUntil = millis() + 1000;
          }
          buttonPressed = false;
        }
      }
    }
  }

  lastButtonReading = reading;

  // verificar si se mantiene presionado para activación larga
  if (buttonPressed && !longTriggerActive) {
    if ((millis() - pressStartTime) >= LONG_PRESS_MS) {
      // activación larga detectada: encender flash
      longTriggerActive = true;
      digitalWrite(FLASH_PIN, HIGH);
      showOverlay = true;
      overlayUntil = millis() + 1000;
      Serial.println("Long press detected - flash ON");
    }
  }
}

// ---------- dibujar overlay sobre cada frame ----------
void drawOverlay(){
  char buf[32];
  snprintf(buf, sizeof(buf), "T:%03d M:%s", bin_threshold, MODE_NAMES[mode_bin]);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.fillRect(0, 0, SCREEN_WIDTH, 10, SSD1306_BLACK); // clear top line
  display.setCursor(0, 0);
  display.print(buf);
}

// ---------- setup & loop ----------
void setup() {
  Serial.begin(115200);
  delay(100);

  // iniciar I2C 
  Wire.begin(I2C_SDA, I2C_SCL);

  // iniciar display OLED 
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("SSD1306 allocation failed");
    while (true) delay(1000);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Init camera...");
  display.display();

  // iniciar camara
  initCamera();

  // ADC config 
  analogReadResolution(12); // 0..4095
  #ifdef analogSetPinAttenuation
    analogSetPinAttenuation(POT_PIN, ADC_11db);
  #endif

  // boton (con pullup)
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // flash pin (salida, estado bajo)
  pinMode(FLASH_PIN, OUTPUT);
  digitalWrite(FLASH_PIN, LOW);

  // Lee el valor inicial del potenciómetro para arrancar
  potTargetValue = map(analogRead(POT_PIN), 0, 4095, 0, 255);
  bin_threshold = potTargetValue; // Asigna el valor inicial
  thresh_f = (float)bin_threshold; // Sincroniza el filtro suave
}

void loop() {
  readPotAndUpdateThreshold();
  handleButton();

  // capturar frame de camara
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    delay(50);
    return;
  }

  int sw = fb->width;
  int sh = fb->height;

  // 1) recortar el centro a 128x64
  crop_center(fb->buf, sw, sh, crop_buf, SCREEN_WIDTH, SCREEN_HEIGHT);

  // 2) aplicar sober en el recorte
  applySobel(crop_buf, edge_crop, SCREEN_WIDTH, SCREEN_HEIGHT);

  // 3) armar el buffer para display segun el modo seleccionado
  pack_from_crop_to_frame(edge_crop, frameBuf, SCREEN_WIDTH, SCREEN_HEIGHT);

  // 4) mostrar en display
  display.clearDisplay();
  display.drawBitmap(0, 0, frameBuf, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);

  // agregar overlay si es necesario
  if (showOverlay) {
    drawOverlay();
    if (millis() > overlayUntil) showOverlay = false;
  }

  display.display();

  esp_camera_fb_return(fb);

  // pequeño delay para estabilidad de fps
  delay(30);
}
