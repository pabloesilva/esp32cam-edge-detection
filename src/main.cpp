// ESP32-CAM deteccion de bordes mediante filtro sobel con webserver

#include "esp_camera.h"
#include "camera_pins.h"
#include <WiFi.h>

// credenciales WiFi
const char* ssid = "*******";
const char* password = "******";


// Kernels Sobel
static const int8_t sobel_x[3][3] = {
  { -1, 0, 1 },
  { -2, 0, 2 },
  { -1, 0, 1 }
};
static const int8_t sobel_y[3][3] = {
  { -1, -2, -1 },
  {  0,  0,  0 },
  {  1,  2,  1 }
};

// Inicializa cámara en modo Grayscale raw
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

    // configuraciones importantes
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_GRAYSCALE;
    config.frame_size = FRAMESIZE_QQVGA;
    config.jpeg_quality = 12;
    config.fb_count = 2;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    while (true);
    }
}

// aplicar Sobel sobre buffer en escala de grises
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
            dst[y * width + x] = mag > 255 ? 255 : mag;
        }
    }
}


// streaming comprimido a JPEG con filtro Sobel
void streamHandler(WiFiClient client) {
    const String boundary = "--jpgboundary";
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: multipart/x-mixed-replace; boundary=" + boundary);
    client.println();

    while (client.connected()) {
        camera_fb_t * fb = esp_camera_fb_get();
        
        if (!fb) break;

        int w = fb->width;
        int h = fb->height;
        size_t len = w * h;
        uint8_t * edge_buf = (uint8_t*) malloc(len);

        // buffer raw fb->buf ya esta en escala de grises
        applySobel(fb->buf, edge_buf, w, h);

        // comprimir a JPEG
        uint8_t * out_buf = nullptr;
        size_t out_len = 0;
        bool ok = fmt2jpg(edge_buf, len, w, h, PIXFORMAT_GRAYSCALE, 80, &out_buf, &out_len);

        if (ok) {
            client.printf("%s\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", boundary.c_str(), out_len);
            client.write(out_buf, out_len);
            client.println();
            free(out_buf);
        }

        free(edge_buf);
        esp_camera_fb_return(fb);
        delay(10);
    }
    client.stop();
}

void setup() {
    Serial.begin(115200);
    initCamera();
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println();
    Serial.print("Connected. IP: ");
    Serial.println(WiFi.localIP());
    WiFiServer server(80);
    server.begin();

    while (true) {
        WiFiClient client = server.available();
        if (client) {
            streamHandler(client);
        }
    }
}

void loop() {
}