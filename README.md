# 1-Bit Camera: ESP32-CAM Edge Detection & Dithering

![Project Banner](img\fig1.png)
*Figure 1: Comparison of the 4 visualization pipelines (Sobel-Threshold, Sobel-Bayer, Real-Threshold, Real-Bayer).*

## Overview

**1-Bit Camera** is an embedded image processing system based on the **ESP32-CAM**. It captures video in real-time, processes it using multiple DSP pipelines (Edge Detection and Dithering), and displays the result on a 128x64 OLED screen.

Unlike standard camera examples, this project implements a custom video pipeline optimized for low-resource hardware, achieving fluid frame rates through **Double Buffering** and **Unified Resolution** architecture. It transforms the ESP32 into a retro-style digital camera capable of saving processed 1-bit aesthetic images to an SD card.

## Key Features

* **Real-Time Processing:** Zero-lag viewfinder running at QQVGA resolution.
* **4 Visualization Modes:** Switch instantly between edge detection and realistic dithering.
* **1-Bit Aesthetics:** Uses Sobel operators and Bayer Matrix Dithering to simulate greyscale on monochrome displays.
* **SD Card Storage:** Captures and saves the exact processed frame as a `.BMP` file.
* **Hardware Controls:** Physical potentiometer for threshold/bias adjustment and a button for mode switching/shutter.
* **Optimized Performance:** I2C bus overclocked to 400kHz and DMA-based double buffering.

## Hardware Setup

### Components
* **Microcontroller:** ESP32-CAM (AI-Thinker module).
* **Display:** 0.96" SSD1306 OLED (I2C).
* **Input:** 10kΩ Potentiometer, Push Button.
* **Storage:** MicroSD Card (FAT32).

### Wiring / Pinout
> **CRITICAL NOTE:** The OLED display uses pins `GPIO 1` and `GPIO 3` (usually Serial TX/RX). This allows `GPIO 14` and `GPIO 15` to remain free for the SD Card communication. You will not see Serial Monitor output after the display initializes.

| Component | Pin Function | ESP32-CAM Pin | Notes |
| :--- | :--- | :--- | :--- |
| **OLED** | SDA | **GPIO 1** | (U0TXD) |
| **OLED** | SCL | **GPIO 3** | (U0RXD) |
| **Potentiometer** | Signal | **GPIO 12** | Do not use GPIO 2 (Conflicts with SD) |
| **Button** | Signal | **GPIO 13** | Input Pull-up |
| **SD Card** | CLK/CMD/DAT | **14, 15, 2** | Internal slot usage |

![Breadboard Setup](img\fig2.png)
*Figure 2: Physical implementation on protoboard.*

## Installation

1.  **Clone the repo:**
    ```bash
    git clone [https://github.com/your-username/esp32cam-edge-detection.git](https://github.com/your-username/esp32cam-edge-detection.git)
    ```
2.  **Dependencies:**
    Install the following libraries in Arduino IDE or PlatformIO:
    * `Adafruit GFX Library`
    * `Adafruit SSD1306`
3.  **Configuration:**
    * Select Board: **esp32cam**.
    * **Upload Speed:** 115200.
4.  **Upload:**
    * Disconnect the OLED from pins 1 & 3 during upload.
    * Connect GPIO 0 to GND.
    * Reset and Upload.
    * Remove GPIO 0 jumper, reconnect OLED, and Reset.

## Controls & Usage

| Control | Action | Function |
| :--- | :--- | :--- |
| **Potentiometer** | Rotate | Adjusts the **Threshold (T)** value. Affects contrast in Threshold modes and bias in Bayer modes. |
| **Button** | Short Click | Cycles through the **4 Processing Modes**. |
| **Button** | Long Press (>0.6s) | **Shutter**. Saves the current frame to the SD Card. |

### Processing Pipelines

The system offers 4 distinct visual styles:

1.  **SOBEL-TR (Edge-Threshold):** High-contrast edge detection. Displays only structural outlines.
2.  **SOBEL-BY (Edge-Bayer):** Textured edge detection. Adds depth to borders using dithering.
3.  **BAYER (Real-Bayer):** "GameBoy Camera" style. Simulates greyscale photos using ordered dithering. Best for general photography.
4.  **THRESH (Real-Threshold):** High contrast "Sin City" style. Pure black and white based on light intensity.

## File Output

Images are saved to the root of the SD card as `pic0.bmp`, `pic1.bmp`, etc.
* **Format:** Windows BMP.
* **Resolution:** 160x120 pixels.
* **Depth:** 8-bit Grayscale (containing binary pixel data).

## License

This project is open-source. Feel free to fork and improve!