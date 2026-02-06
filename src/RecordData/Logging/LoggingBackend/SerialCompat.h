// serial_compat.h
#pragma once
#include <Arduino.h>

// ---------- Teensy (Teensyduino) ----------
#if defined(TEENSYDUINO)
using SerialUART_t = HardwareSerial;  // Serial1, Serial2, ...
using SerialUSB_t = usb_serial_class; // Serial (USB CDC)

// ---------- ESP32 ----------
#elif defined(ENV_ESP)
// Classic ESP32 has only UARTs. ESP32-S2/S3 may expose USB CDC as USBCDC.
using SerialUART_t = HardwareSerial; // Serial, Serial1, Serial2
using SerialUSB_t = HWCDC;

// ---------- STM32 (stm32duino) ----------
#elif defined(ENV_STM)
using SerialUART_t = HardwareSerial;
using SerialUSB_t = HardwareSerial; // works whether it's USBSerial or USBCDC

// ---------- NATIVE ----------
#elif defined(NATIVE)
using SerialUART_t = SerialClass;
using SerialUSB_t = SerialClass; // works whether it's USBSerial or USBCDC

// ---------- Fallback (unknown core) ----------
#else
using SerialUART_t = HardwareSerial;
using SerialUSB_t = HardwareSerial;
#endif
