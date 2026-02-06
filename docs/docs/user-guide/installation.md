---
title: Installation
---

# Installation

This guide assumes you are using PlatformIO with the Arduino framework.

---

## Prerequisites

- VS Code + PlatformIO extension
- A supported board (Teensy, STM32, ESP32)
- Basic C++ familiarity

---

## Add Astra to `platformio.ini`

Create a new PlatformIO project and add Astra to `lib_deps`.

```ini title="platformio.ini"
[env:teensy41]
platform = teensy
board = teensy41
framework = arduino
lib_deps =
  https://github.com/Terrapin-Rocket-Team/Astra.git
build_flags =
  -D ENV_TEENSY
```

```ini title="platformio.ini"
[env:stm32h723vehx]
platform = ststm32
board = stm32h723vehx
framework = arduino
lib_deps =
  https://github.com/Terrapin-Rocket-Team/Astra.git
  stm32duino/STM32duino STM32SD
  https://github.com/stm32duino/FatFs.git
build_flags =
  -D ENV_STM
```

```ini title="platformio.ini"
[env:esp32]
platform = espressif32
board = esp32-s3-devkitm-1
framework = arduino
lib_deps =
  https://github.com/Terrapin-Rocket-Team/Astra.git
build_flags =
  -D ENV_ESP
```

!!! tip
    Pin a specific release by appending a tag, e.g. `https://github.com/Terrapin-Rocket-Team/Astra.git#v0.2.0`.

---

## Native Build (SITL)

To run on your desktop for SITL testing, create a native environment:

```ini title="platformio.ini"
[env:native]
platform = native
lib_compat_mode = off
lib_deps =
  https://github.com/DrewBrandt/NativeTestMocks.git
build_flags =
  -D NATIVE=1
  -D ARDUINO=100
  -std=c++17
```

See [SITL](sitl.md) for details.

---

## Verify the Build

Add a basic `main.cpp`:

```cpp title="src/main.cpp"
#include <Arduino.h>
#include <Utils/Astra.h>

void setup() {
    Serial.begin(115200);
}

void loop() {
}
```

Build using PlatformIO. If the build succeeds, Astra is installed correctly.

