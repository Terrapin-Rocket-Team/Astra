---
title: Installation
---

You may skip this installation step if you are working in a repo that already uses Astra and you are just looking to get introduced.

!!! warning "Important!"
    In order for PlatformIO to recognize that you are working on a PIO project, you *must* open VSCode in the root directory of that project—the directory that has the `platformio.ini` file in it. Without this, PlatformIO **will not initialize** and you will be unable to build or use proper Intellisense.

---

## Prerequisites

Before you begin, make sure you have the following:

- Basic knowledge of C++ programming (knowledge of what a pointer is and how to use one)
- VSCode and the PlatformIO (PIO) extension installed [[Installation Guide](https://docs.platformio.org/en/latest/core/installation/index.html)]
- Basic knowledge of VSCode and the PlatformIO interface
- Basic knowledge of Git and GitHub
- A supported development board:
    - **Teensy 4.1** (recommended, most tested)
    - **STM32** boards (e.g., STM32F4, STM32F7, STM32H7)
    - **ESP32** boards

---

## Installation

### Create a New PlatformIO Project

Pick any folder to create it in, and create a new project. Choose the Arduino framework and select your target board:

- **Teensy 4.1**: `board = teensy41`, `platform = teensy`
- **STM32**: `board = <your_stm32_board>`, `platform = ststm32`
- **ESP32**: `board = <your_esp32_board>`, `platform = espressif32`

### Modify the `platformio.ini` File

Add a dependency to Astra by adding the highlighted lines to your `platformio.ini` file, found in your project's root directory.

**For Teensy 4.1:**

```ini linenums="10" hl_lines="5-6" title=""
[env:teensy41]
platform = teensy
board = teensy41
framework = arduino
lib_deps =
    https://github.com/Terrapin-Rocket-Team/Astra.git#v3.1.1
```

**For STM32:**

```ini linenums="10" hl_lines="5-6" title=""
[env:your_stm32]
platform = ststm32
board = nucleo_f767zi  ; or your specific board
framework = arduino
lib_deps =
    https://github.com/Terrapin-Rocket-Team/Astra.git#v3.1.1
```

**For ESP32:**

```ini linenums="10" hl_lines="5-6" title=""
[env:esp32dev]
platform = espressif32
board = esp32dev  ; or your specific board
framework = arduino
lib_deps =
    https://github.com/Terrapin-Rocket-Team/Astra.git#v3.1.1
```

!!! info
    You may add multiple dependencies by appending new indented lines under `lib_deps`.

!!! tip
    We recommend always attaching the version specifier to the end of the URL, as Astra undergoes frequent breaking changes. For GitHub links, this looks like `#{tag}`. We give our releases (a.k.a. tags) semantic versioning[^sv] numbers like `v3.1.1`.

Now save the file. You should notice PlatformIO start to download Astra and all of its dependencies. It may take a few minutes.

---

## Platform-Specific Configuration

### Teensy 4.1

Teensy 4.1 is the most tested platform and requires no additional configuration. It supports:

- SD card via SPI (`SD` library) or SDIO (`SD_SDIO`)
- High-speed USB logging
- Full sensor support

### STM32

STM32 boards support varies by model. Ensure your board has:

- Sufficient flash (256KB+ recommended)
- Sufficient RAM (128KB+ recommended)
- SD card support via SPI or SDIO depending on your board

For SDIO support on STM32, you may need to configure pins in your platformio.ini:

```ini
build_flags =
    -D SDIO_ENABLED
```

### ESP32

ESP32 boards have built-in features that work well with Astra:

- Internal flash filesystem (SPIFFS/LittleFS)
- SD card via SPI
- WiFi for remote logging (requires custom implementation)

ESP32 typically requires explicit filesystem initialization in your code:

```cpp
#include <SPIFFS.h>

void setup() {
    if (!SPIFFS.begin(true)) {
        LOGE("SPIFFS initialization failed");
    }
    // Rest of setup
}
```

---

## Add Astra to Your Code

Looking at the folder structure, PlatformIO should have created a `src` folder with a `main.cpp` in it. To link Astra to this main file, add the include:

```cpp hl_lines="2" title="src/main.cpp"
#include <Arduino.h>
#include <Astra.h>

void setup() {
    Serial.begin(115200);
    // Your initialization here
}

void loop() {
    // Your code here
}
```

---

## Build the Project

The last step is to verify everything is working by building the project.

You can use any of PlatformIO's `build` buttons to achieve this. If you don't know where the buttons are, we recommend using the toolbar at the bottom of the screen, where the `build` command is represented by the checkmark (✓).

The terminal should output:

```
========[SUCCESS] Took ##.## seconds=======
```

If not, try to understand the error message. Common issues:

- **Missing dependencies**: PlatformIO will automatically install them on next build
- **Platform not installed**: Click the error to install the platform
- **Board not found**: Verify your `board =` value in platformio.ini

If you get stuck, please message any club members with experience working in PlatformIO or Astra.

---

## Verify Installation

Create a simple test program to verify Astra is working:

```cpp
#include <Arduino.h>
#include <Astra.h>

void setup() {
    Serial.begin(115200);
    delay(2000);  // Wait for serial

    LOGI("Astra installation successful!");
    LOGI("Platform: %s", ARDUINO_BOARD);
}

void loop() {
    delay(1000);
}
```

Upload this to your board and open the serial monitor. You should see:

```
0.000 - [INFO] Astra installation successful!
0.001 - [INFO] Platform: TEENSY41
```

(The platform name will vary based on your board.)

---

## Next Steps

Now that Astra is installed, you're ready to start using it! Check out:

- [Basic Use](basic-use.md) - Your first Astra program
- [Introduction](intro.md) - Overview of Astra's architecture
- [Sensors](ifaces/sensor.md) - Working with sensors

---

## Troubleshooting

### "Library not found: Astra"

**Cause**: PlatformIO hasn't downloaded the library yet

**Solution**: Wait for the download to complete, or manually trigger it with `pio lib install` in the terminal

### "No such file or directory: Astra.h"

**Cause**: Include path not set correctly

**Solution**: Ensure you're opening VSCode in the project root (where platformio.ini is located), then rebuild

### Build errors on ESP32

**Cause**: Some Arduino ESP32 core versions have compatibility issues

**Solution**: Specify a known-good platform version in platformio.ini:

```ini
platform = espressif32@6.3.2
```

### Build errors on STM32

**Cause**: Some STM32 boards have limited support in Arduino framework

**Solution**: Verify your board is supported by checking the [PlatformIO STM32 boards list](https://docs.platformio.org/en/latest/platforms/ststm32.html)

### Out of memory errors

**Cause**: Board has insufficient RAM for your configuration

**Solution**: Reduce buffer sizes, logging queue sizes, or sensor update rates

---

## Conclusion

That's all there is to installing the library. Astra supports Teensy 4.1, STM32, and ESP32 platforms with full cross-platform compatibility. You're now ready to move on to using it!

[^sv]: We always use this format, but we don't always follow correct [semantic versioning procedures](https://semver.org/).
