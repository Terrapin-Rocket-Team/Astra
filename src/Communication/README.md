# Serial Message Router

The SerialMessageRouter is a centralized serial message handler that monitors multiple serial interfaces (UART, USB, etc.) and dispatches messages to registered callbacks based on prefix matching.

## Features

- **Multiple Serial Interfaces** - Monitor any number of serial ports simultaneously (Serial, Serial1-8, USB, etc.)
- **Prefix-Based Routing** - Route messages to callbacks based on message prefixes (e.g., "HITL/", "CMD/", "RAD/")
- **Hot-Swappable** - Any prefix can appear on any interface, enabling flexible routing
- **Builder Pattern API** - Easy, chainable configuration
- **Non-Blocking** - Designed for use in main loop without blocking
- **Default Handler** - Optional fallback for unmatched messages
- **Automatic Buffer Management** - Handles line buffering and overflow protection

## Architecture

```
┌─────────────────────────────────┐
│   Multiple Serial Interfaces    │
│   Serial, Serial8, USB, etc.    │
└────────────┬────────────────────┘
             │ Raw bytes
             ▼
┌─────────────────────────────────┐
│   SerialMessageRouter            │
│   - Line buffering               │
│   - Prefix matching              │
│   - Callback dispatch            │
└────────────┬────────────────────┘
             │ Parsed messages
             ▼
┌─────────────────────────────────┐
│   Application Callbacks          │
│   handleHITL(), handleRadio(),  │
│   handleCommand(), etc.          │
└──────────────────────────────────┘
```

## Usage

### Basic Example

```cpp
#include "Communication/SerialMessageRouter.h"

using namespace astra;

SerialMessageRouter router;

// Define callback functions
void handleHITL(const char* message, const char* prefix, Stream* source) {
    // message = "1.234,0.0,9.81,..." (prefix stripped)
    // prefix = "HITL/"
    // source = pointer to the Stream that received this message

    double simTime;
    if (HITLParser::parseAndInject(message, simTime)) {
        astraSys->update(simTime);
    }
}

void handleRadio(const char* message, const char* prefix, Stream* source) {
    LOGI("Radio: %s", message);
    // Process radio telemetry
}

void handleCommand(const char* message, const char* prefix, Stream* source) {
    if (strcmp(message, "REBOOT") == 0) {
        LOGW("Reboot command received");
        // Handle reboot
    } else if (strcmp(message, "STATUS") == 0) {
        // Send status
    }
}

void setup() {
    Serial.begin(115200);
    Serial8.begin(9600);  // Radio on Serial8

    // Configure router
    router
        .withInterface(&Serial)     // Monitor USB Serial
        .withInterface(&Serial8)    // Monitor Radio Serial
        .withListener("HITL/", handleHITL)
        .withListener("RAD/", handleRadio)
        .withListener("CMD/", handleCommand);
}

void loop() {
    router.update();  // Poll all interfaces, dispatch callbacks

    // Rest of your code...
}
```

### Advanced Example - Multi-Source Commands

```cpp
// Commands can come from any source!
void handleCommand(const char* message, const char* prefix, Stream* source) {
    if (strcmp(message, "STATUS") == 0) {
        RocketState state = avionics.getState();

        // Respond on the same interface the command came from
        source->printf("STATUS: Alt=%.2f Stage=%d\n",
                      state.altitude, state.flightStage);
    }
}

void setup() {
    router
        .withInterface(&Serial)   // USB
        .withInterface(&Serial8)  // Radio
        .withListener("CMD/", handleCommand);

    // Now "CMD/STATUS" works from USB or Radio!
}
```

### Default Handler for Unmatched Messages

```cpp
void handleUnknown(const char* message, const char* prefix, Stream* source) {
    LOGW("Unknown message: %s", message);
}

void setup() {
    router
        .withInterface(&Serial)
        .withListener("HITL/", handleHITL)
        .withListener("CMD/", handleCommand)
        .withDefaultHandler(handleUnknown);  // Catches everything else
}
```

### Custom Delimiter

```cpp
void setup() {
    router
        .withInterface(&Serial)
        .withListener("DATA/", handleData)
        .withDelimiter(';');  // Use semicolon instead of newline
}
```

### Configuration Options

```cpp
// Constructor parameters:
// SerialMessageRouter(maxInterfaces, maxPrefixes, bufferSize)
SerialMessageRouter router(8, 16, 512);  // 8 interfaces, 16 prefixes, 512-byte buffer

router
    .withInterface(&Serial)           // Register a serial interface
    .withListener("PREFIX/", callback) // Register a prefix listener
    .withDelimiter('\n')              // Set delimiter (default: '\n')
    .withDefaultHandler(callback);    // Set default handler
```

## Message Format

Messages are line-delimited strings with a prefix:

```
PREFIX/payload\n
```

Examples:
- `HITL/1.234,0.0,9.81,0.0,0.0,0.0\n`
- `CMD/REBOOT\n`
- `RAD/RSSI:-45dBm,SNR:12.5\n`
- `TELEM/1,123.45,BOOST\n`

The router:
1. Reads bytes from each registered interface
2. Buffers until delimiter is found
3. Matches against registered prefixes (in registration order)
4. Strips the prefix and calls the callback with the payload
5. If no prefix matches, calls default handler (if set)

## Callback Signature

```cpp
void callback(const char* message, const char* prefix, Stream* source);
```

Parameters:
- `message` - The message content *after* the prefix has been stripped
- `prefix` - The prefix that matched (e.g., "HITL/", "CMD/")
- `source` - Pointer to the Stream that received this message

## Important Notes

### Prefix Matching Priority
Prefixes are matched in registration order. First match wins.

```cpp
router
    .withListener("H", handleH)        // Matches first!
    .withListener("HITL/", handleHITL); // Never matches for "HITL/..." messages
```

### Buffer Management
- Each interface gets its own line buffer (default: 256 bytes)
- Messages longer than buffer are dropped
- Partial messages are buffered until delimiter arrives
- Buffers are automatically managed (no memory leaks)

### Thread Safety
Not thread-safe. Call `update()` from a single thread only.

### No Data Available
If no data is available, `update()` returns immediately without blocking.

### Memory Usage
- Fixed allocation at construction time
- Memory = `(maxInterfaces * bufferSize) + (maxPrefixes * sizeof(PrefixListener))`
- Default: `(4 * 256) + (8 * ~16) = ~1KB`

## Integration Examples

### HITL System

```cpp
router
    .withInterface(&Serial)
    .withListener("HITL/", [](const char* msg, const char* prefix, Stream* src) {
        double simTime;
        if (HITLParser::parseAndInject(msg, simTime)) {
            astraSys->update(simTime);
        }
    });
```

### Radio Telemetry

```cpp
router
    .withInterface(&Serial8)
    .withListener("RAD/", [](const char* msg, const char* prefix, Stream* src) {
        // Parse radio-specific telemetry
        RadioPacket pkt;
        if (parseRadioPacket(msg, &pkt)) {
            processRadioData(pkt);
        }
    });
```

### Ground Station Commands

```cpp
router
    .withInterface(&Serial)
    .withInterface(&Serial8)  // Commands can come from USB or Radio!
    .withListener("CMD/", [](const char* msg, const char* prefix, Stream* src) {
        executeCommand(msg);

        // Send ACK back on same interface
        src->printf("ACK/%s\n", msg);
    });
```

## Testing

Comprehensive unit tests are available in `test/test_serial_router/test_serial_router.cpp`.

Run tests:
```bash
pio test -e native -f test_serial_router
```

Tests cover:
- Interface and listener registration
- Message routing (single/multiple prefixes)
- Multi-interface support
- Default handler
- Custom delimiters
- Buffer overflow handling
- Partial messages
- Empty messages
- Null pointer safety
- Real-world scenarios (HITL)

All 21 tests pass.

## Performance

- **Non-blocking** - Returns immediately if no data available
- **Efficient** - Only scans registered prefixes for each message
- **Low overhead** - ~1KB memory for typical configuration
- **Fast** - Suitable for high-frequency message processing (tested at 50Hz+)

## Comparison to Existing SerialHandler

| Feature | SerialHandler | SerialMessageRouter |
|---------|---------------|---------------------|
| Purpose | HITL-specific binary protocol | General text-based routing |
| Interfaces | Single (hardcoded Serial) | Multiple, configurable |
| Protocol | Binary with markers `<>` | Text with prefixes |
| Routing | Fixed message type | Flexible prefix-based |
| Extensibility | Limited | High (builder pattern) |
| Use Case | HITL testing only | Any serial messaging |

Use `SerialMessageRouter` for general text-based serial routing. Use `SerialHandler` specifically for HITL binary protocol testing.

## Future Enhancements

Potential additions (not currently implemented):
- Message queuing for slow callbacks
- Timeout detection on interfaces
- Statistics (message counts, parse failures)
- Checksum validation
- Binary protocol support with length prefixes
- Per-listener buffer sizes
- Regular expression prefix matching

## License

Part of the Astra flight computer framework.
