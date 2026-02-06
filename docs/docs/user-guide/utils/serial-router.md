# SerialMessageRouter

`SerialMessageRouter` centralizes serial parsing and routes messages by prefix.

---

## Why Use It?

- Multiple serial interfaces
- Prefix‑based dispatch (`HITL/`, `CMD/`, `RAD/`)
- Non‑blocking, line‑buffered operation

---

## Basic Usage

```cpp
#include <Communication/SerialMessageRouter.h>

using namespace astra;

SerialMessageRouter router;

void handleCmd(const char* msg, const char* prefix, Stream* src) {
    if (strcmp(msg, "PING") == 0) {
        src->println("PONG");
    }
}

void setup() {
    Serial.begin(115200);

    router.withInterface(&Serial)
          .withListener("CMD/", handleCmd);
}

void loop() {
    router.update();  // Non-blocking poll
}
```

---

## Callback Signature

```cpp
void callback(const char* message,
              const char* prefix,
              Stream* source);
```

- `message` excludes the prefix
- `source` is the serial interface that received the message

---

## Configuration Options

```cpp
SerialMessageRouter router(4, 8, 256);

// 4  = max serial interfaces
// 8  = max prefix listeners
// 256 = per-interface line buffer size (bytes)

router.withInterface(&Serial)
      .withListener("HITL/", handleHITL)
      .withDelimiter('\n')
      .withDefaultHandler(handleUnknown);
```

---

## Astra Integration

`Astra` creates its own router and listens for:

```
CMD/HEADER
```

Use `Astra::getMessageRouter()` to add more listeners.

If you use the full Astra system, you **do not** need to call `router.update()` yourself.
Astra calls it internally each update cycle.

---

## What the Basic Example Does

- Registers `Serial` as an input interface
- Matches messages that start with `CMD/`
- Strips the prefix and passes the remainder to `handleCmd`
- Responds on the same interface (useful for USB vs radio)
