# HITL (Hardware‑In‑The‑Loop)

HITL lets you run Astra against simulated sensor data over a serial link.

---

## Data Flow

```
Simulator → HITL/ lines → HITLParser → HITLSensorBuffer → HITL Sensors → Astra
```

---

## Message Format

```
HITL/timestamp,ax,ay,az,gx,gy,gz,mx,my,mz,pressure,temp,lat,lon,alt,fix,fixqual,heading
```

Units:

- Accel: m/s²
- Gyro: rad/s
- Mag: µT
- Pressure: hPa
- Temp: °C
- GPS lat/lon: degrees
- GPS alt: meters

---

## Configure Astra

```cpp
AstraConfig config;

Astra sys(&config);

void setup() {
    Serial.begin(115200);
    config.withHITL()
        .withHITLInterface(&Serial);
    if (sys.init() == 0) {
        // Enable the HITL READY response used by Astra-Support.
        sys.update(0.0);
    }
}

void loop() {
    sys.update();
}
```

`withHITL()` installs Astra-owned HITL sensor defaults. `withHITLInterface()`
selects the stream carrying the packets. Astra creates the message router,
parses `HITL/` packets, and performs the simulation-time update internally; do
not register a second `HITL/` listener or call the router separately.

Configure a named `AstraConfig` object directly. HITL configurations own their
sensor objects; copying a configured temporary leaves dangling sensor pointers
when the temporary is destroyed. For the Astra-Support startup handshake, call
`sys.update(0.0)` once after successful initialization, then use `sys.update()`
in the loop. The complete [HITLWithRouter example](https://github.com/Terrapin-Rocket-Team/Astra/blob/main/examples/HITLWithRouter/main.cpp)
also configures the serial telemetry and event-log sinks.

