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
AstraConfig config = AstraConfig()
    .withHITL()
    .withHITLInterface(&Serial);

Astra sys(&config);

void setup() {
    Serial.begin(115200);
    sys.init();
}

void loop() {
    sys.update();
}
```

`withHITL()` installs Astra-owned HITL sensor defaults. `withHITLInterface()`
selects the stream carrying the packets. Astra creates the message router,
parses `HITL/` packets, and performs the simulation-time update internally; do
not register a second `HITL/` listener or call the router separately.

