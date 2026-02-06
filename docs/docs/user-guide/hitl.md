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

## Router Integration (Recommended)

```cpp
auto* router = sys.getMessageRouter();
router->withListener("HITL/", [](const char* msg, const char* prefix, Stream* src) {
    double simTime;
    if (HITLParser::parse(msg, simTime)) {
        sys.update(simTime);
    }
});
```

Because Astra already updates its router, you do not need to call `router->update()` manually.

