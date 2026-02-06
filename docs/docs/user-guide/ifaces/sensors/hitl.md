# HITL Sensors

HITL sensors read simulated data from `HITLSensorBuffer` instead of hardware.

They are drop‑in replacements for hardware sensors and integrate with Astra normally.

---

## Available HITL Sensors

- `HITLBarometer`
- `HITLAccel`
- `HITLGyro`
- `HITLMag`
- `HITLGPS`

---

## Message Format

HITL data is provided via `HITL/` messages:

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
- Fix: 0/1
- FixQual: satellites

---

## Parser + Router

Use `SerialMessageRouter` to handle incoming HITL lines:

```cpp
router.withInterface(&Serial)
      .withListener("HITL/", [](const char* msg, const char* prefix, Stream* src) {
          double simTime;
          if (HITLParser::parse(msg, simTime)) {
              sys.update(simTime);
          }
      });
```

If you use the full Astra system, register the listener on
`sys.getMessageRouter()` and **skip calling** `router.update()` yourself.

---

## Recommended Reading

See the full [HITL Guide](../../hitl.md).
