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

## Astra Integration

Enable the simulated sensor set and select its input stream through
`AstraConfig`:

```cpp
AstraConfig config = AstraConfig()
    .withHITL()
    .withHITLInterface(&Serial);
```

Astra owns the `HITL/` listener and parser when using the full system. The
application loop only calls `sys.update()`. Use `HITLParser` and
`HITLSensorBuffer` directly only when building a lower-level integration without
`Astra`.

---

## Recommended Reading

See the full [HITL Guide](../../hitl.md).
