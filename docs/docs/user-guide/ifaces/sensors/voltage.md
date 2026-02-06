# VoltageSensor

`VoltageSensor` reads a voltage via ADC and (optionally) a resistor divider.

---

## Constructors

```cpp
VoltageSensor(int pin, const char* name = "Voltage Sensor");
VoltageSensor(int pin, int r1, int r2,
              const char* name = "Voltage Sensor",
              double refVoltage = PLATFORM_DEFAULT_REF_VOLTAGE);
```

- `r1` and `r2` are divider resistances (ohms)
- `refVoltage` is the ADC reference voltage

---

## Usage

```cpp
VoltageSensor vbat(A0, 10000, 2000, "Battery");

double v = vbat.getVoltage();
int raw = vbat.getRawValue();
```

---

## Notes

- ADC resolution is platform-specific
- The default reference voltage is chosen by build flags (`ENV_TEENSY`, `ENV_STM`, `ENV_ESP`)

