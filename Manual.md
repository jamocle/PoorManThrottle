# DEAD-RAIL REMOTE THROTTLE  
## ESP32 + IBT-2 (BTS7960) Motor Control System  
### Final Build & Wiring Manual  
### Revision 1.3 (Professional Edition)

---

# 1. System Overview

## Design Philosophy

This system is engineered for:

- Electrical robustness  
- Noise immunity in a motor environment  
- Safe incremental testing  
- Clear separation of logic and power systems  

High-current motor power and low-current logic power are intentionally separated to improve stability and prevent resets or interference.

---

## System Ratings

- **System Voltage:** 15.3V nominal  
- **Logic Voltage:** 5V  
- **Max Motor Current (design target):** ~6A stall  
- **Recommended Fuse:** 7.5A  
- **Maximum Temporary Test Fuse:** 10A  

Motor stall current may momentarily exceed 6A during startup. Short surge events are normal for brushed DC motors.

---

## System Components

- ESP32 Development Board (USB-C)  
- IBT-2 / BTS7960 High Current Motor Driver  
- 15.3V DC Main Power Bus (black box source)  
- 15V → 5V Buck Converter (USB-C output)  
- Brushed DC Motor (up to ~6A stall)  

---

## System Architecture

```text
15.3V Bus → Fuse → IBT-2 (Motor Power)
15.3V Bus → Buck Converter → ESP32 (Logic Power)
ESP32 → PWM → IBT-2 → Motor
```

---

# 2. Safety Notes

## Fuse Guidance

- Recommended starting fuse: 7.5A  
- 10A acceptable for testing if nuisance blowing occurs  
- Do not exceed 10A without re-evaluating wiring and driver ratings  

## Always

- Power OFF before modifying wiring  
- Verify polarity before applying power  
- Keep high-current wires short and direct  
- Twist motor wires together  
- Ensure common ground is connected  

## Never

- Connect IBT-2 VCC to 15V  
- Use ESP32 3V3 for IBT-2 logic power  
- Power ESP32 from VIN and 5V simultaneously from different sources  
- Allow M+ and M- to touch while powered (direct short across H-bridge)  

---

# 3. Wire Gauge Recommendations

| Connection | Recommended Wire |
|------------|------------------|
| 15V Bus to Fuse | 16 AWG |
| Fuse to IBT-2 B+ | 16 AWG |
| IBT-2 to Motor | 16 AWG |
| Buck Input | 18–20 AWG |
| ESP32 Signals | 22–24 AWG |

---

# 4. Stage 1 — Main Power Backbone

## Wiring

```text
15V BUS + → Fuse Holder → FUSED +15V
15V BUS - → MAIN GND
```

## Test (Power OFF)

- Continuity: FUSED +15V to MAIN GND → Should NOT beep  
- Continuity: 15V BUS + to FUSED +15V → Should beep  

---

# 5. Stage 2 — IBT-2 Power Wiring (Motor Disconnected)

| IBT-2 Terminal | Connect To |
|---------------|------------|
| B+ | FUSED +15V |
| B- | MAIN GND |
| M+ | Leave empty |
| M- | Leave empty |

## Test (Power ON)

Measure B+ to B- → ~15.3V

---

# 6. Stage 3 — Buck Converter Installation

```text
Buck Input Red → FUSED +15V
Buck Input Black → MAIN GND
USB-C Output → ESP32 USB-C Port
```

## Test

- Buck input voltage → ~15V  
- ESP32 powers on  
- ESP32 5V to GND → ~5.0V  
- When USB-powered, VIN often reads ~5V (board dependent — verify with meter)  

---

# 7. Stage 4 — IBT-2 Logic Power and Ground

## Common Ground (Mandatory)

```text
ESP32 GND → IBT-2 GND (header)
```

## Provide IBT-2 Logic Supply

```text
ESP32 5V → IBT-2 VCC
```

### Logic-Level Note

- IBT-2 control inputs are 5V tolerant  
- ESP32 GPIO outputs are 3.3V logic HIGH  
- 3.3V HIGH is sufficient to drive RPWM and LPWM correctly  

## Enable Pins

```text
R_EN → VCC
L_EN → VCC
```

## Additional Decoupling (Recommended)

Add a 100nF ceramic capacitor directly between IBT-2 VCC and GND at the header pins.

---

# 8. Stage 5 — Control Signal Wiring

| ESP32 Pin | IBT-2 Pin |
|------------|------------|
| GPIO25 | RPWM |
| GPIO26 | LPWM |

Leave R_IS and L_IS unconnected.

---

# 9. Stage 6 — Capacitor Installation

## Motor Supply Bulk Capacitor

470µF (25V or 50V) across IBT-2 B+ and B-  
Stripe → B-

## 5V Stability Capacitor

220µF (10V+) across ESP32 5V and GND  
Stripe → GND  

## Optional EMI Reduction

- 100nF ceramic capacitor across motor terminals  
- Ferrite clip around BOTH motor wires  
- Twist motor wires together  

---

# 10. Stage 7 — First Power-Up (No Motor)

| Measurement | Expected |
|-------------|----------|
| IBT-2 B+ to B- | ~15V |
| IBT-2 VCC to GND | ~5V |
| ESP32 stable | No resets |

Power OFF before connecting motor.

---

# 11. Stage 8 — Connect Motor

| IBT-2 | Motor |
|--------|--------|
| M+ | Motor lead 1 |
| M- | Motor lead 2 |

Twist M+ and M- wires together.

## Wire Routing Guidance

- Keep motor wires physically separated from ESP32 signal wires  
- Avoid running PWM signal wires parallel to motor leads  

## Pre-Motion Check

M+ to M- at idle → Small residual voltage (<0.5V) is normal.  

If ~15V appears, disconnect and inspect wiring immediately.

---

# 12. Initial Motion Test

Start with small PWM value.

## PWM Configuration

- Recommended frequency: 15–20 kHz  
- Use hardware PWM (LEDC on ESP32)  
- Do NOT use delay-based software PWM  

Motor should move gently.

Monitor fuse, wiring temperature, and ESP32 stability.

Firmware should enforce a brief zero-PWM interval before reversing direction to reduce stress on the motor and H-bridge.

---

# 13. Thermal Guidance

IBT-2 may become warm under load.

- Warm is normal  
- If too hot to comfortably touch (~60°C), improve airflow  
- For sustained loads above 5A, provide ventilation or active airflow  
- Do not fully enclose heatsink without ventilation openings  

---

# 14. Ground Reference Diagram

```text
15V BUS -
├── IBT-2 B-
├── Buck IN-
└── ESP32 GND
```

Common ground is mandatory for proper PWM operation.

---

# 15. Troubleshooting Order

1. 15V present?  
2. 5V present?  
3. Grounds common?  
4. IBT-2 VCC = 5V?  
5. Signals wired correctly?  
6. Then motor  

---

# 16. Wiring Diagram (Box & Line Illustration)

## 16.1 System Wiring Diagram  
### Complete Electrical Layout

```text
                         ┌────────────────────────────┐
                         │        15.3V BUS           │
                         │   (Battery + Buck System)  │
                         └─────────────┬──────────────┘
                                       │
                                       │
                         ┌─────────────▼──────────────┐
                         │        7.5A FUSE           │
                         └─────────────┬──────────────┘
                                       │
                           FUSED +15V  │
                                       │
                           ┌───────────┴─────────────────────┐
                 │         │                                 │
                 │         │                                 │
        ┌────────▼────────┐│                      ┌──────────▼─────────┐
        │      IBT-2      ││                      │  15V → 5V BUCK     │
        │  BTS7960 Driver ││                      │  Converter Module  │
        │                 ││                      │                    │
        │   B+  ◄──────────┘                      │  IN+ ◄─────────────┘
        │   B-  ◄──────────── MAIN GND ───────────┤  IN- ◄─────────────┐
        │                 │                       │                    │
        │   M+ ──┐        │                       │   USB-C OUTPUT     │
        │   M- ──┐        │                       └──────────┬─────────┘
        └────────┬────────┘                                 │
                 │                                           │
                 │                                           │
           ┌─────▼─────┐                              ┌──────▼──────┐
           │   MOTOR   │                              │    ESP32    │
           │  (Brushed)│                              │  Dev Board  │
           └───────────┘                              │             │
                                                      │ USB-C Power │
                                                      │             │
                                                      │  5V ────────┼─────┐
                                                      │  GND ───────┼─────┼───┐
                                                      │  GPIO25 ────┼─────┼──┐│
                                                      │  GPIO26 ────┼─────┼─┐││
                                                      └─────────────┘     │ │││
                                                                          │ │││
                                                                          │ │││
                                                     ┌────────────────────▼─▼▼▼────────┐
                                                     │          IBT-2 HEADER           │
                                                     │                                 │
                                                     │  VCC ◄──────── ESP32 5V         │
                                                     │  GND ◄──────── ESP32 GND        │
                                                     │  RPWM ◄─────── GPIO25           │
                                                     │  LPWM ◄─────── GPIO26           │
                                                     │  R_EN ────────┐                 │
                                                     │  L_EN ────────┘ → tied to VCC   │
                                                     │  R_IS (unused)                  │
                                                     │  L_IS (unused)                  │
                                                     └─────────────────────────────────┘
```
## 16.2 Capacitor Placement Diagram
### Bulk Capacitors for Stability
```text
             +15V Rail
                │
                │
           ┌────▼────┐
           │ 470µF   │
           │ (25V+)  │
           │         │
           └────┬────┘
                │
               GND
Stripe on capacitor → GND
```
```text
             5V Rail (Near ESP32)
                │
                │
           ┌────▼────┐
           │ 220µF   │
           │ (10V+)  │
           │         │
           └────┬────┘
                │
               GND
Stripe on capacitor → GND
```
## 16.3 Wire Gauge Guidance

| Connection | Recommended Wire |
|------------|------------------|
| 15V BUS to Fuse | 16 AWG |
| Fuse to IBT-2 B+ | 16 AWG |
| IBT-2 to Motor | 16 AWG |
| Buck Input | 18–20 AWG |
| ESP32 Signals | 22–24 AWG |


## 16.4 Ground Reference Diagram
All grounds connect together:
```text
15V BUS - 
      │
      ├── IBT-2 B-
      ├── Buck IN-
      └── ESP32 GND
```
Common ground is mandatory.

---

**END OF MANUAL — REVISION 1.3**
