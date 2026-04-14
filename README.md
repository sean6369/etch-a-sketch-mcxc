# Etch-a-Sketch MCXC

Firmware for the MCXC controller in the Etch-a-Sketch project.

## Shake Erase

Shake erase is detected from MPU accelerometer motion energy.

- The MCU reads full 16-bit X/Y/Z accelerometer samples from `ACCEL_XOUT_H`.
- Per-sample motion is computed as `abs(dx) + abs(dy) + abs(dz)`.
- A shake event requires a 5-sample window with:
  - at least 2 hot samples above `SHAKE_SAMPLE_DELTA_THRESHOLD`
  - total motion energy above `SHAKE_ENERGY_THRESHOLD`
- The confirmed working energy threshold is `40000`.
- Erase is only asserted during `ROUND_PHASE_FREE_DRAW`.
- A post-trigger lockout prevents one shake from creating repeated erase events.

Compact debug telemetry is printed as:

```text
[SHAKE] a=..., d=..., s=..., e=..., hot=..., hit=..., err=..., rec=...
```

Fields:

- `a`: raw accelerometer X,Y,Z sample
- `d`: per-axis delta from the previous sample
- `s`: instant motion score
- `e`: rolling window energy
- `hot`: number of hot samples in the current window
- `hit`: shake event emitted this sample
- `err`: I2C error code (`0` none, `2` timeout, `3` NACK)
- `rec`: MPU/I2C recovery count

## Debugging With MPU Wiring

If SDA or SCL is disconnected while the firmware is running, do not rely on
debug resume. Resume continues from the halted instruction and may leave the
I2C peripheral, firmware state, and MPU in the same bad transaction state.

Use a clean restart instead:

1. Clean up debug
2. Debug
3. Resume all debug sessions

At rest, the accelerometer should not normally report `a=+0,+0,+0`. One axis
should show gravity. Repeated zero samples usually mean the MPU or I2C bus did
not restart cleanly.
