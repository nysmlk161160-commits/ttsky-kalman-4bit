## Kalman 4-bit Digit-Serial Accelerator

This project implements the scalar Kalman measurement update using 10-bit Q4.6 fixed-point arithmetic. It is designed as a demonstrator of **area-versus-latency trade-offs**, rather than as a general-purpose Kalman filter with state prediction.

The update equations are:

- `K = P / (P + R)`
- `x_new = x + K * (z - x)`
- `P_new = P - K * P`

The scaled Kalman gain is `floor(64*P/(P+R))` for nonnegative P and R. The two products reuse one multiplier. The multiplier processes **four bits of its 12-bit upper partial-product accumulator per clock cycle** (three cycles per multiplier bit, seven multiplier bits, i.e. 21 arithmetic cycles per multiplication). The fractional-divider architecture calculates six quotient bits; its subtraction is also implemented using four-bit slices.

### External interface

`ui_in[7:0]`: byte being written. `uio_in[1:0]`: input register address (`0=z`, `1=x`, `2=P`, `3=R`). `uio_in[2]`: `0=low byte`, `1=upper two bits`. `uio_in[3]`: synchronous write strobe. `uio_in[4]`: synchronous start. `uio_in[5]`: select result (`0=x_new`, `1=P_new`). `uo_out[7:0]`: selected result byte. `uio_out[6]`: busy; `uio_out[7]`: ready. `uio_oe=0xC0`.

To write an input, drive the 8-bit data and input register address while asserting write for one rising edge. Write low byte and upper two bits separately. To calculate, pulse `start` for one rising edge and wait until `ready` becomes high. Set result selector and byte selector to read the signed `x_new` code and the unsigned `P_new` code. Decode Q4.6 by dividing the signed or unsigned integer code by 64.

Example inputs `z=96`, `x=0`, `P=64`, `R=320` correspond to `z=1.5`, `x=0`, `P=1`, `R=5`. Expected output codes are `x_new=15`, `P_new=54`.

### Limitations

Use nonnegative `P` and `R` with `P+R > 0`. Outputs are fixed-width and may wrap on overflow; there is no saturation or rounding-to-nearest. Input strobes are synchronous with the clock; external asynchronous hardware would require synchronization. This project uses fixed-point truncation, so results may differ from ideal floating-point Kalman updates. A successful HDL simulation does not imply that placement, timing, DRC or LVS are successful.

### Verification

`test/test.py` uses a Python integer arithmetic reference, tests sign, zero gain, gain of one and repeat transactions, and contains seeded randomized cases. Run the template's test workflow first, then build GDS. Examine PPA reports instead of assuming this architecture is smaller than the prior version.
