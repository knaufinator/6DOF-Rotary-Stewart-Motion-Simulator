# R13 single-axis bench evidence and tools

R13 was manufactured and tested on axis 0. The laptop's exact tested source delta
is preserved in [the recovered-source evidence](../../6dof2_r14/review/r13-recovered-source/README.md).
The [firmware status](../../6dof2_r14/FIRMWARE_STATUS.md) separates the retained final
verification from earlier soaks and subsequent R14 changes.

`r13bench_core.py` is the native-USB command client; `r13bench_app.py` provides its
bench interface. Use `python r13bench_core.py --help` to inspect options without
opening a port. The client asserts DTR and deasserts RTS; it does not use UART
reset toggling. Bench commands require the separate `R13_BENCH_ONE_AXIS` image.
Do not transfer old adapters or testpoint assumptions to the corrected R14 DB25s.

The verified configuration is electronic gear 2:1, 2,500 encoder counts/revolution,
50:1 gearbox, and ±72° / 50,000-pulse range. Counts and drive parameters must agree
before motion. A one-axis test does not establish six-axis or loaded-platform behavior.

```text
python -m unittest hardware/pcb/6dof2_r13/bringup/test_r13bench_core.py
```

Run the host-only test command from the repository root. It opens no serial port.
For new R14 hardware use the [R14 qualification plan](../../6dof2_r14/BRINGUP.md).
