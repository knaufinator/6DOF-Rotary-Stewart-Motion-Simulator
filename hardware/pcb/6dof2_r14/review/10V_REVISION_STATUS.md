# R14 sensor-power revision status

**Ordered on 26 October 2026, confirmed by the owner. Testing is next.**
The current [R14 overview](../README.md), [design decisions](../DECISIONS.md),
[review method](../REVIEW_METHOD.md) and [order record](../ORDER_RECORD.json)
supersede earlier in-progress checkpoints.

The design retains nominal 12 V sensor power. TLV6700 supervision now falls at
11.046 V nominal, allowing a stated cable-loss budget above the sensor's 10 V
minimum. U41 directly clears common pulse permission on field-good or torque-latch
loss. U42/Q11 reduce input loss while D1 retains its startup path. Q11 is now
DMN6075S-7 / C155242; Y1 remains C9006.

The saved/reopened master has 324 parts and 1,214 logical pins. Native comparison
and DRC are empty; the [section report](2026-10-26-10V/README.md) records the
complete static-CAM, drill, aperture, label and STEP checks. The 15 ft cable's
≤2.0 Ω hot complete-loop requirement and the conditional voltage budget still
need actual measurements under [BRINGUP.md](../BRINGUP.md).
