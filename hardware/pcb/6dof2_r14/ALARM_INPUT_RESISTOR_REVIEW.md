# R14 drive-alarm optocoupler LED resistor review

> Supporting design study. For the ordered 26 October revision, use
> [current decisions](DECISIONS.md), [review method](REVIEW_METHOD.md) and
> [hardware-test plan](BRINGUP.md). Dated proposals and open-status statements
> below retain their original context.

**2026-10-12 status correction:** The 2026-10-10 note immediately below
describes an earlier schematic-only checkpoint. Later R14 PCB synchronization
replaced and placed all six R30–R35 footprints with the 1.5 kΩ/1206 parts;
see `R14_PLAN.md` and its saved source checkpoints. The R13 220 Ω heating
figures in this review are calculated, not measured on the first article.
Actual supply range, drive alarm-output polarity/current, optocoupler logic
levels and resistor temperature remain untested. The R13 bringup log still
holds drive-alarm testing, and the current R14 board still needs final alarm
copper, BOM and diagnostic-access checks.

**2026-10-10 schematic implementation:** R30-R35 are now the exact YAGEO
`AC1206FR-071K5L` / JLC `C229395` 1.5 kΩ, 1206 parts in the saved R14
schematic. A save/close/reopen export and netlist comparison confirmed all six
`VINRAW` to `M*_ALARM_LED` pin pairs, all other 139 component records and the
named nets unchanged. The R14 PCB still has the inherited R0603 footprints;
the schematic has **not** been synchronized to PCB. Supply and alarm-output
measurements, optocoupler low-level and thermal tests, and assembly allocation
remain release gates. See `R14_ALARM_RESISTOR_SCHEMATIC_CHECKPOINT.md`.

**2026-10-10 correction, before CAD change:** R13 R30–R35 are six `220 Ω`
`0603WAF2201T5E` / LCSC `C4190` 0603 resistors from `VINRAW` to the six
`M*_ALARM_LED` nets. The original R14 draft suggested changing them to `1 kΩ`
`0603WAF1001T5E` / `C21190` in the same footprint. That value would reduce
LED current, but **would still exceed the 0603 resistor's 0.1 W rating** at
normal supply voltage. Do not implement that earlier draft instruction.

The [UNI-ROYAL thick-film datasheet](https://www.uni-royal.cn/en/images/userfile/file/1753752986c56505e6d9ab55c7.pdf)
identifies `WA` as 1/10 W, `W4` as 1/4 W, and ordinary 0603/1206 packages as
0.1/0.25 W respectively. The [LCSC C21190 listing](https://www.lcsc.com/product-detail/Chip-Resistor-Surface-Mount_Uniroyal-Elec-0603WAF1001T5E_C21190.html)
confirms the proposed 1 kΩ 0603 part is 100 mW. The
[Lite-On LTV-8X7 datasheet](https://optoelectronics.liteon.com/upload/download/DS-70-96-0016/LTV-8X7%20series%20%20Rev.S.PDF)
specifies a 50 mA maximum LED forward current and minimum 50% current
transfer ratio at 5 mA input current, 5 V collector-emitter. The R13 alarm
output load is a 10 kΩ pull-up to 3.3 V, about 0.33 mA before input leakage.

Using 1.2 V as an **estimate** of opto LED forward voltage and ignoring the
drive output's low-state drop gives the conservative resistor calculations:

| R and package | 12.0 V: LED current / resistor heat | 13.8 V: LED current / resistor heat | Status |
|---|---:|---:|---|
| R13 220 Ω, 0603 0.1 W | 49 mA / 0.530 W | 57 mA / 0.722 W | Fails resistor rating badly; at 13.8 V also exceeds 50 mA LED rating |
| Old R14 suggestion 1 kΩ, 0603 0.1 W | 10.8 mA / 0.117 W | 12.6 mA / 0.159 W | LED current reasonable, resistor still overrated |
| **1.8 kΩ, 1206 0.25 W trial** | 6.0 mA / 0.065 W | 7.0 mA / 0.088 W | Plausible electrical starting point; still needs actual VINRAW envelope, drive output, thermal and logic bench proof |
| **2.0 kΩ, 1206 0.25 W trial** | 5.4 mA / 0.058 W | 6.3 mA / 0.079 W | Lower current and heat; at 10 V only 4.4 mA, below the optocoupler's 5 mA CTR test condition |

At a hypothetical 10 V supply, the 1.8 kΩ trial gives about 4.9 mA, near the
datasheet's 5 mA CTR test point. The optocoupler current needed to pull a
10 kΩ/3.3 V input low appears much smaller than the guaranteed CTR current at
5 mA, but exact thresholds, drive alarm polarity, series output impedance and
temperature require measurement. The 1206 trial has substantial power margin
for the **assumed 12–13.8 V example only**; verify the actual maximum VINRAW
and derating at local temperature. It is not a 24 V-tolerant design by default.

Initial sourcing candidates are [UNI-ROYAL 1206W4F1801T5E / C22108](https://www.lcsc.com/product-image/C22108.html)
and [RESI AECR1206F1K80K9 / C352131](https://jlcpcb.com/partdetail/Resi-AECR1206F1K80K9/C352131),
both 1.8 kΩ, 1206, 0.25 W. The UNI-ROYAL listing showed zero stock at the
check. JLC listed C352131 as an assembly candidate, but current allocation
and its manufacturer's drawing must be rechecked before selection. A stronger
assembly-sourcing candidate is [UNI-ROYAL 1206W4F2001T5E / C17944](https://jlcpcb.com/partdetail/C17944):
JLC lists this 2.0 kΩ, 1206, 0.25 W part as **Basic**. Its lower current is
acceptable only after the minimum actual supply, drive-output voltage drop,
opto logic-low threshold, temperature and fault polarity are measured. The
5 mA datasheet CTR condition alone does not guarantee saturation of this
specific 3.3 V output circuit at the minimum supply. None of these earlier
1.8 kΩ or 2.0 kΩ candidates was selected in the R14 schematic.

**2026-10-10 stronger low-voltage candidate:** [YAGEO
`AC1206FR-071K5L` / JLC `C229395`](https://jlcpcb.com/partdetail/YAGEO-AC1206FR071K5L/C229395)
is an exact 1.5 kΩ ±1%, 1206, 0.25 W Extended assembly part. The R14
EasyEDA library returned its exact device once (`b867dd1c468b4a4299800a11ded9db30`).
At a **provisional 10–14 V VINRAW**, use 1.485 kΩ minimum for the upper
current/heat bound: even treating the optocoupler LED and drive output as
zero volts, 14 V gives at most **9.43 mA** and **0.132 W**. At 10 V with an
assumed 1.4 V LED and 0.5 V sinking-output drop, 1.515 kΩ gives **5.35 mA**.
The 14 V resistor heat is 53% of its nominal 0.25 W rating; the board's
actual ambient/thermal derating still needs checking. Lite-On specifies the
opto's 50% minimum CTR at 5 mA and 5 V collector-emitter, so this arithmetic
does not guarantee a saturated 3.3 V logic low. Drive alarm output sink
capacity, polarity, VINRAW bounds, JLC available-order quantity, exact pads
and the bench low level remain release gates. The saved 1.5 kΩ schematic trial
has more input current than the 2.0 kΩ part when minimum voltage may be near 10 V;
it does not establish compatibility with 24 V VINRAW.

Before applying the alarm inputs, measure the owner's actual supply range and
one drive alarm output at clear/fault/disconnected states. Confirm R30-R35
dissipation at the maximum supply and the optocoupler output threshold at the
minimum supply. Synchronize all six R1206 footprints to R14 PCB, then inspect
numbered pads, placement and routing. Preserve the imported checkpoint for
rollback. The inherited R13 PCB 220 Ω path is unsuitable for direct use at
the assumed 12 V alarm-on condition.
