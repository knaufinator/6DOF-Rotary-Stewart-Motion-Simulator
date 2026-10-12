#!/usr/bin/env python
"""R13 bench test app (Tkinter) for the probe firmware and the controller bench image.

    <IDF venv>\\Scripts\\python.exe r13bench_app.py [--port COM9] [--operator NAME]

Tabs:
  Bring-up  guided steps from M1_OUTPUT_TEST_PLAN.md + R13_FULL_BRINGUP_PLAN.md: purpose, setup,
            one-click actions, meter entries checked against expected ranges, PASS/FAIL/SKIP to the log
  Motion    axis, inhibit, arm, DIR, bounded pulse / tone (caps follow the "Drive connected" switch)
  Console   any firmware command

STOP (button or Esc) sends `safe` (probe) or `BENCH:STOP` (controller). The J2 wire stays IN to run;
pulling it OUT is the hardware stop (re-arm needed after refitting). The probe runs one command at
a time, so during a probe tone/pulse STOP is only processed when that command ends: the immediate
stop is the J2 link. Every TX/RX and step result goes to logs/r13_session_*.md and .jsonl.
"""
from __future__ import annotations

import argparse
import queue
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

import r13bench_core as core

POLL_MS = 1500


class Worker(threading.Thread):
    """Serial jobs run here, one at a time; results come back through a queue."""

    def __init__(self, app):
        super().__init__(daemon=True)
        self.app = app
        self.jobs: "queue.Queue" = queue.Queue()
        self.busy = False

    def run(self):
        while True:
            fn, args = self.jobs.get()
            self.busy = True
            try:
                fn(*args)
            except Exception as e:  # report, keep the worker alive
                self.app.ui(self.app.log_line, f"[error] {e}")
            finally:
                self.busy = False

    def submit(self, fn, *args):
        self.jobs.put((fn, args))


class App:
    def __init__(self, root: tk.Tk, port: str = "", operator: str = ""):
        self.root = root
        self.link: core.Link | None = None
        self.log = core.SessionLog(operator=operator)
        self.uiq: "queue.Queue" = queue.Queue()
        self.worker = Worker(self); self.worker.start()
        self.drive_connected = tk.BooleanVar(value=True)
        self.live = tk.BooleanVar(value=True)
        self.port_var = tk.StringVar(value=port)
        self.fw_var = tk.StringVar(value="not connected")
        self.values: dict[str, dict[str, tk.StringVar]] = {}
        root.title("R13 bench — probe / controller bench mode")
        root.geometry("1180x780")
        self._build()
        root.bind("<Escape>", lambda e: self.stop())
        root.after(50, self._drain_ui)
        root.after(POLL_MS, self._poll)
        self.refresh_ports()
        self.log_line(f"session log: {self.log.md_path}")

    # ── thread-safe UI ──────────────────────────────────────────────────────────────────
    def ui(self, fn, *args):
        self.uiq.put((fn, args))

    def _drain_ui(self):
        while True:
            try:
                fn, args = self.uiq.get_nowait()
            except queue.Empty:
                break
            try:
                fn(*args)
            except Exception as e:  # one bad callback must not stop UI updates
                try:
                    self.log_line(f"[ui error] {e}")
                except Exception:
                    pass
        self.root.after(50, self._drain_ui)

    # ── layout ──────────────────────────────────────────────────────────────────────────
    def _build(self):
        top = ttk.Frame(self.root, padding=6); top.pack(fill="x")
        ttk.Label(top, text="Port").pack(side="left")
        self.port_cb = ttk.Combobox(top, textvariable=self.port_var, width=34); self.port_cb.pack(side="left", padx=4)
        ttk.Button(top, text="↻", width=3, command=self.refresh_ports).pack(side="left")
        ttk.Button(top, text="Connect", command=self.connect).pack(side="left", padx=4)
        ttk.Button(top, text="Disconnect", command=self.disconnect).pack(side="left")
        ttk.Label(top, textvariable=self.fw_var, foreground="#0a5").pack(side="left", padx=10)
        tk.Button(top, text="STOP  (Esc)", bg="#c00", fg="white", activebackground="#900",
                  font=("Segoe UI", 12, "bold"), width=14, command=self.stop).pack(side="right")
        ttk.Checkbutton(top, text="Drive connected (bench caps)", variable=self.drive_connected,
                        command=self._caps_changed).pack(side="right", padx=8)
        ttk.Checkbutton(top, text="Live status", variable=self.live).pack(side="right")

        body = ttk.PanedWindow(self.root, orient="horizontal"); body.pack(fill="both", expand=True)
        left = ttk.Frame(body, padding=6, width=250); body.add(left, weight=0)
        self._build_status(left)
        nb = ttk.Notebook(body); body.add(nb, weight=1)
        self._build_steps(nb); self._build_motion(nb); self._build_console(nb)

        logf = ttk.Frame(self.root, padding=(6, 0, 6, 6)); logf.pack(fill="both")
        self.logbox = tk.Text(logf, height=11, font=("Consolas", 9)); self.logbox.pack(fill="both", expand=True)

    def _build_status(self, f):
        ttk.Label(f, text="Board status", font=("Segoe UI", 11, "bold")).pack(anchor="w")
        self.st = {}
        for k, lbl in [("loop", "Stop loop (J2)"), ("inhibit", "Inhibit GPIO47"), ("arm", "Arm GPIO48"),
                       ("armed", "Bench armed"), ("alarms", "Alarms 0-5"), ("step", "STEP readback"),
                       ("dir", "DIR readback"), ("pos", "Axis-0 position")]:
            row = ttk.Frame(f); row.pack(fill="x", pady=1)
            ttk.Label(row, text=lbl, width=16).pack(side="left")
            v = tk.StringVar(value="—"); self.st[k] = v
            ttk.Label(row, textvariable=v, font=("Consolas", 10)).pack(side="left")
        ttk.Separator(f).pack(fill="x", pady=8)
        ttk.Label(f, text="DRIVE_ENABLE has no MCU readback:\nmeter TP8 (U3 pin 4).",
                  foreground="#a60").pack(anchor="w")
        ttk.Label(f, text="Axis N = DB25 J(3+N), 'MOTOR N'.\nBench motor 1 = axis 0 = J3.",
                  foreground="#555").pack(anchor="w", pady=6)
        ttk.Label(f, text="J2 wire IN = run (loop closed).\nHardware STOP: pull the J2 wire OUT.\n"
                          "Refitting it does not restart:\nre-arm is required.",
                  foreground="#c00", justify="left").pack(anchor="w")

    def _build_steps(self, nb):
        f = ttk.Frame(nb, padding=6); nb.add(f, text="Bring-up")
        self.step_list = tk.Listbox(f, width=34, exportselection=False, font=("Segoe UI", 10))
        self.step_list.pack(side="left", fill="y")
        for s in core.STEPS:
            self.step_list.insert("end", f"{s.sid:>3}  {s.title}")
        self.step_list.bind("<<ListboxSelect>>", lambda e: self.show_step())
        self.step_pane = ttk.Frame(f, padding=(10, 0)); self.step_pane.pack(side="left", fill="both", expand=True)
        self.step_list.selection_set(0); self.show_step()

    def show_step(self):
        sel = self.step_list.curselection()
        if not sel:
            return
        s = core.STEPS[sel[0]]
        for w in self.step_pane.winfo_children():
            w.destroy()
        p = self.step_pane
        ttk.Label(p, text=f"Step {s.sid} — {s.title}", font=("Segoe UI", 12, "bold")).pack(anchor="w")
        ttk.Label(p, text=f"Ref: {s.ref}", foreground="#555").pack(anchor="w")
        for head, txt in [("Purpose", s.purpose), ("Setup", s.setup), ("Pass", s.passes)]:
            ttk.Label(p, text=f"{head}: {txt}", wraplength=760, justify="left").pack(anchor="w", pady=2)
        af = ttk.LabelFrame(p, text="Actions", padding=6); af.pack(fill="x", pady=4)
        for i, act in enumerate(s.actions):
            b = ttk.Button(af, text=act.label, command=lambda a=act: self.run_action(a))
            b.grid(row=i // 3, column=i % 3, sticky="ew", padx=3, pady=2)
        mf = ttk.LabelFrame(p, text="Measurements", padding=6); mf.pack(fill="x", pady=4)
        vals = self.values.setdefault(s.sid, {})
        for r, m in enumerate(s.measurements):
            exp = m.expect if m.unit == "Y/N" else f"{'' if m.lo is None else m.lo} – {'' if m.hi is None else m.hi} {m.unit}"
            ttk.Label(mf, text=m.label, width=38).grid(row=r, column=0, sticky="w")
            ttk.Label(mf, text=m.point, width=18, foreground="#555").grid(row=r, column=1, sticky="w")
            v = vals.setdefault(m.key, tk.StringVar())
            e = ttk.Entry(mf, textvariable=v, width=10); e.grid(row=r, column=2)
            ttk.Label(mf, text=exp, width=16).grid(row=r, column=3, sticky="w")
            res = ttk.Label(mf, text="", width=6); res.grid(row=r, column=4)
            v.trace_add("write", lambda *_a, m=m, v=v, res=res: self._eval(m, v, res))
            self._eval(m, v, res)
        nf = ttk.Frame(p); nf.pack(fill="x", pady=4)
        ttk.Label(nf, text="Notes").pack(side="left")
        self.notes = tk.Text(nf, height=3, width=80); self.notes.pack(side="left", fill="x", expand=True)
        bf = ttk.Frame(p); bf.pack(anchor="w", pady=4)
        for verdict in ("PASS", "FAIL", "SKIP"):
            ttk.Button(bf, text=f"Record {verdict}", command=lambda v=verdict, s=s: self.record(s, v)).pack(side="left", padx=3)

    def _eval(self, m, v, res):
        ev = m.evaluate(v.get())
        res.configure(text="" if ev is None else ("PASS" if ev else "FAIL"),
                      foreground="#0a5" if ev else "#c00")

    def record(self, s, verdict):
        vals = {k: v.get() for k, v in self.values.get(s.sid, {}).items()}
        notes = self.notes.get("1.0", "end").strip() if hasattr(self, "notes") else ""
        auto = [m.evaluate(vals.get(m.key, "")) for m in s.measurements]
        if verdict == "PASS" and any(a is False for a in auto):
            if not messagebox.askyesno("Out of range", "Some readings are out of range. Record PASS anyway?"):
                return
        self.log.step_result(s, verdict, vals, notes)
        self.log_line(f"[step {s.sid}] recorded {verdict}")

    def _build_motion(self, nb):
        f = ttk.Frame(nb, padding=10); nb.add(f, text="Motion")
        self.axis = tk.IntVar(value=0)
        r = ttk.Frame(f); r.pack(anchor="w", pady=4)
        ttk.Label(r, text="Axis").pack(side="left")
        ttk.Spinbox(r, from_=0, to=5, textvariable=self.axis, width=4,
                    command=self._axis_changed).pack(side="left", padx=4)
        self.axis_lbl = ttk.Label(r, text=core.axis_connector(0)); self.axis_lbl.pack(side="left", padx=6)
        r = ttk.Frame(f); r.pack(anchor="w", pady=6)
        for txt, cmd in [("Inhibit 1 (safe)", lambda: self.send(["inhibit 1"])),
                         ("Inhibit 0", lambda: self.send(["inhibit 0"])),
                         ("ARM", self.arm), ("DIR 0", lambda: self.send([f"dir {self.axis.get()} 0"])),
                         ("DIR 1", lambda: self.send([f"dir {self.axis.get()} 1"]))]:
            ttk.Button(r, text=txt, command=cmd).pack(side="left", padx=3)
        pf = ttk.LabelFrame(f, text="Bounded pulse train", padding=8); pf.pack(anchor="w", fill="x", pady=6)
        self.p_hz = tk.StringVar(value="500"); self.p_n = tk.StringVar(value="200")
        ttk.Label(pf, text="Hz").pack(side="left"); ttk.Entry(pf, textvariable=self.p_hz, width=8).pack(side="left")
        ttk.Label(pf, text="  pulses (− = reverse on bench image)").pack(side="left")
        ttk.Entry(pf, textvariable=self.p_n, width=8).pack(side="left")
        ttk.Button(pf, text="Send pulses", command=self.pulse).pack(side="left", padx=8)
        tf = ttk.LabelFrame(f, text="Tone (probe only, continuous for ms)", padding=8); tf.pack(anchor="w", fill="x", pady=6)
        self.t_hz = tk.StringVar(value="1000"); self.t_ms = tk.StringVar(value="2000")
        ttk.Label(tf, text="Hz").pack(side="left"); ttk.Entry(tf, textvariable=self.t_hz, width=8).pack(side="left")
        ttk.Label(tf, text="  ms").pack(side="left"); ttk.Entry(tf, textvariable=self.t_ms, width=8).pack(side="left")
        ttk.Button(tf, text="Tone", command=self.tone).pack(side="left", padx=8)
        self.caps_lbl = ttk.Label(f, foreground="#a60"); self.caps_lbl.pack(anchor="w", pady=8)
        self._caps_changed()

    def _build_console(self, nb):
        f = ttk.Frame(nb, padding=10); nb.add(f, text="Console")
        self.cmd_var = tk.StringVar()
        e = ttk.Entry(f, textvariable=self.cmd_var, width=60); e.pack(side="left", anchor="n")
        e.bind("<Return>", lambda ev: self._console_send())
        ttk.Button(f, text="Send", command=self._console_send).pack(side="left", anchor="n", padx=4)
        ttk.Label(f, text="probe: help, info, in, selftest, mon 10, w5500, uartloop, wifi, temp, safe\n"
                          "bench: BENCH:STATUS, BENCH:ARM, BENCH:PULSE <hz> <n>, BENCH:STOP",
                  foreground="#555", justify="left").pack(side="left", anchor="n", padx=10)

    # ── connection ──────────────────────────────────────────────────────────────────────
    def refresh_ports(self):
        ports = core.list_ports()
        self.port_cb["values"] = [f"{d}  {'[ESP32-S3] ' if esp else ''}{desc}" for d, desc, esp in ports]
        if not self.port_var.get():
            esp = [d for d, _, e in ports if e]
            if esp:
                self.port_var.set(esp[0])

    def connect(self):
        port = self.port_var.get().split()[0] if self.port_var.get() else ""
        if not port:
            messagebox.showwarning("Port", "Choose a port"); return
        self.disconnect()

        def job():
            link, info = core.detect(port, on_rx=self._rx)
            def done():
                if link is None:
                    self.fw_var.set("no firmware reply"); self.log_line(info); return
                self.link = link
                self.fw_var.set(f"{port}: {link.kind} firmware")
                self.log.md(f"- connected {port}: **{link.kind}** firmware\n\n```\n{info}\n```")
                self.log.event("connect", port=port, firmware=link.kind, info=info)
            self.ui(done)
        self.worker.submit(job)

    def disconnect(self):
        if self.link:
            self.link.close(); self.link = None
            self.fw_var.set("not connected")

    # ── commands ────────────────────────────────────────────────────────────────────────
    def _rx(self, line):
        self.log.rx(line); self.ui(self.log_line, "  " + line)

    def log_line(self, text):
        self.logbox.insert("end", time.strftime("%H:%M:%S ") + text + "\n"); self.logbox.see("end")

    def _adapt(self, cmd: str) -> str | None:
        """Translate generic motion commands for the controller bench image."""
        if not self.link or self.link.kind == "probe":
            return cmd
        p = cmd.split()
        if p[0] == "safe":
            return "BENCH:STOP"
        if cmd.startswith("BENCH:") or p[0] in ("VERSION", "ESTOP?", "DBG:1", "DBG:0"):
            return cmd
        self.log_line(f"[skip] '{cmd}' is a probe command; the controller bench image only takes BENCH:*")
        return None

    def send(self, cmds, confirm: str = ""):
        if not self.link:
            messagebox.showwarning("Not connected", "Connect first"); return
        out = []
        for c in cmds:
            c2 = self._adapt(c)
            if c2 is None:
                continue
            try:
                core.check_motion_command(c2, self.drive_connected.get())
            except core.CapError as e:
                messagebox.showerror("Refused by caps", f"{c2}\n\n{e}"); return
            out.append(c2)
        if not out:
            return
        if confirm and not messagebox.askyesno("Confirm", confirm + "\n\n" + "\n".join(out)):
            return

        def job():
            for c in out:
                self.log.tx(c); self.ui(self.log_line, f"> {c}")
                ok, _ = self.link.command(c, core.command_timeout(c))
                if not ok:
                    self.ui(self.log_line, f"[{c}] did not return OK — sequence stopped")
                    break
        self.worker.submit(job)

    def run_action(self, act: core.Action):
        if self.link and self.link.kind == "probe" and not act.probe_only:
            messagebox.showinfo("Bench image needed", "Flash the controller -DR13_BENCH_ONE_AXIS=ON image and reconnect.")
            return
        confirm = act.confirm
        if not confirm and any(core.is_motion_command(c) for c in act.commands) and self.drive_connected.get():
            confirm = "Drive connected: this can move the motor."
        self.send(act.commands, confirm)

    def arm(self):
        if self.link and self.link.kind == "bench":
            self.send(["BENCH:ARM"], "Arm the output latch?")
        else:
            self.send(["inhibit 0", "arm"], "Arm the output latch?")

    def pulse(self):
        try:
            hz, n = int(self.p_hz.get()), int(self.p_n.get())
        except ValueError:
            messagebox.showerror("Pulse", "Hz and pulses must be integers"); return
        if self.link and self.link.kind == "bench":
            self.send([f"BENCH:PULSE {hz} {n}"], f"Send {n} pulses at {hz} Hz on axis 0?")
        else:
            ax = self.axis.get()
            cmds = [f"dir {ax} {0 if n >= 0 else 1}", f"pulse {ax} {hz} {abs(n)}"]
            self.send(cmds, f"Send {abs(n)} pulses at {hz} Hz on axis {ax} ({core.axis_connector(ax)})?")

    def tone(self):
        ax = self.axis.get()
        self.send([f"tone {ax} {self.t_hz.get()} {self.t_ms.get()}"],
                  f"Continuous {self.t_hz.get()} Hz for {self.t_ms.get()} ms on axis {ax}?")

    def _console_send(self):
        c = self.cmd_var.get().strip()
        if c:
            self.send([c], "This can move the motor. Send?" if core.is_motion_command(c) and self.drive_connected.get() else "")

    def stop(self):
        if not self.link:
            self.log_line("[STOP] not connected — use the J2 link / drive power"); return
        cmd = self.link.stop_command
        try:
            self.link.ser.write((cmd + "\n").encode() if self.link.kind == "probe"
                                else core.cobs_encode(bytes([core.CH_CMD]) + cmd.encode()) + b"\x00")
            self.link.ser.flush()
            self.log.tx(cmd + " (STOP button)")
            msg = f"[STOP] sent {cmd}"
            if self.link.kind == "probe" and self.worker.busy:
                msg += " — probe is busy and will act when the current command ends; pull J2 for an immediate stop"
            self.log_line(msg)
        except Exception as e:
            self.log_line(f"[STOP] send failed: {e} — pull J2 / switch off drive")

    def _axis_changed(self):
        self.axis_lbl.configure(text=core.axis_connector(self.axis.get()))

    def _caps_changed(self):
        c = core.DRIVE_CAPS if self.drive_connected.get() else core.SIGNAL_CAPS
        if hasattr(self, "caps_lbl"):
            self.caps_lbl.configure(text=f"Caps: pulse ≤ {c['pulse_hz']} Hz / {c['pulse_n']} pulses, "
                                         f"tone ≤ {c['tone_hz']} Hz / {c['tone_ms']} ms "
                                         f"({'drive connected' if self.drive_connected.get() else 'signal only: NO cable'})")

    # ── live status ─────────────────────────────────────────────────────────────────────
    def _poll(self):
        if self.live.get() and self.link and not self.worker.busy and self.worker.jobs.empty():
            def job():
                st = self.link.status()
                self.ui(self._show_status, st)
            self.worker.submit(job)
        self.root.after(POLL_MS, self._poll)

    def _show_status(self, st: core.BoardStatus):
        f = lambda v, t="1", f_="0": "—" if v is None else str(v)
        self.st["loop"].set("—" if st.loop_closed is None else ("CLOSED (run)" if st.loop_closed else "OPEN (stop)"))
        self.st["inhibit"].set(f(st.inhibit)); self.st["arm"].set(f(st.arm))
        self.st["armed"].set("—" if st.armed is None else ("YES" if st.armed else "no"))
        self.st["alarms"].set(",".join(map(str, st.alarms)) if st.alarms else "—")
        self.st["step"].set(st.step or "—"); self.st["dir"].set(st.dir or "—")
        self.st["pos"].set("—" if st.pos is None else str(st.pos))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="")
    ap.add_argument("--operator", default="")
    a = ap.parse_args()
    root = tk.Tk()
    App(root, a.port, a.operator)
    root.mainloop()


if __name__ == "__main__":
    main()
