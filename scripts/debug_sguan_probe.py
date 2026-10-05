#!/usr/bin/env python3
"""Host-side debug probe: pyOCD + ttyACM0 AS5600 -> NDJSON."""
from __future__ import annotations

import json
import re
import struct
import time
from pathlib import Path

LOG = Path("/home/tony/CW32/cw32l012_blank/.cursor/debug-d7364d.log")
ELF = Path("/home/tony/CW32/cw32l012_blank/build/Debug/cw32l012_blank.elf")
PACK = Path.home() / ".local/share/cmsis-pack-manager/WHXY/CW32L012_DFP/1.0.2.pack"
SESSION = "d7364d"


def nm_map(elf: Path) -> dict[str, int]:
    import subprocess

    out = subprocess.check_output(["arm-none-eabi-nm", str(elf)], text=True)
    m: dict[str, int] = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) >= 3:
            m[parts[2]] = int(parts[0], 16)
    return m


def f32(b: bytes) -> float:
    return struct.unpack("<f", b)[0]


def u32(b: bytes) -> int:
    return struct.unpack("<I", b)[0]


def u16(b: bytes) -> int:
    return struct.unpack("<H", b)[0]


def log_line(hid: str, loc: str, msg: str, data: dict) -> None:
    rec = {
        "sessionId": SESSION,
        "runId": "restart-trace",
        "hypothesisId": hid,
        "location": loc,
        "message": msg,
        "data": data,
        "timestamp": int(time.time() * 1000),
    }
    with LOG.open("a", encoding="utf-8") as f:
        f.write(json.dumps(rec, ensure_ascii=False) + "\n")


def as5600_sample(seconds: float = 2.0) -> dict:
    try:
        import serial
    except Exception as e:
        return {"error": str(e)}
    ser = serial.Serial("/dev/ttyACM0", 115200, timeout=0.2, dsrdtr=False, rtscts=False)
    try:
        ser.dtr = False
        ser.rts = False
    except Exception:
        pass
    t0 = time.time()
    raw = b""
    while time.time() - t0 < seconds:
        n = ser.in_waiting
        raw += ser.read(n if n else 64)
    ser.close()
    degs = [float(x) for x in re.findall(rb"as5600: ([0-9.]+) deg", raw)]
    if not degs:
        return {"samples": 0, "text_tail": raw[-200:].decode("utf-8", "replace")}
    jumps = 0
    max_step = 0.0
    stalls = 0
    stall_ms = 0
    unwrapped = 0.0
    last = degs[0]
    t_est = seconds / max(len(degs) - 1, 1)
    vel_bins = []
    bin_sum = 0.0
    bin_n = 0
    bin_t = 0.0
    for b in degs[1:]:
        d = b - last
        if d > 180.0:
            d -= 360.0
        elif d < -180.0:
            d += 360.0
        unwrapped += d
        ad = abs(d)
        if ad > max_step:
            max_step = ad
        if ad > 30.0:
            jumps += 1
        if ad < 0.4:
            stall_ms += t_est * 1000.0
            stalls += 1
        last = b
        bin_sum += d
        bin_n += 1
        bin_t += t_est
        if bin_t >= 0.1:
            vel_bins.append(round(bin_sum / bin_t, 1))
            bin_sum = 0.0
            bin_n = 0
            bin_t = 0.0
    rpm = (unwrapped / seconds) * (60.0 / 360.0)
    return {
        "samples": len(degs),
        "first": degs[0],
        "last": degs[-1],
        "min": min(degs),
        "max": max(degs),
        "span": max(degs) - min(degs),
        "jumps_gt30": jumps,
        "max_step": max_step,
        "small_steps": stalls,
        "approx_stall_ms": round(stall_ms, 1),
        "unwrapped_deg": round(unwrapped, 1),
        "mean_rpm": round(rpm, 1),
        "vel_dps_100ms": vel_bins[:40],
    }


def uart_sample(port: str, seconds: float = 2.0) -> dict:
    try:
        import serial
    except Exception as e:
        return {"error": str(e)}
    ser = serial.Serial(port, 115200, timeout=0.2)
    t0 = time.time()
    raw = b""
    while time.time() - t0 < seconds:
        n = ser.in_waiting
        raw += ser.read(n if n else 64)
    ser.close()
    text = raw.decode("utf-8", "replace")
    sts = [int(x) for x in re.findall(r"st=(\d+)", text)]
    return {
        "port": port,
        "n": len(raw),
        "st_unique": sorted(set(sts)),
        "st_n": len(sts),
        "tail": text[-240:],
    }


def snapshot(target, sym, i: int) -> dict:
    def rb(name: str, n: int) -> bytes:
        return bytes(target.read_memory_block8(sym[name], n))

    return {
        "i": i,
        "status": rb("g_dbg_st", 1)[0],
        "started": rb("g_dbg_started", 1)[0],
        "real_speed": f32(rb("g_dbg_spd", 4)),
        "target_speed": f32(rb("g_dbg_tgt", 4)),
        "real_re": f32(rb("g_dbg_re", 4)),
        "ud": f32(rb("g_dbg_ud", 4)),
        "uq": f32(rb("g_dbg_uq", 4)),
        "id": f32(rb("g_dbg_id", 4)),
        "iq": f32(rb("g_dbg_iq", 4)),
        "isr": u32(rb("g_dbg_isr", 4)),
        "ms": u32(rb("g_dbg_ms", 4)),
        "boot": u32(rb("g_dbg_boot", 4)),
        "rst": u32(rb("g_dbg_rst", 4)),
        "click": u32(rb("g_dbg_click", 4)),
    }


def main() -> None:
    from pyocd.core.helpers import ConnectHelper

    try:
        enc = as5600_sample(8.0)
        log_line("E", "ttyACM0", "as5600", enc)
    except Exception as e:
        log_line("E", "ttyACM0", "as5600-error", {"error": str(e)})
    return

    try:
        log_line("C", "ttyUSB0", "mcu-uart", uart_sample("/dev/ttyUSB0", 2.0))
    except Exception as e:
        log_line("C", "ttyUSB0", "mcu-uart-error", {"error": str(e)})

    sym = nm_map(ELF)
    opts = {
        "pack": [str(PACK)],
        "frequency": 100000,
        "connect_mode": "attach",
    }
    last_isr = None
    try:
        with ConnectHelper.session_with_chosen_probe(unique_id=None, **opts) as session:
            session.options.set("target_override", "cw32l012c8")
            target = session.target
            for i in range(3):
                try:
                    target.halt()
                    rec = snapshot(target, sym, i)
                    target.resume()
                    if last_isr is not None and rec["isr"] < last_isr:
                        rec["isr_drop"] = True
                    last_isr = rec["isr"]
                    hid = "C"
                    if rec.get("isr_drop") or rec["ms"] < 50:
                        hid = "A"
                    elif rec["click"] > 0:
                        hid = "B"
                    elif rec["status"] not in (4, 8, 9, 10):
                        hid = "C"
                    elif rec["id"] > 5.0 or rec["iq"] > 5.0:
                        hid = "D"
                    log_line(hid, "probe:halt", "restart-trace", rec)
                except Exception as e:
                    log_line("A", "probe:halt", "swd-fail", {"i": i, "error": str(e)})
                time.sleep(0.4)
    except Exception as e:
        log_line("A", "probe:connect", "swd-connect-fail", {"error": str(e)})


if __name__ == "__main__":
    main()
