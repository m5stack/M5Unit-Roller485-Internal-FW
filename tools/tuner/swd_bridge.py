#!/usr/bin/env python3
"""SWD bridge for the Roller485 detent tuner.

Serves index.html and a tiny JSON API, talking to the running firmware over the
ST-Link (pyocd) through the `tune_mailbox` struct -- no serial path required.

    tools/tuner/.venv/bin/python tools/tuner/swd_bridge.py [--elf path] [--port 8765]
"""
import argparse, json, os, struct, sys, threading, time
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_ELF = os.path.join(HERE, "..", "..", "code", "ROLLER485", "gcc_build", "ROLLER485.elf")
TUNE_MAGIC = 0x454E5554

# struct tune_mailbox_t layout (all 4-byte fields, in order)
FIELDS = ["magic", "req_seq", "ack_seq", "cmd", "idx", "value", "result", "status",
          "param_count", "preset_count", "preset_index",
          "tick", "position", "angle_rad", "torque", "current_ma", "rps", "sub_position", "motor_on"]
FMT = "<IIIIIffIIII" + "IifffffI"
assert struct.calcsize(FMT) == len(FIELDS) * 4
OFF = {n: i * 4 for i, n in enumerate(FIELDS)}
SIZE = len(FIELDS) * 4
CMD = {"set": 1, "get": 2, "preset": 3, "motor": 4, "zero": 5}


def mailbox_address(elf_path):
    from elftools.elf.elffile import ELFFile
    with open(elf_path, "rb") as f:
        elf = ELFFile(f)
        symtab = elf.get_section_by_name(".symtab")
        for sym in symtab.iter_symbols():
            if sym.name == "tune_mailbox":
                return sym["st_value"]
    raise SystemExit("tune_mailbox symbol not found in " + elf_path)


class Target:
    def __init__(self, elf):
        from pyocd.core.helpers import ConnectHelper
        self.addr = mailbox_address(elf)
        self.session = ConnectHelper.session_with_chosen_probe(
            target_override="cortex_m", connect_mode="attach",
            resume_on_disconnect=True, options={"frequency": 4000000})
        self.session.open()
        self.t = self.session.target
        self.lock = threading.Lock()
        self.seq = self.rd32("req_seq")
        magic = self.rd32("magic")
        if magic != TUNE_MAGIC:
            raise SystemExit(f"mailbox magic mismatch at 0x{self.addr:08x}: 0x{magic:08x} (is the new firmware flashed?)")
        print(f"attached: mailbox @0x{self.addr:08x}, {self.rd32('param_count')} params, "
              f"{self.rd32('preset_count')} presets, core {self.t.get_state().name}")

    def rd32(self, field):
        return self.t.read32(self.addr + OFF[field])

    def snapshot(self):
        with self.lock:
            words = self.t.read_memory_block32(self.addr, SIZE // 4)
        raw = struct.pack("<%dI" % len(words), *words)
        vals = struct.unpack(FMT, raw)
        return dict(zip(FIELDS, vals))

    def command(self, cmd, idx=0, value=0.0, timeout=0.3):
        with self.lock:
            self.seq = (self.seq + 1) & 0xFFFFFFFF
            a = self.addr
            self.t.write32(a + OFF["cmd"], CMD[cmd])
            self.t.write32(a + OFF["idx"], int(idx) & 0xFFFFFFFF)
            self.t.write32(a + OFF["value"], struct.unpack("<I", struct.pack("<f", float(value)))[0])
            self.t.write32(a + OFF["req_seq"], self.seq)
            t0 = time.time()
            while self.t.read32(a + OFF["ack_seq"]) != self.seq:
                if time.time() - t0 > timeout:
                    raise TimeoutError("firmware did not service the mailbox (main loop stalled?)")
                time.sleep(0.001)
            res = struct.unpack("<f", struct.pack("<I", self.t.read32(a + OFF["result"])))[0]
            st = self.t.read32(a + OFF["status"])
        return {"ok": bool(st), "value": res}


def make_handler(target):
    class H(SimpleHTTPRequestHandler):
        def __init__(self, *a, **k):
            super().__init__(*a, directory=HERE, **k)

        def log_message(self, *a):  # quiet
            pass

        def _json(self, obj, code=200):
            body = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path.startswith("/api/ping"):
                return self._json({"bridge": "swd", "mailbox": hex(target.addr)})
            if self.path.startswith("/api/telemetry"):
                try:
                    return self._json(target.snapshot())
                except Exception as e:
                    return self._json({"error": str(e)}, 500)
            return super().do_GET()

        def do_POST(self):
            if not self.path.startswith("/api/cmd"):
                return self._json({"error": "not found"}, 404)
            n = int(self.headers.get("Content-Length", 0))
            req = json.loads(self.rfile.read(n) or b"{}")
            try:
                r = target.command(req["cmd"], req.get("idx", 0), req.get("value", 0.0))
                return self._json(r)
            except Exception as e:
                return self._json({"error": str(e)}, 500)
    return H


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--elf", default=DEFAULT_ELF)
    ap.add_argument("--port", type=int, default=8765)
    a = ap.parse_args()
    target = Target(os.path.abspath(a.elf))
    srv = ThreadingHTTPServer(("127.0.0.1", a.port), make_handler(target))
    print(f"tuner: http://localhost:{a.port}/  (ctrl-c to stop)")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        target.session.close()


if __name__ == "__main__":
    main()
