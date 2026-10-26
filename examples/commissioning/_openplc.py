"""OpenPLC v3 as a test fixture: the runtime's web UI driven as an API
(login, program upload and compilation, slave devices, settings, start /
stop) and the address the PLC's Modbus master dials to reach this machine.
Shared by the commissioning demos (`openplc_pick_cell`, `openplc_feed_line`);
the container is `docker compose up -d` in either folder."""

from __future__ import annotations

import http.cookiejar
import re
import time
import urllib.parse
import urllib.request

OPENPLC = "http://127.0.0.1:8080"
CONTAINER = "botrail-openplc"


def host_ip_for_the_plc(container: str = CONTAINER) -> str:
    """The address the PLC's Modbus master dials to reach this machine.
    libmodbus wants an IP, not a name. With the compose file's host
    networking that is loopback; on a bridge network it is the address
    `host.docker.internal` resolves to inside the container (the host
    gateway), else the default bridge's gateway, else Docker's 172.17.0.1."""
    import subprocess

    try:
        mode = subprocess.run(["docker", "inspect", container, "-f", "{{.HostConfig.NetworkMode}}"],
                              capture_output=True, text=True, timeout=10).stdout.strip()
        if mode == "host":
            return "127.0.0.1"
    except Exception:  # noqa: BLE001
        pass
    try:
        out = subprocess.run(["docker", "exec", container, "getent", "hosts", "host.docker.internal"],
                             capture_output=True, text=True, timeout=10).stdout.split()
        if out and out[0].count(".") == 3:
            return out[0]
    except Exception:  # noqa: BLE001
        pass
    try:
        out = subprocess.run(["docker", "network", "inspect", "bridge", "-f", "{{(index .IPAM.Config 0).Gateway}}"],
                             capture_output=True, text=True, timeout=10).stdout.strip()
        if out.count(".") == 3:
            return out
    except Exception:  # noqa: BLE001
        pass
    return "172.17.0.1"


class OpenPlc:
    """The OpenPLC v3 web UI as an API: login, program upload and
    compilation, slave devices, settings, start / stop."""

    def __init__(self, base: str = OPENPLC, user: str = "openplc", password: str = "openplc"):
        self.base = base.rstrip("/")
        self.jar = http.cookiejar.CookieJar()
        self.opener = urllib.request.build_opener(urllib.request.HTTPCookieProcessor(self.jar))
        self._post("/login", {"username": user, "password": password})

    def _get(self, path: str, timeout: float = 30.0) -> str:
        with self.opener.open(self.base + path, timeout=timeout) as r:
            return r.read().decode("utf-8", "replace")

    def _post(self, path: str, fields: dict, timeout: float = 30.0) -> str:
        data = urllib.parse.urlencode(fields).encode()
        with self.opener.open(self.base + path, data=data, timeout=timeout) as r:
            return r.read().decode("utf-8", "replace")

    def _post_file(self, path: str, field: str, filename: str, content: bytes) -> str:
        boundary = "----botrail%d" % int(time.time() * 1000)
        body = (
            f"--{boundary}\r\nContent-Disposition: form-data; name=\"{field}\"; filename=\"{filename}\"\r\n"
            f"Content-Type: text/plain\r\n\r\n"
        ).encode() + content + f"\r\n--{boundary}--\r\n".encode()
        req = urllib.request.Request(
            self.base + path, data=body,
            headers={"Content-Type": f"multipart/form-data; boundary={boundary}"},
        )
        with self.opener.open(req, timeout=60.0) as r:
            return r.read().decode("utf-8", "replace")

    def wait_ready(self, seconds: float = 60.0) -> None:
        deadline = time.time() + seconds
        while time.time() < deadline:
            try:
                self._get("/login", timeout=3.0)
                return
            except Exception:  # noqa: BLE001
                time.sleep(1.0)
        raise RuntimeError(f"OpenPLC at {self.base} did not answer within {seconds} s — is the container up?")

    def stop(self) -> None:
        self._get("/stop_plc")

    def start(self) -> None:
        self._get("/start_plc")

    def status(self) -> str:
        page = self._get("/dashboard")
        m = re.search(r"Status:\s*<[^>]*>\s*([A-Za-z]+)", page) or re.search(r"(Running|Stopped|Compiling)", page)
        return m.group(1) if m else "?"

    def settings(self, polling_ms: int, timeout_ms: int = 1000) -> None:
        self._post("/settings", {
            "modbus_server": "true", "modbus_server_port": "502",
            "slave_polling_period": str(polling_ms), "slave_timeout": str(timeout_ms),
            "auto_run_text": "false",
        })

    def clear_slave_devices(self) -> None:
        page = self._get("/modbus")
        for dev_id in re.findall(r"modbus-edit-device\?table_id=(\d+)", page):
            self._get(f"/delete-device?dev_id={dev_id}")

    def add_slave_device(self, name: str, ip: str, port: int, di: int, coils: int, ir: int, hr: int) -> None:
        """A Modbus TCP slave the PLC polls: `di` discrete inputs → %IX100.0…,
        `coils` → %QX100.0…, `ir` input registers → %IW100…, `hr` holding
        registers (written) → %QW100…."""
        self._post("/add-modbus-device", {
            "device_name": name, "device_protocol": "TCP", "device_id": "1",
            "device_ip": ip, "device_port": str(port),
            "device_cport": "/dev/ttyUSB0", "device_baud": "115200", "device_parity": "None",
            "device_data": "8", "device_stop": "1", "device_pause": "0",
            "di_start": "0", "di_size": str(di), "do_start": "0", "do_size": str(coils),
            "ai_start": "0", "ai_size": str(ir), "aor_start": "0", "aor_size": "0",
            "aow_start": "0", "aow_size": str(hr),
        })

    def load_program(self, name: str, st_text: str, timeout: float = 240.0) -> str:
        """Uploads the ST, compiles it (matiec + gcc inside the container) and
        returns the compilation log; raises when the compiler says no."""
        page = self._post_file("/upload-program", "file", f"{name}.st", st_text.encode("utf-8"))
        m = re.search(r"name='prog_file'[^>]*value='([^']+)'|value='([^']+)'[^>]*name='prog_file'", page)
        prog_file = next(g for g in (m.groups() if m else ()) if g) if m else None
        if prog_file is None:
            raise RuntimeError("OpenPLC did not accept the upload (no prog_file in the reply)")
        epoch = re.search(r"name='epoch_time'[^>]*value='(\d+)'|value='(\d+)'[^>]*name='epoch_time'", page)
        epoch_time = next((g for g in (epoch.groups() if epoch else ()) if g), str(int(time.time())))
        self._post("/upload-program-action", {
            "prog_name": name, "prog_descr": "botrail ST export", "prog_file": prog_file,
            "epoch_time": epoch_time,
        })
        self._get(f"/compile-program?file={prog_file}")
        deadline = time.time() + timeout
        log = ""
        while time.time() < deadline:
            log = self._get("/compilation-logs")
            if "Compilation finished successfully!" in log:
                return log
            if "Compilation finished with errors!" in log:
                raise RuntimeError("OpenPLC could not compile the program:\n" + log)
            time.sleep(1.0)
        raise RuntimeError("OpenPLC compilation did not finish in time:\n" + log)
