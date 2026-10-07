"""Shared persisted APB processing controls; absence means enabled, corruption means off."""
import json
import os
import time
from pathlib import Path

DEFAULTS = {"version": 1, "enabled": True, "vio": True, "trn": True, "imuFallback": True, "lidar": True, "revision": 0}


class FusionControl:
    def __init__(self, path=None, clock=time.monotonic):
        self.path = Path(path or os.environ.get("WINGMAN_FUSION_SETTINGS", "/nvram/sensor-fusion.json"))
        self.clock = clock
        self.next_read = 0.0
        self.settings = dict(DEFAULTS)

    def enabled(self, source):
        now = self.clock()
        if now >= self.next_read:
            self.next_read = now + 0.5
            try:
                value = json.loads(self.path.read_text())
                if value.get("version") != 1 or any(type(value.get(k)) is not bool for k in ("enabled", "vio", "trn", "imuFallback")) or ("lidar" in value and type(value["lidar"]) is not bool):
                    raise ValueError("invalid fusion settings")
                self.settings = {**DEFAULTS, **value}
            except FileNotFoundError:
                self.settings = dict(DEFAULTS)
            except (OSError, ValueError, TypeError):
                self.settings = {**DEFAULTS, "enabled": False}
        return self.settings["enabled"] and self.settings.get(source, False)


def native_vio_running(path=None):
    """Avoid paying for the legacy optical-flow worker while native VIO is producing."""
    try:
        stamp = float(Path(path or "/run/wingman-vio-native.timestamp").read_text())
        return 0 <= time.monotonic() - stamp < 1.0
    except (OSError, ValueError):
        return False


class CpuReserve:
    """Optional workload admission with hysteresis and the existing 25% reserve floor."""
    def __init__(self, path="/proc/stat"):
        self.path = Path(path)
        self.previous = None
        self.overloaded = 0
        self.recovered = 0
        self.paused = False
        self.reserve = None

    def allows(self):
        try:
            values = [int(x) for x in self.path.read_text().splitlines()[0].split()[1:9]]
            total, idle = sum(values), values[3] + values[4]
            previous, self.previous = self.previous, (total, idle)
            if previous is None or total <= previous[0]:
                return False
            self.reserve = (idle - previous[1]) / (total - previous[0])
            self.overloaded = self.overloaded + 1 if self.reserve < .25 else 0
            self.recovered = self.recovered + 1 if self.reserve >= .35 else 0
            if self.overloaded >= 3:
                self.paused = True
            if self.recovered >= 5:
                self.paused = False
            return not self.paused
        except (OSError, ValueError, IndexError):
            return False  # Unknown resource headroom cannot admit optional heavy work.
