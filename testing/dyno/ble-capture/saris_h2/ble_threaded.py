"""Threaded BLE session helper for the Saris H2.

A single asyncio event loop runs on a dedicated thread (`run_forever`).
The main thread submits work via `asyncio.run_coroutine_threadsafe`, which
returns a `concurrent.futures.Future` we can wait on with a timeout — so
failures are visible instead of silently dropped.

Usage:
    sess = SarisH2Session(addr)
    sess.start()                            # connects + subscribes; blocks until ready
    sess.write_proprietary("HEADLESS", cmd_bytes)   # blocks until sent or timeout
    sess.events                             # list of (t_ble_s, kind, info)
    sess.stop()                             # tears down + joins thread
"""
from __future__ import annotations
import asyncio
import threading
import time
from concurrent.futures import Future, TimeoutError as FutTimeout

from bleak import BleakClient, BleakScanner

from .protocol import (
    DEFAULT_ADDR,
    SARIS_RESISTANCE_UUID,
    CPS_MEASUREMENT_UUID,
    parse_cps,
)


class SarisH2Session:
    """Threaded BLE session against the Saris H2 trainer.

    Always subscribes to CPS measurement (brake-safe). Subscribes to the
    proprietary control characteristic only if `subscribe_proprietary=True`,
    which is needed before any write to that characteristic but engages the
    brake.
    """

    def __init__(self, addr: str = DEFAULT_ADDR, *, subscribe_proprietary: bool = True,
                 connect_timeout_s: float = 15.0):
        self.addr = addr
        self.subscribe_proprietary = subscribe_proprietary
        self.connect_timeout_s = connect_timeout_s

        self.events: list[tuple[float, str, str]] = []
        self.t0: float | None = None
        self.cps_count = 0
        self.last_power: int | None = None

        self._client: BleakClient | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._thread: threading.Thread | None = None
        self._ready = threading.Event()
        self._setup_error: BaseException | None = None

    # ----------------------------------------------------------- setup/teardown

    async def _setup(self):
        try:
            self._client = BleakClient(self.addr, timeout=self.connect_timeout_s)
            await self._client.connect()
        except Exception:
            device = await BleakScanner.find_device_by_address(self.addr,
                                                               timeout=self.connect_timeout_s)
            if not device:
                raise RuntimeError(f"trainer not found at {self.addr}")
            self._client = BleakClient(device, timeout=self.connect_timeout_s)
            await self._client.connect()

        self.t0 = time.time()
        self._ev("connected", "")

        await self._client.start_notify(CPS_MEASUREMENT_UUID, self._on_cps)
        self._ev("subscribed_cps", "")

        if self.subscribe_proprietary:
            await self._client.start_notify(SARIS_RESISTANCE_UUID, self._on_resp)
            self._ev("subscribed_proprietary", "(brake engaged)")

    async def _teardown(self):
        if self._client is None:
            return
        for uuid in (CPS_MEASUREMENT_UUID, SARIS_RESISTANCE_UUID):
            try:
                await self._client.stop_notify(uuid)
            except Exception:
                pass
        try:
            await self._client.disconnect()
        except Exception:
            pass
        self._ev("disconnected", "")

    # -------------------------------------------------------------- callbacks

    def _on_cps(self, _, data: bytearray):
        try:
            m = parse_cps(bytes(data))
            self.cps_count += 1
            self.last_power = m.power_w
            self._ev("cps", f"pwr={m.power_w}W acc_t={m.acc_torque_nm:.2f}Nm "
                            f"hex={bytes(data).hex()}")
        except Exception as e:
            self._ev("cps_err", str(e))

    def _on_resp(self, _, data: bytearray):
        self._ev("resp", f"hex={bytes(data).hex()}")

    def _ev(self, kind: str, info: str):
        t = (time.time() - self.t0) if self.t0 else 0.0
        self.events.append((t, kind, info))

    # ------------------------------------------------------------ thread main

    def _thread_main(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._setup())
        except BaseException as e:
            self._setup_error = e
            self._ready.set()
            try: self._loop.close()
            except Exception: pass
            return
        self._ready.set()
        try:
            self._loop.run_forever()
        finally:
            try:
                self._loop.run_until_complete(self._teardown())
            except Exception:
                pass
            self._loop.close()

    # --------------------------------------------------------- public API

    def start(self):
        self._thread = threading.Thread(target=self._thread_main, daemon=True,
                                        name="SarisH2-BLE")
        self._thread.start()
        if not self._ready.wait(timeout=self.connect_timeout_s + 10.0):
            raise TimeoutError("BLE setup did not complete in time")
        if self._setup_error is not None:
            raise self._setup_error

    def write_proprietary(self, label: str, cmd_bytes: bytes,
                          *, timeout_s: float = 3.0,
                          response: bool = False) -> bool:
        """Schedule a write to the Saris proprietary control char.

        Returns True if the write resolved cleanly, False (and logs the error
        via events) if it threw or timed out.
        """
        if self._loop is None:
            raise RuntimeError("session not started")
        self._ev("write_submit", f"{label} hex={cmd_bytes.hex()}")

        async def _do_write():
            await self._client.write_gatt_char(SARIS_RESISTANCE_UUID, cmd_bytes,
                                               response=response)

        fut: Future = asyncio.run_coroutine_threadsafe(_do_write(), self._loop)
        try:
            fut.result(timeout=timeout_s)
            self._ev("write_ok", label)
            return True
        except FutTimeout:
            self._ev("write_timeout", f"{label} after {timeout_s:.1f}s")
            return False
        except Exception as e:
            self._ev("write_err", f"{label}: {e!r}")
            return False

    def stop(self):
        if self._loop is None:
            return
        try:
            self._loop.call_soon_threadsafe(self._loop.stop)
        except Exception:
            pass
        if self._thread:
            self._thread.join(timeout=10.0)

    # ---------------------------------------------------------- diagnostics

    def event_count_by_kind(self) -> dict[str, int]:
        out: dict[str, int] = {}
        for _, kind, _ in self.events:
            out[kind] = out.get(kind, 0) + 1
        return out
