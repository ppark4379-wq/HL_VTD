"""Crash-tolerant, dependency-free evidence recording for dashboard video."""
from __future__ import annotations

import copy
import json
import os
import shutil
import threading
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import BinaryIO, Callable, TextIO


VIEWS = ("mainrs", "birdrs")
MANIFEST_NAME = "recording_manifest.json"
LOCK_NAME = ".recording.lock"
# Reserve space for filesystem block rounding and the next atomic manifest.
DISK_WRITE_RESERVE_BYTES = 4 * 1024 * 1024


class VideoRecordingExistsError(RuntimeError):
    """The requested evidence directory is already claimed or non-empty."""


def _utc_iso(wall_time_ns: int) -> str:
    return datetime.fromtimestamp(
        wall_time_ns / 1_000_000_000, tz=timezone.utc
    ).isoformat(timespec="microseconds")


class VideoRecordingSession:
    """Write two independent raw-MJPEG streams and timestamped JSONL indexes.

    Recording failures are retained in status/manifest and never raised from
    ``record_frame`` so browser streaming can continue independently.
    """

    def __init__(
        self,
        directory: Path,
        min_free_bytes: int,
        *,
        disk_usage: Callable[[Path], object] | None = None,
        disk_check_interval_s: float = 0.0,
        manifest_interval_s: float = 1.0,
    ):
        if min_free_bytes <= 0:
            raise ValueError("min_free_bytes must be greater than zero")
        self.directory = directory.expanduser().resolve()
        self.min_free_bytes = int(min_free_bytes)
        self.disk_check_interval_ns = max(
            0, int(disk_check_interval_s * 1_000_000_000)
        )
        self.manifest_interval_ns = max(
            0, int(manifest_interval_s * 1_000_000_000)
        )
        self._disk_usage = disk_usage or shutil.disk_usage
        self._lock = threading.RLock()
        self._lock_path = self.directory / LOCK_NAME
        self.manifest_path = self.directory / MANIFEST_NAME
        self._manifest_tmp = self.directory / f"{MANIFEST_NAME}.tmp"
        self.session_id = (
            datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
            + f"-{os.getpid()}-{uuid.uuid4().hex[:8]}"
        )
        self.created_wall_time_ns = time.time_ns()
        self.created_monotonic_ns = time.monotonic_ns()
        self.status_code = "RECORDING"
        self.error: str | None = None
        self.active = True
        self.closed = False
        self.stopped_wall_time_ns: int | None = None
        self.stopped_monotonic_ns: int | None = None
        self.free_bytes: int | None = None
        self._last_disk_check_ns = 0
        self._bytes_since_disk_check = 0
        self._last_manifest_ns = self.created_monotonic_ns
        self._views: dict[str, dict] = {}

        self.directory.mkdir(parents=True, exist_ok=True)
        try:
            with self._lock_path.open("x", encoding="utf-8") as stream:
                stream.write(
                    f"session_id={self.session_id}\npid={os.getpid()}\n"
                )
        except FileExistsError as error:
            raise VideoRecordingExistsError(
                f"video recording directory is already claimed: {self.directory}"
            ) from error

        existing = [
            path for path in self.directory.iterdir() if path != self._lock_path
        ]
        if existing:
            self._lock_path.unlink(missing_ok=True)
            raise VideoRecordingExistsError(
                "video recording directory is not empty; choose a new unique "
                f"directory: {self.directory}"
            )

        created_paths: list[Path] = []
        try:
            for view in VIEWS:
                data_path = self.directory / f"{view}.mjpeg"
                index_path = self.directory / f"{view}.frames.jsonl"
                data_stream = data_path.open("xb")
                created_paths.append(data_path)
                try:
                    index_stream = index_path.open(
                        "x", encoding="utf-8", buffering=1
                    )
                    created_paths.append(index_path)
                except BaseException:
                    data_stream.close()
                    raise
                self._views[view] = {
                    "data_path": data_path,
                    "index_path": index_path,
                    "data_stream": data_stream,
                    "index_stream": index_stream,
                    "frame_count": 0,
                    "data_bytes": 0,
                    "indexed_data_bytes": 0,
                    "index_bytes": 0,
                    "first_sequence": None,
                    "last_sequence": None,
                    "first_wall_time_ns": None,
                    "last_wall_time_ns": None,
                    "first_monotonic_ns": None,
                    "last_monotonic_ns": None,
                    "last_source": None,
                    "source_segments": [],
                    "source_signature": None,
                }
            if self._check_disk_locked(force=True, upcoming_bytes=0):
                self._write_manifest_locked()
        except BaseException:
            self._close_streams_locked(sync=False)
            self._lock_path.unlink(missing_ok=True)
            self._manifest_tmp.unlink(missing_ok=True)
            self.manifest_path.unlink(missing_ok=True)
            for path in created_paths:
                path.unlink(missing_ok=True)
            raise

    def _view_payload_locked(self, view: str) -> dict:
        state = self._views[view]
        try:
            stored_data_bytes = state["data_path"].stat().st_size
        except OSError:
            stored_data_bytes = state["data_bytes"]
        try:
            stored_index_bytes = state["index_path"].stat().st_size
        except OSError:
            stored_index_bytes = state["index_bytes"]
        orphan_tail_bytes = max(
            0, stored_data_bytes - state["indexed_data_bytes"]
        )
        return {
            "view": view,
            "data_file": state["data_path"].name,
            "index_file": state["index_path"].name,
            "format": "concatenated_jpeg_with_jsonl_index",
            "frame_count": state["frame_count"],
            "data_bytes": stored_data_bytes,
            "index_bytes": stored_index_bytes,
            "indexed_data_bytes": state["indexed_data_bytes"],
            "orphan_tail_bytes": orphan_tail_bytes,
            "index_authoritative": True,
            "first_sequence": state["first_sequence"],
            "last_sequence": state["last_sequence"],
            "first_wall_time_ns": state["first_wall_time_ns"],
            "last_wall_time_ns": state["last_wall_time_ns"],
            "first_monotonic_ns": state["first_monotonic_ns"],
            "last_monotonic_ns": state["last_monotonic_ns"],
            "last_source": copy.deepcopy(state["last_source"]),
            "source_segments": copy.deepcopy(state["source_segments"]),
        }

    def _payload_locked(self) -> dict:
        return {
            "schema_version": 1,
            "session_id": self.session_id,
            "directory": str(self.directory),
            "status": self.status_code,
            "active": self.active,
            "error": self.error,
            "created_wall_time_utc": _utc_iso(self.created_wall_time_ns),
            "created_wall_time_ns": self.created_wall_time_ns,
            "created_monotonic_ns": self.created_monotonic_ns,
            "stopped_wall_time_utc": (
                _utc_iso(self.stopped_wall_time_ns)
                if self.stopped_wall_time_ns is not None
                else None
            ),
            "stopped_wall_time_ns": self.stopped_wall_time_ns,
            "stopped_monotonic_ns": self.stopped_monotonic_ns,
            "disk_guard": {
                "min_free_bytes": self.min_free_bytes,
                "last_free_bytes": self.free_bytes,
                "estimated_free_bytes": (
                    max(0, self.free_bytes - self._bytes_since_disk_check)
                    if self.free_bytes is not None
                    else None
                ),
                "check_interval_s": self.disk_check_interval_ns / 1_000_000_000,
                "per_write_reserve_bytes": DISK_WRITE_RESERVE_BYTES,
            },
            "views": {
                view: self._view_payload_locked(view) for view in VIEWS
            },
        }

    def _write_manifest_locked(self) -> None:
        self._manifest_tmp.write_text(
            json.dumps(self._payload_locked(), ensure_ascii=False, indent=2)
            + "\n",
            encoding="utf-8",
        )
        self._manifest_tmp.replace(self.manifest_path)
        self._last_manifest_ns = time.monotonic_ns()

    def _close_streams_locked(self, *, sync: bool = True) -> list[str]:
        errors = []
        for state in self._views.values():
            for key in ("data_stream", "index_stream"):
                stream: BinaryIO | TextIO | None = state.get(key)
                if stream is None:
                    continue
                try:
                    stream.flush()
                    if sync:
                        os.fsync(stream.fileno())
                except (OSError, ValueError) as error:
                    errors.append(str(error))
                try:
                    stream.close()
                except (OSError, ValueError) as error:
                    errors.append(str(error))
                state[key] = None
        return errors

    def _finish_locked(self, status: str, error: str | None = None) -> None:
        if not self.active:
            return
        self.active = False
        self.status_code = status
        self.error = error
        self.stopped_wall_time_ns = time.time_ns()
        self.stopped_monotonic_ns = time.monotonic_ns()
        close_errors = self._close_streams_locked()
        if close_errors:
            suffix = "; ".join(close_errors)
            self.error = f"{self.error}; {suffix}" if self.error else suffix
            if status == "CLOSED":
                self.status_code = "ERROR"
        try:
            self._lock_path.unlink(missing_ok=True)
        except OSError as lock_error:
            self.error = (
                f"{self.error}; lock: {lock_error}"
                if self.error
                else f"lock: {lock_error}"
            )
        try:
            self._write_manifest_locked()
        except Exception as manifest_error:
            self.error = (
                f"{self.error}; manifest: {manifest_error}"
                if self.error
                else f"manifest: {manifest_error}"
            )

    def _check_disk_locked(self, *, force: bool, upcoming_bytes: int) -> bool:
        now = time.monotonic_ns()
        if (
            not force
            and self.disk_check_interval_ns > 0
            and now - self._last_disk_check_ns < self.disk_check_interval_ns
        ):
            estimated_free = (
                self.free_bytes - self._bytes_since_disk_check
                if self.free_bytes is not None
                else 0
            )
            required = (
                self.min_free_bytes
                + DISK_WRITE_RESERVE_BYTES
                + max(0, upcoming_bytes)
            )
            if estimated_free < required:
                self._finish_locked(
                    "STOPPED_LOW_DISK",
                    "estimated free disk space "
                    f"{max(0, estimated_free)} is below required {required}",
                )
                return False
            return True
        self._last_disk_check_ns = now
        try:
            self.free_bytes = int(self._disk_usage(self.directory).free)
        except (OSError, AttributeError, TypeError, ValueError) as error:
            self._finish_locked(
                "STOPPED_DISK_CHECK_ERROR",
                f"cannot verify free disk space: {error}",
            )
            return False
        self._bytes_since_disk_check = 0
        required = (
            self.min_free_bytes
            + DISK_WRITE_RESERVE_BYTES
            + max(0, upcoming_bytes)
        )
        if self.free_bytes < required:
            self._finish_locked(
                "STOPPED_LOW_DISK",
                f"free disk space {self.free_bytes} is below required {required}",
            )
            return False
        return True

    def _update_source_locked(
        self,
        state: dict,
        sequence: int,
        source: dict | str | None,
        wall_time_ns: int,
        monotonic_ns: int,
    ) -> None:
        signature = json.dumps(source, ensure_ascii=False, sort_keys=True)
        if signature != state["source_signature"]:
            state["source_segments"].append({
                "source": copy.deepcopy(source),
                "first_sequence": sequence,
                "last_sequence": sequence,
                "first_wall_time_ns": wall_time_ns,
                "last_wall_time_ns": wall_time_ns,
                "first_monotonic_ns": monotonic_ns,
                "last_monotonic_ns": monotonic_ns,
                "frame_count": 1,
            })
            state["source_signature"] = signature
        else:
            segment = state["source_segments"][-1]
            segment["last_sequence"] = sequence
            segment["last_wall_time_ns"] = wall_time_ns
            segment["last_monotonic_ns"] = monotonic_ns
            segment["frame_count"] += 1
        state["last_source"] = copy.deepcopy(source)

    def record_frame(
        self,
        view: str,
        sequence: int,
        jpeg: bytes,
        source: dict | str | None,
        *,
        wall_time_ns: int | None = None,
        monotonic_ns: int | None = None,
    ) -> bool:
        """Record one relay frame; return false after any protective stop."""
        if view not in VIEWS:
            raise ValueError(f"unsupported video view: {view}")
        wall_ns = time.time_ns() if wall_time_ns is None else wall_time_ns
        mono_ns = time.monotonic_ns() if monotonic_ns is None else monotonic_ns
        with self._lock:
            if not self.active:
                return False
            state = self._views[view]
            offset = state["data_bytes"]
            entry = {
                "session_id": self.session_id,
                "view": view,
                "sequence": sequence,
                "wall_time_utc": _utc_iso(wall_ns),
                "wall_time_ns": wall_ns,
                "monotonic_ns": mono_ns,
                "offset": offset,
                "length": len(jpeg),
                "source": copy.deepcopy(source),
            }
            try:
                index_line = json.dumps(
                    entry, ensure_ascii=False, separators=(",", ":")
                ) + "\n"
                index_size = len(index_line.encode("utf-8"))
                if not self._check_disk_locked(
                    force=False,
                    upcoming_bytes=len(jpeg) + index_size,
                ):
                    return False
                data_stream: BinaryIO = state["data_stream"]
                index_stream: TextIO = state["index_stream"]
                written = data_stream.write(jpeg)
                data_stream.flush()
                if written != len(jpeg):
                    raise OSError(
                        f"short MJPEG write: expected {len(jpeg)}, got {written}"
                    )
                state["data_bytes"] += written
                self._bytes_since_disk_check += written
                index_stream.write(index_line)
                index_stream.flush()
                state["index_bytes"] += index_size
                state["indexed_data_bytes"] = state["data_bytes"]
                self._bytes_since_disk_check += index_size
                state["frame_count"] += 1
                if state["first_sequence"] is None:
                    state["first_sequence"] = sequence
                    state["first_wall_time_ns"] = wall_ns
                    state["first_monotonic_ns"] = mono_ns
                state["last_sequence"] = sequence
                state["last_wall_time_ns"] = wall_ns
                state["last_monotonic_ns"] = mono_ns
                self._update_source_locked(
                    state, sequence, source, wall_ns, mono_ns
                )
                if (
                    self.manifest_interval_ns == 0
                    or mono_ns - self._last_manifest_ns
                    >= self.manifest_interval_ns
                ):
                    self._write_manifest_locked()
                return True
            except Exception as error:
                self._finish_locked(
                    "ERROR",
                    f"{error}; JSONL index is authoritative; ignore any "
                    "unindexed raw MJPEG tail",
                )
                return False

    def status(self, view: str | None = None) -> dict:
        with self._lock:
            payload = self._payload_locked()
            if view is None:
                return payload
            if view not in VIEWS:
                raise ValueError(f"unsupported video view: {view}")
            result = {
                key: payload[key]
                for key in (
                    "session_id", "directory", "status", "active", "error",
                    "created_wall_time_utc", "created_wall_time_ns",
                    "created_monotonic_ns", "stopped_wall_time_utc",
                    "stopped_wall_time_ns", "stopped_monotonic_ns",
                    "disk_guard",
                )
            }
            result.update(payload["views"][view])
            return result

    def close(self) -> None:
        with self._lock:
            if self.closed:
                return
            if self.active:
                self._finish_locked("CLOSED")
            else:
                # A prior low-disk/write stop may have been unable to replace
                # the manifest at that instant. Shutdown gets one final retry.
                try:
                    self._write_manifest_locked()
                except Exception as error:
                    self.error = (
                        f"{self.error}; final manifest: {error}"
                        if self.error
                        else f"final manifest: {error}"
                    )
            self.closed = True
