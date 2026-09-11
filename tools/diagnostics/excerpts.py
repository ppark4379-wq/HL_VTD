"""Select whole evidence frames; never discard incident anchors for a size cap."""

from __future__ import annotations

from collections import Counter, deque
import json
from pathlib import Path
import re

from analyze import finite, records
from contracts import ExcerptAnchor, FrameEvidence, Incident, PackRequest, ScanResult


EVIDENCE_KINDS = {"input", "frame", "send_result", "stage"}
DECISION_PATTERN = re.compile(r"frame=(\d+) \[([^]]+)\] ([^ |]+)")


def excerpt_anchors(scan: ScanResult, selected: list[Incident]) -> list[ExcerptAnchor]:
    anchors = []
    for item in selected:
        anchors.extend((ExcerptAnchor(item.first_frame, item.first_wall_s),
                        ExcerptAnchor(item.last_frame, item.last_wall_s)))
    for row in (scan.first, scan.last):
        if isinstance(row.get("frame"), int) and finite(row.get("process_wall_s")) is not None:
            anchors.append(ExcerptAnchor(row["frame"], row["process_wall_s"], False))
    return list(dict.fromkeys(anchors))


def index_evidence_frames(request: PackRequest, anchors: list[ExcerptAnchor]) -> dict[int, FrameEvidence]:
    # Metadata only: raw object arrays stay on disk, even for long runs.
    index = {}
    for line, row, error in records(request.directory / "diagnostics.jsonl"):
        if error or row.get("kind") not in EVIDENCE_KINDS:
            continue
        frame = row.get("frame")
        if not isinstance(frame, int) or isinstance(frame, bool) or frame < 0:
            continue
        group = index.setdefault(frame, FrameEvidence(frame, line))
        group.records += 1
        now = finite(row.get("process_wall_s"))
        if now is None:
            continue
        for anchor in anchors:
            if anchor.wall_s - request.context_seconds <= now <= anchor.wall_s + 2.0:
                group.candidate = True
                if anchor.incident and not group.near_incident:
                    group.near_incident = True
                    group.distance = float("inf")
                if anchor.incident == group.near_incident:
                    group.distance = min(group.distance, abs(now - anchor.wall_s))
    return index


def protected_frame_ids(index: dict[int, FrameEvidence], anchors: list[ExcerptAnchor]) -> set[int]:
    ordered = list(index)
    positions = {frame: position for position, frame in enumerate(ordered)}
    important = [anchor for anchor in anchors if anchor.incident] or anchors
    protected = set()
    for anchor in important:
        position = positions.get(anchor.frame)
        if position is not None:
            # Nearest actually recorded neighbours, not fabricated frame +/- 1.
            protected.update(ordered[max(0, position - 1):position + 2])
    return protected


def choose_frame_ids(index: dict[int, FrameEvidence], protected: set[int], budget: int) -> set[int]:
    included = set(protected)
    used = sum(index[frame].records for frame in included)
    candidates = sorted((not group.near_incident, group.distance, group.first_line, frame)
                        for frame, group in index.items() if group.candidate and frame not in included)
    for _, _, _, frame in candidates:
        count = index[frame].records
        if used + count <= budget:
            included.add(frame)
            used += count
    return included


def write_frame_excerpt(request: PackRequest, scan: ScanResult, selected: list[Incident]) -> tuple[set[int], dict]:
    anchors = excerpt_anchors(scan, selected)
    index = index_evidence_frames(request, anchors)
    protected = protected_frame_ids(index, anchors)
    included = choose_frame_ids(index, protected, request.max_excerpt_records)
    count = 0
    with (request.directory / "focus_frames.jsonl").open("w", encoding="utf-8") as stream:
        for _, row, error in records(request.directory / "diagnostics.jsonl"):
            if error or row.get("kind") not in EVIDENCE_KINDS or row.get("frame") not in included:
                continue
            stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n")
            count += 1
    candidates = sum(group.records for frame, group in index.items() if group.candidate or frame in protected)
    return included, {
        "selection_policy": "incident_start_end_and_adjacent_frames_first_whole_frame_groups",
        "candidate_records": candidates, "included_records": count, "truncated": candidates > count,
        "included_frames": sorted(included), "protected_frames": sorted(protected),
        "missing_anchor_frames": sorted({anchor.frame for anchor in anchors if anchor.incident and anchor.frame not in index}),
        "record_budget": request.max_excerpt_records, "record_budget_exceeded": count > request.max_excerpt_records,
        "full_file": "diagnostics.jsonl",
    }


def optional_decision_lines(first: list[int], last: deque[int], available: int) -> set[int]:
    if available <= 0:
        return set()
    candidates = list(dict.fromkeys([*first, *last]))
    if len(candidates) <= available:
        return set(candidates)
    head = (available + 1) // 2
    tail = available // 2
    return set(candidates[:head] + (candidates[-tail:] if tail else []))


def write_decision_excerpt(directory: Path, frames: set[int], protected_frames: set[int] | None = None) -> dict:
    path = directory / "repair_decision.log"
    if not path.is_file():
        path = directory / "decision.log"
    protected = frames if protected_frames is None else frames & protected_frames
    maximum = 200
    counts = Counter()
    first = []
    last = deque(maxlen=maximum)
    matched = protected_lines = 0
    observed_frames = set()
    if path.is_file():
        with path.open(encoding="utf-8", errors="replace") as stream:
            for line_no, line in enumerate(stream, 1):
                match = DECISION_PATTERN.search(line)
                if not match:
                    continue
                counts[match[2] + "/" + match[3]] += 1
                frame = int(match[1])
                if frame not in frames:
                    continue
                matched += 1
                observed_frames.add(frame)
                if frame in protected:
                    protected_lines += 1
                elif len(first) < maximum:
                    first.append(line_no)
                else:
                    last.append(line_no)
    optional = optional_decision_lines(first, last, maximum - protected_lines)
    written = 0
    with (directory / "focus_decisions.log").open("w", encoding="utf-8") as output:
        if path.is_file():
            with path.open(encoding="utf-8", errors="replace") as stream:
                for line_no, line in enumerate(stream, 1):
                    match = DECISION_PATTERN.search(line)
                    if match and (int(match[1]) in protected or line_no in optional):
                        output.write(f"{path.name}:{line_no}: {line.rstrip()}\n")
                        written += 1
    return {"event_counts": dict(counts.most_common()), "matched_lines": matched,
            "included_lines": written, "truncated": matched > written, "full_file": path.name,
            "protected_lines": protected_lines, "line_budget": maximum,
            "line_budget_exceeded": written > maximum,
            "protected_frames_without_events": sorted(protected - observed_frames)}
