"""Shared type contracts for the passive VTD assessment pipeline.

The runtime representation intentionally remains ``dict``.  TypedDict makes
the wire and scoring contracts visible to editors and type checkers without
changing JSON serialization, equality, or the values consumed by the UI.
"""
from __future__ import annotations

from typing import Any, Dict, List, Tuple, TypedDict


Point2D = Tuple[float, float]


class EgoState(TypedDict, total=False):
    id: int
    name: str
    category: int
    type: int
    x: float
    y: float
    z: float
    h: float
    speed: float
    speed_source: str
    length: float
    width: float
    height: float
    off_x: float
    off_y: float
    off_z: float


class RoadPosition(TypedDict, total=False):
    road: int
    lane: int
    s: float
    t: float
    heading: float


class FrameData(TypedDict, total=False):
    frame: int
    time: float
    ego: EgoState
    objects: Dict[int, EgoState]
    road: Dict[int, RoadPosition]
    lanes: Dict[tuple, Dict[str, Any]]
    lights: Dict[int, str]
    signals: Dict[int, Dict[str, Any]]


class ViolationResult(TypedDict, total=False):
    event: str
    active: bool
    source: str
    evidence: Dict[str, Any]


class AssessmentState(TypedDict, total=False):
    status: str
    session_id: str
    section: str
    distance_m: float
    rows: List[Dict[str, str]]
    events: List[Dict[str, Any]]
    deductions: List[Dict[str, Any]]


class AssessmentSnapshot(TypedDict, total=False):
    session_id: str
    status: str
    unsegmented_frames: bool
    sections: List[str]
    reached_sections: List[str]
    official_grid: List[Dict[str, Any]]
    summary: Dict[str, Any]
    latest: Dict[str, str]
    time_limit_s: float
    events: List[Dict[str, Any]]
    deductions: List[Dict[str, Any]]
    missing_metadata: List[str]
    warnings: List[str]


class RunPayload(TypedDict, total=False):
    scenario_id: str
    run_id: str
    frame_count: int
    sections: List[str]
    official_grid: List[Dict[str, Any]]
    summary: Dict[str, str]
    latest: Dict[str, Any]
    track: List[Dict[str, Any]]
