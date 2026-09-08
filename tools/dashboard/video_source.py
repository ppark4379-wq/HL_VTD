"""Discover VTD render windows and build read-only ffmpeg inputs."""
from __future__ import annotations

import os
import re
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional, Sequence, Tuple

import yaml


DEFAULT_WINDOW_TITLES = ('igVisualBroadcast', 'cam1', 'mainRS', 'birdRS')

_WINDOW_LINE_RE = re.compile(
    r'(?P<window_id>0x[0-9a-fA-F]+)\s+"(?P<title>[^"]*)":\s+\("(?P<class1>[^"]*)"\s+"(?P<class2>[^"]*)"\)\s+'
    r'(?P<width>\d+)x(?P<height>\d+)\+-?\d+\+-?\d+'
    r'(?:\s+\+(?P<absx>-?\d+)\+(?P<absy>-?\d+))?'
)


@dataclass(frozen=True)
class VideoSourceConfig:
    mode: str
    rtsp_url: Optional[str]
    framerate: int


def find_mainrs_window(titles: Sequence[str] = DEFAULT_WINDOW_TITLES) -> Optional[dict]:
    """Return the preferred VTD OSG render surface from the X11 window tree."""
    try:
        result = subprocess.run(
            ['xwininfo', '-root', '-tree'],
            capture_output=True,
            text=True,
            timeout=3,
            check=True,
        )
    except (OSError, subprocess.CalledProcessError, subprocess.TimeoutExpired):
        return None
    candidates = []
    for line in result.stdout.splitlines():
        match = _WINDOW_LINE_RE.search(line)
        if not match or not any(
            title.lower() in match.group('title').lower() for title in titles
        ):
            continue
        width = int(match.group('width'))
        height = int(match.group('height'))
        is_render_surface = (
            'osg' in match.group('class1').lower()
            or 'osg' in match.group('class2').lower()
        )
        candidates.append({
            'window_id': int(match.group('window_id'), 16),
            'width': width,
            'height': height,
            'x': int(match.group('absx')) if match.group('absx') else 0,
            'y': int(match.group('absy')) if match.group('absy') else 0,
            'is_render_surface': is_render_surface,
            'title_rank': next(
                index for index, title in enumerate(titles)
                if title.lower() in match.group('title').lower()
            ),
        })
    if not candidates:
        return None
    candidates.sort(
        key=lambda candidate: (
            not candidate['is_render_surface'],
            candidate['title_rank'],
            -candidate['width'] * candidate['height'],
        )
    )
    return candidates[0]


def configured_rtsp_url(network_config_path: Path) -> Optional[str]:
    """Build the VTD broadcast RTSP URL from network.yaml."""
    try:
        config = yaml.safe_load(network_config_path.read_text(encoding='utf-8')) or {}
        vtd = config.get('vtd', {})
        host = str(vtd.get('host_ip', '')).strip()
        port = int(vtd.get('rtsp_port', 8554))
    except (OSError, TypeError, ValueError, yaml.YAMLError):
        return None
    if not host:
        return None
    return f'rtsp://{host}:{port}/broadcast'


def local_window_command(window: dict, framerate: int) -> list:
    """Build the exact ffmpeg command for an X11 VTD render surface."""
    display = os.environ.get('DISPLAY', ':0')
    return [
        'ffmpeg', '-hide_banner', '-loglevel', 'error',
        '-f', 'x11grab',
        '-window_id', str(window['window_id']),
        '-video_size', f"{window['width']}x{window['height']}",
        '-framerate', str(framerate),
        '-i', display,
        '-an', '-f', 'mjpeg', '-q:v', '7', 'pipe:1',
    ]


def rtsp_command(url: str, framerate: int) -> list:
    """Build the exact ffmpeg command for the VTD RTSP broadcast."""
    return [
        'ffmpeg', '-hide_banner', '-loglevel', 'error',
        '-rtsp_transport', 'tcp',
        '-rw_timeout', '5000000',
        '-i', url,
        '-an', '-vf', f'fps={framerate}',
        '-f', 'mjpeg', '-q:v', '7', 'pipe:1',
    ]


def video_capture_command(
    config: VideoSourceConfig,
    window_finder: Callable[[], Optional[dict]] = find_mainrs_window,
) -> Tuple[Optional[list], Optional[str], Optional[str]]:
    """Return the ffmpeg command, source label, and configuration error."""
    if shutil.which('ffmpeg') is None:
        return None, None, 'ffmpeg is not installed'
    if config.mode in {'auto', 'window'}:
        window = window_finder()
        if window is not None:
            return local_window_command(window, config.framerate), 'local VTD window', None
        if config.mode == 'window':
            return None, None, 'mainRS/birdRS window not found'
    if config.mode in {'auto', 'rtsp'} and config.rtsp_url:
        return rtsp_command(config.rtsp_url, config.framerate), config.rtsp_url, None
    return None, None, 'VTD video source is not configured'
