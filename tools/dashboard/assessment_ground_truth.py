"""Read-only scenario inventory and frame-local GT completeness checks.

Names are exact VTD identifiers, not stripped/truncated aliases. A complete
inventory is evidence for this scenario/frame only, never a global assertion
that every possible simulator object or collision mesh was observed.
"""
from collections import Counter
import hashlib
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def valid_geometry(obj):
    required = ('x', 'y', 'z', 'h', 'length', 'width', 'height')
    optional = ('p', 'r', 'off_x', 'off_y', 'off_z')
    return (all(isinstance(obj.get(k), (float, int)) and math.isfinite(obj[k])
                for k in required)
            and all(isinstance(obj.get(k, 0.), (float, int)) and math.isfinite(obj.get(k, 0.))
                    for k in optional)
            and all(obj[k] > 0 for k in ('length', 'width', 'height')))


class ScenarioGroundTruth:
    def __init__(self, scenario_file):
        self.path = Path(scenario_file).resolve()
        raw = self.path.read_bytes()
        root = ET.fromstring(raw)
        self.sha256 = hashlib.sha256(raw).hexdigest()
        self.roles = {}
        declarations = []
        for node in root.findall('.//Player/Description'):
            declarations.append((node.get('Name'), 'road_user'))
        for node in root.findall('.//Character'):
            kind = (node.get('CharacterType') or '').lower()
            role = 'cyclist' if 'bicycle' in kind else 'pedestrian'
            declarations.append((node.get('Name'), role))
        for node in root.findall('.//Object'):
            declarations.append((node.get('Name'), 'static_obstacle'))
        counts = Counter(name for name, _ in declarations if name)
        if not counts or any(not name for name, _ in declarations) or any(count != 1 for count in counts.values()):
            raise ValueError('scenario inventory empty or duplicate entity names')
        self.roles = {name: role for name, role in declarations if name}

    def describe(self):
        return dict(source='scenario_entity_inventory', path=str(self.path),
                    sha256=self.sha256, expected_entities=len(self.roles),
                    roles=dict(self.roles))

    def inspect(self, frame):
        objects = list(frame.get('objects', {}).values())
        counts = Counter(o.get('name') for o in objects)
        missing = sorted(set(self.roles) - set(counts))
        duplicate = sorted(n for n in self.roles if counts[n] > 1)
        invalid = []
        for obj in objects:
            # All observed actors can interact with Ego, including dynamically
            # added actors outside the scenario declaration.
            if obj.get('category') not in {1, 5, 6}:
                if obj.get('name') in self.roles:
                    invalid.append(obj.get('name'))
                continue
            if not valid_geometry(obj):
                invalid.append(obj.get('name') or str(obj.get('id')))
        complete = not missing and not duplicate and not invalid
        return dict(complete=complete, source='scenario_inventory_per_rdb_frame',
                    expected=len(self.roles), observed=len(objects), missing=missing,
                    duplicate=duplicate, invalid_geometry=invalid,
                    extra=sorted(str(n) for n in set(counts) - set(self.roles)),
                    scenario_sha256=self.sha256)


def _box3d(obj):
    h, p, r = obj['h'], obj.get('p', 0.), obj.get('r', 0.)
    ch, sh, cp, sp, cr, sr = math.cos(h), math.sin(h), math.cos(p), math.sin(p), math.cos(r), math.sin(r)
    axes = ((ch*cp, sh*cp, -sp),
            (ch*sp*sr-sh*cr, sh*sp*sr+ch*cr, cp*sr),
            (ch*sp*cr+sh*sr, sh*sp*cr-ch*sr, cp*cr))
    offsets = (obj.get('off_x', 0.), obj.get('off_y', 0.), obj.get('off_z', 0.))
    centre = tuple(obj[k] + sum(axes[j][i]*offsets[j] for j in range(3))
                   for i, k in enumerate(('x', 'y', 'z')))
    return centre, axes, (obj['length']/2, obj['width']/2, obj['height']/2)


def vertical_overlap(first, second):
    """Reject actors on disjoint vertical levels before 2D corridor tests."""
    if not valid_geometry(first) or not valid_geometry(second):
        return False
    a, aa, ah = _box3d(first)
    b, ba, bh = _box3d(second)
    extent = sum(abs(axis[2])*half for axis, half in zip(aa, ah))
    extent += sum(abs(axis[2])*half for axis, half in zip(ba, bh))
    return abs(a[2]-b[2]) <= extent+1e-9


def obb_overlap_3d(first, second):
    """Exact separating-axis test of two GT oriented boxes, including offsets.

This is the configured OBB scoring primitive, not a rendered-mesh collision
event. Missing roll/pitch in legacy explicitly supplied fixtures means zero.
"""
    if not valid_geometry(first) or not valid_geometry(second):
        raise ValueError('invalid RDB oriented-box geometry')
    def dot(a, b):
        return sum(x*y for x, y in zip(a, b))
    ca, aa, ha = _box3d(first)
    cb, ab, hb = _box3d(second)
    delta = tuple(b-a for a, b in zip(ca, cb))
    axes = list(aa) + list(ab)
    axes.extend((a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
                for a in aa for b in ab)
    for axis in axes:
        norm = math.sqrt(dot(axis, axis))
        if norm < 1e-10:
            continue
        axis = tuple(v/norm for v in axis)
        ra = sum(half*abs(dot(axis, direction)) for half, direction in zip(ha, aa))
        rb = sum(half*abs(dot(axis, direction)) for half, direction in zip(hb, ab))
        if abs(dot(delta, axis)) > ra + rb + 1e-9:
            return False
    return True
