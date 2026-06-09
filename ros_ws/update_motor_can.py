#!/usr/bin/env python3
"""
update_motor_can.py — scan all active SocketCAN interfaces, discover motors of all
types (Robstride, Damiao, EZmotion), and update the channel fields in the config TOMLs.

Patched files and fields:
  src/robstride_p/config/*.toml  → [defaults] channel
  src/damiao_p/config/*.toml     → [MotorName] channel  (one entry per motor)
  src/ezmotion_p/config/*.toml   → [defaults] channel

Usage:
    python3 update_motor_can.py
    python3 update_motor_can.py --timeout 0.08 --dry-run
    python3 update_motor_can.py --interfaces can0 can2
"""

import argparse
import importlib.util
import re
import subprocess
import sys
import tomllib
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

HERE = Path(__file__).parent

sys.path.insert(0, str(HERE / 'src' / 'robstride_p'))
sys.path.insert(0, str(HERE / 'src' / 'damiao_p'))
sys.path.insert(0, str(HERE / 'src' / 'ezmotion_p'))


# ── Helpers ────────────────────────────────────────────────────────────────────

def _load_script(path: Path):
    spec = importlib.util.spec_from_file_location(path.stem, path)
    mod  = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def find_can_interfaces() -> List[str]:
    """Return names of all active SocketCAN interfaces, e.g. ['can0', 'can1']."""
    try:
        out = subprocess.check_output(['ip', '-o', 'link', 'show'], text=True)
    except (FileNotFoundError, subprocess.CalledProcessError) as e:
        sys.exit(f'Cannot list network interfaces: {e}')
    ifaces = []
    for line in out.splitlines():
        m = re.search(r'\d+:\s+(can\d+):', line)
        if m and 'UP' in line:
            ifaces.append(m.group(1))
    return sorted(ifaces)


# ── TOML channel patch ─────────────────────────────────────────────────────────

def _parse_section_channel(lines: List[str], section: str) -> Optional[str]:
    in_sec = False
    for line in lines:
        if re.match(r'^\[(?!\[)', line.strip()):
            m = re.match(r'^\[([^\]]+)\]', line.strip())
            if m:
                in_sec = m.group(1).strip() == section
        if in_sec:
            m = re.match(r'^\s*channel\s*=\s*"([^"]*)"', line)
            if m:
                return m.group(1)
    return None


def _patch_section_channel(lines: List[str], section: str, new_channel: str) -> Tuple[List[str], bool]:
    out, in_sec, replaced = [], False, False
    for line in lines:
        if re.match(r'^\[(?!\[)', line.strip()):
            m = re.match(r'^\[([^\]]+)\]', line.strip())
            if m:
                in_sec = m.group(1).strip() == section
        if in_sec and not replaced:
            m = re.match(r'^(\s*)channel\s*=\s*"[^"]*"', line)
            if m:
                out.append(f'{m.group(1)}channel = "{new_channel}"\n')
                replaced = True
                continue
        out.append(line)
    return out, replaced


def apply_channel_update(path: Path, section: str, new_channel: str, dry_run: bool) -> None:
    lines = path.read_text().splitlines(keepends=True)
    old   = _parse_section_channel(lines, section)

    if old is None:
        print(f'  [warn]  {path.name} [{section}]: no channel line found — skipping')
        return
    if old == new_channel:
        print(f'  [skip]  {path.name} [{section}].channel already = "{new_channel}"')
        return

    new_lines, replaced = _patch_section_channel(lines, section, new_channel)
    tag = 'dry ' if dry_run else 'upd '
    print(f'  [{tag}]  {path.name} [{section}].channel  "{old}" → "{new_channel}"')
    if not dry_run:
        path.write_text(''.join(new_lines))


# ── Config parsing ─────────────────────────────────────────────────────────────

def parse_robstride_config(path: Path) -> Dict:
    with open(path, 'rb') as f:
        cfg = tomllib.load(f)
    motors = {
        val['motor_id']: key
        for key, val in cfg.items()
        if key != 'defaults' and isinstance(val, dict) and 'motor_id' in val
    }
    return {'path': path, 'channel': cfg.get('defaults', {}).get('channel', ''), 'motors': motors}


def parse_damiao_config(path: Path) -> Dict:
    with open(path, 'rb') as f:
        cfg = tomllib.load(f)
    motors = {
        key: {'motor_id': val['motor_id'], 'channel': val.get('channel', '')}
        for key, val in cfg.items()
        if key != 'defaults' and isinstance(val, dict) and 'motor_id' in val
    }
    return {'path': path, 'motors': motors}


def parse_ezmotion_config(path: Path) -> Dict:
    with open(path, 'rb') as f:
        cfg = tomllib.load(f)
    motors = {
        val['node_id']: key
        for key, val in cfg.items()
        if key != 'defaults' and isinstance(val, dict) and 'node_id' in val
    }
    return {'path': path, 'channel': cfg.get('defaults', {}).get('channel', ''), 'motors': motors}


# ── Matching ───────────────────────────────────────────────────────────────────

def best_iface(wanted: Set[int], found_per_iface: Dict[str, Set[int]]) -> Optional[str]:
    """Interface with the most matches for the wanted ID set."""
    best_name, best_count = None, 0
    for iface, ids in found_per_iface.items():
        count = len(wanted & ids)
        if count > best_count:
            best_name, best_count = iface, count
    return best_name


# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Scan all SocketCAN interfaces and update motor config channel fields'
    )
    parser.add_argument('--timeout', type=float, default=0.05,
                        help='Per-motor response timeout in seconds (default: 0.05)')
    parser.add_argument('--dry-run', action='store_true',
                        help='Show what would change without writing files')
    parser.add_argument('--debug', action='store_true',
                        help='Print every discovered motor with its CAN bus and ID; do not update files')
    parser.add_argument('--interfaces', nargs='+', metavar='IF',
                        help='CAN interfaces to scan (default: all active canX interfaces)')
    args = parser.parse_args()

    interfaces = args.interfaces or find_can_interfaces()
    if not interfaces:
        sys.exit(
            'No active SocketCAN interfaces found.\n'
            'Bring them up first, e.g.:\n'
            '  sudo ip link set can0 up type can bitrate 1000000'
        )

    print(f'Interfaces : {interfaces}')
    print(f'Timeout    : {args.timeout * 1000:.0f} ms per motor')
    if args.debug:
        print('Mode       : debug (no files updated)\n')
    elif args.dry_run:
        print('Mode       : dry-run (no files updated)\n')
    else:
        print()

    _find_rs = _load_script(HERE / 'find_robstride_motors.py')
    _find_dm = _load_script(HERE / 'find_damiao_motors.py')
    _find_ez = _load_script(HERE / 'find_ezmotion_motors.py')

    # ── Scan all interfaces ────────────────────────────────────────────────────
    rs_results: Dict[str, List[int]]  = {}  # iface → [motor_id, ...]
    dm_results: Dict[str, List[Dict]] = {}  # iface → [{'motor_id', 'mst_id'}, ...]
    ez_results: Dict[str, List[int]]  = {}  # iface → [node_id, ...]

    for iface in interfaces:
        print(f'══ {iface} ══════════════════════')

        print(f'  Robstride  (IDs 0–127) …')
        try:
            rs_results[iface] = _find_rs.scan(iface, 0, 127, args.timeout)
        except Exception as e:
            print(f'  [error] Robstride scan: {e}')
            rs_results[iface] = []

        print(f'  Damiao     (IDs 0–15)  …')
        try:
            dm_results[iface] = _find_dm.scan(iface, args.timeout)
        except Exception as e:
            print(f'  [error] Damiao scan: {e}')
            dm_results[iface] = []

        print(f'  EZmotion   (IDs 1–127) …')
        try:
            ez_results[iface] = _find_ez.scan(iface, 1, 127, args.timeout)
        except Exception as e:
            print(f'  [error] EZmotion scan: {e}')
            ez_results[iface] = []

        total = len(rs_results[iface]) + len(dm_results[iface]) + len(ez_results[iface])
        print(f'  → {total} motor(s) found\n')

    # Flat sets per interface for matching
    rs_by_iface: Dict[str, Set[int]] = {k: set(v)                       for k, v in rs_results.items()}
    dm_by_iface: Dict[str, Set[int]] = {k: {m['motor_id'] for m in v}   for k, v in dm_results.items()}
    ez_by_iface: Dict[str, Set[int]] = {k: set(v)                       for k, v in ez_results.items()}

    # Damiao: motor_id → list of interfaces (motor_id is ESC_ID 0-15, may collide across buses)
    dm_id_to_ifaces: Dict[int, List[str]] = {}
    for iface, motors in dm_results.items():
        for m in motors:
            dm_id_to_ifaces.setdefault(m['motor_id'], []).append(iface)

    # ── Debug: print every discovered motor and stop ──────────────────────────
    if args.debug:
        print('══ Discovered motors ═════════════')
        total = 0
        for iface in interfaces:
            for mid in sorted(rs_by_iface.get(iface, [])):
                print(f'  robstride   {iface}  motor_id={mid}')
                total += 1
            for m in sorted(dm_results.get(iface, []), key=lambda x: x['motor_id']):
                print(f'  damiao      {iface}  motor_id={m["motor_id"]}  mst_id=0x{m["mst_id"]:02X}')
                total += 1
            for nid in sorted(ez_by_iface.get(iface, [])):
                print(f'  ezmotion    {iface}  node_id={nid}')
                total += 1
        print(f'\n{total} motor(s) found total.')
        return

    # ── Update configs ─────────────────────────────────────────────────────────
    print('══ Updating configs ══════════════')

    # Robstride
    for toml_path in sorted((HERE / 'src' / 'robstride_p' / 'config').glob('*.toml')):
        info = parse_robstride_config(toml_path)
        if not info['motors']:
            continue
        wanted = set(info['motors'])
        iface  = best_iface(wanted, rs_by_iface)
        if iface is None:
            print(f'  [warn]  {toml_path.name}: no Robstride motors found on any interface — skipping')
            continue
        missing = wanted - rs_by_iface[iface]
        if missing:
            missing_names = [info['motors'][m] for m in sorted(missing)]
            print(f'  [warn]  {toml_path.name}: {len(wanted)-len(missing)}/{len(wanted)} motors '
                  f'found on {iface} (missing: {missing_names})')
        apply_channel_update(toml_path, 'defaults', iface, args.dry_run)

    # Damiao (per-motor channel)
    for toml_path in sorted((HERE / 'src' / 'damiao_p' / 'config').glob('*.toml')):
        info = parse_damiao_config(toml_path)
        for motor_name, minfo in info['motors'].items():
            mid    = minfo['motor_id']
            ifaces = dm_id_to_ifaces.get(mid, [])
            if not ifaces:
                print(f'  [warn]  {toml_path.name} [{motor_name}]: motor_id={mid} '
                      f'not found on any interface — skipping')
                continue
            if len(ifaces) > 1:
                print(f'  [warn]  {toml_path.name} [{motor_name}]: motor_id={mid} '
                      f'found on {ifaces} — using {ifaces[0]}')
            apply_channel_update(toml_path, motor_name, ifaces[0], args.dry_run)

    # EZmotion
    for toml_path in sorted((HERE / 'src' / 'ezmotion_p' / 'config').glob('*.toml')):
        info = parse_ezmotion_config(toml_path)
        if not info['motors']:
            continue
        wanted = set(info['motors'])
        iface  = best_iface(wanted, ez_by_iface)
        if iface is None:
            print(f'  [warn]  {toml_path.name}: no EZmotion motors found on any interface — skipping')
            continue
        missing = wanted - ez_by_iface[iface]
        if missing:
            missing_names = [info['motors'][m] for m in sorted(missing)]
            print(f'  [warn]  {toml_path.name}: {len(wanted)-len(missing)}/{len(wanted)} motors '
                  f'found on {iface} (missing: {missing_names})')
        apply_channel_update(toml_path, 'defaults', iface, args.dry_run)

    print('\nDone.')


if __name__ == '__main__':
    main()
