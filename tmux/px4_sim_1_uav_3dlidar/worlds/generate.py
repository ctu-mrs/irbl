#!/usr/bin/env python3
"""Generate gz worlds for this session from the generators in worldgen/worlds/.

  ./generate.py --list                       # available worlds + tunable params
  ./generate.py forest                       # -> forest.sdf (default seed), goal -> world_goals.yaml
  ./generate.py city --seed 7                # different random layout, same goal
  ./generate.py forest --set density=0.09    # override a generator param
  ./generate.py --all                        # regenerate every world
  ./generate.py forest --name forest_s7 --seed 7   # extra variant next to the default one
  ./generate.py warehouse --stdout           # print instead of writing

Output goes next to this script; session.yml symlinks worlds/*.sdf into PX4 on start, so a new
world is usable right away with ./start.sh <name>, and activation.sh reads its goal from
world_goals.yaml.
"""
import argparse
import ast
import os
import random
import re
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from worldgen import discover  # noqa: E402

GOALS_FILE = os.path.join(HERE, "world_goals.yaml")


def parse_set(items):
    out = {}
    for item in items:
        key, _, val = item.partition("=")
        try:
            out[key] = ast.literal_eval(val)
        except (ValueError, SyntaxError):
            out[key] = val
    return out


def save_goal(name, goal):
    goals = {}
    if os.path.exists(GOALS_FILE):
        with open(GOALS_FILE) as f:
            goals = yaml.safe_load(f) or {}
    goals[name] = [float(v) for v in goal]
    with open(GOALS_FILE, "w") as f:
        f.write("# world name -> rbl_controller goal [x, y, z, yaw], read by ../activation.sh.\n"
                "# Written by generate.py; worlds not listed here (e.g. PX4's own) fall back to the\n"
                "# default goal in activation.sh.\n")
        yaml.safe_dump(dict(sorted(goals.items())), f, default_flow_style=None)


def main():
    worlds = discover()
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("worlds", nargs="*", help="world(s) to generate")
    ap.add_argument("--all", action="store_true", help="generate every registered world")
    ap.add_argument("--list", action="store_true", help="list registered worlds and their params")
    ap.add_argument("--seed", type=int, help="random seed (default: the world's own)")
    ap.add_argument("--set", nargs="*", default=[], metavar="KEY=VALUE", help="override generator params")
    ap.add_argument("--name", help="output world name (single world only), for variants")
    ap.add_argument("--stdout", action="store_true", help="print the sdf instead of writing it")
    args = ap.parse_args()

    if args.list:
        for w in worlds.values():
            print(f"{w.name:12s} {w.description}")
            print(f"{'':12s} goal={list(w.goal)}  seed={w.default_seed}  params={w.params}")
        return

    names = list(worlds) if args.all else args.worlds
    if not names:
        ap.error("give world name(s), --all or --list")
    unknown = [n for n in names if n not in worlds]
    if unknown:
        ap.error(f"unknown world(s) {unknown}; available: {list(worlds)}")
    if args.name and len(names) != 1:
        ap.error("--name needs exactly one world")

    overrides = parse_set(args.set)
    for n in names:
        w = worlds[n]
        bad = set(overrides) - set(w.params)
        if bad:
            ap.error(f"{n}: unknown param(s) {sorted(bad)}; available: {sorted(w.params)}")
        params = {**w.params, **overrides}
        seed = w.default_seed if args.seed is None else args.seed
        sdf = w.build(random.Random(seed), params)
        out_name = args.name or w.name
        if out_name != w.name:
            # gz topics are /world/<name>/..., and the session derives them from the file name
            sdf = re.sub(r'<world name="[^"]*">', f'<world name="{out_name}">', sdf, count=1)
        if args.stdout:
            sys.stdout.write(sdf)
            continue
        path = os.path.join(HERE, f"{out_name}.sdf")
        with open(path, "w") as f:
            f.write(sdf)
        save_goal(out_name, w.goal)
        print(f"{out_name}: wrote {os.path.relpath(path)} ({sdf.count('<model ')} models, seed {seed})",
              file=sys.stderr)


if __name__ == "__main__":
    main()
