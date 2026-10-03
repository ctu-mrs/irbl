"""World registry. Every module in worldgen/worlds/ that defines a module-level `WORLD = World(...)`
is picked up automatically -- adding a world means adding one file there (see worlds/README.md)."""
import importlib
import pkgutil
from dataclasses import dataclass, field
from typing import Callable


@dataclass
class World:
    name: str                  # also the gz <world name> and the .sdf file name (must match!)
    description: str
    goal: tuple                # (x, y, z, yaw) handed to rbl_controller/goto by activation.sh
    build: Callable            # build(rng, params) -> sdf string (header + models + footer)
    params: dict = field(default_factory=dict)  # tunables, overridable with --set key=value
    default_seed: int = 1


def discover():
    from . import worlds as pkg
    found = {}
    for mod in pkgutil.iter_modules(pkg.__path__):
        if mod.name.startswith("_"):
            continue
        w = getattr(importlib.import_module(f"{pkg.__name__}.{mod.name}"), "WORLD", None)
        if isinstance(w, World):
            found[w.name] = w
    return dict(sorted(found.items()))
