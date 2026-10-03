#!/usr/bin/env python3
"""Kept for backwards compatibility: the warehouse generator now lives in worldgen/worlds/warehouse.py.

Usage: ./generate_warehouse.py [--seed N] > warehouse.sdf   (same as ./generate.py warehouse --stdout)
"""
import os
import sys

here = os.path.dirname(os.path.abspath(__file__))
os.execv(sys.executable, [sys.executable, os.path.join(here, "generate.py"), "warehouse", "--stdout", *sys.argv[1:]])
