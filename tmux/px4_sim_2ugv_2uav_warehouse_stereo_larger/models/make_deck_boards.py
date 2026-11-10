#!/usr/bin/env python3
"""Generate the ArUco landing boards on the UGV decks (models/ugv<k>_carrier_stereo/deck_board_ugv<k>.png).

Board (DICT_4X4_50), 0.36 x 0.36 m white plate inside the 0.40 m deck's 0.36 m inner area, origin = deck
centre, x right / y up of the image:
  - one small centre marker (CENTRE_SIZE): stays fully in the down camera's view until touchdown
  - four CORNER_SIZE markers centred at (+-CORNER_OFFSET, +-CORNER_OFFSET): detection from the
    rendezvous height, 16 extra corners for the pose
Ids: pair k uses 5*(k-1) .. 5*(k-1)+4 (centre first, then top-left, top-right, bottom-right,
bottom-left), so each UAV only locks onto its own UGV.

The detector (rbl_mission aruco_landing_target.py, board_* parameters in session.yml) must use the
same layout: print_params() below prints it.
"""

import os

import cv2
import numpy as np

PLATE = 0.36          # [m]
CENTRE_SIZE = 0.05    # [m]
CORNER_SIZE = 0.11    # [m]  (~28 px at 1.5 m with the 80 deg 640 px down camera)
CORNER_OFFSET = 0.095 # [m]  (1.5 cm white gap to the centre marker, 3 cm to the plate edge)
PX = 720              # texture resolution (2000 px/m)


def layout(pair):
    base = 5 * (pair - 1)
    o = CORNER_OFFSET
    return [
        (base + 0, CENTRE_SIZE, 0.0, 0.0),
        (base + 1, CORNER_SIZE, -o, o),
        (base + 2, CORNER_SIZE, o, o),
        (base + 3, CORNER_SIZE, o, -o),
        (base + 4, CORNER_SIZE, -o, -o),
    ]


def make(pair, path):
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    gen = cv2.aruco.generateImageMarker if hasattr(cv2.aruco, 'generateImageMarker') else cv2.aruco.drawMarker
    img = np.full((PX, PX), 255, np.uint8)
    ppm = PX / PLATE
    for mid, size, cx, cy in layout(pair):
        n = int(round(size * ppm))
        m = cv2.resize(gen(d, mid, 600), (n, n), interpolation=cv2.INTER_NEAREST)
        c0 = int(round((PLATE / 2 + cx) * ppm - n / 2))
        r0 = int(round((PLATE / 2 - cy) * ppm - n / 2))  # image rows go down, y up
        img[r0:r0 + n, c0:c0 + n] = m
    cv2.imwrite(path, img)
    # self-check: every marker decodes
    ids = cv2.aruco.ArucoDetector(d).detectMarkers(img)[1] if hasattr(cv2.aruco, 'ArucoDetector') \
        else cv2.aruco.detectMarkers(img, d)[1]
    found = sorted(ids.ravel().tolist()) if ids is not None else []
    assert found == sorted(m[0] for m in layout(pair)), found
    print(f'{path}: ids {found}')


def print_params(pair):
    lay = layout(pair)
    print(f'  pair {pair}: -p "board_ids:={[m[0] for m in lay]}" -p "board_sizes:={[m[1] for m in lay]}" '
          f'-p "board_centers:={[v for m in lay for v in m[2:]]}"')


if __name__ == '__main__':
    here = os.path.dirname(os.path.abspath(__file__))
    for pair in (1, 2):
        make(pair, os.path.join(here, f'ugv{pair}_carrier_stereo', f'deck_board_ugv{pair}.png'))
        print_params(pair)
