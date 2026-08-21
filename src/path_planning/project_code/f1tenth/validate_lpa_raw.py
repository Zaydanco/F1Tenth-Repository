#!/usr/bin/env python3

import csv
import sys
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT))

from autodrive_common import (
    load_autodrive_map,
    grid_to_image,
)


INFLATION_RADIUS_PX = 2

map_data = load_autodrive_map(
    inflation_radius_px=INFLATION_RADIUS_PX
)

csv_path = HERE / "lpa_autodrive_raw.csv"

path = []

with csv_path.open("r", encoding="utf-8") as file:
    reader = csv.DictReader(file)

    for row in reader:
        path.append(
            (
                int(row["grid_x"]),
                int(row["grid_y"]),
            )
        )


# Distancia desde cada celda libre hasta el obstáculo más cercano.
free_mask = (
    (~map_data.blocked).astype(np.uint8)
)

distance_map = cv2.distanceTransform(
    free_mask,
    cv2.DIST_L2,
    5,
)


minimum_clearance_px = float("inf")
bad_steps = 0


for index, point in enumerate(path):

    px, py = grid_to_image(
        map_data,
        point,
    )

    if map_data.blocked[py, px]:
        raise RuntimeError(
            f"Nodo {index} está en obstáculo: {point}"
        )

    clearance = float(
        distance_map[py, px]
    )

    minimum_clearance_px = min(
        minimum_clearance_px,
        clearance,
    )


for p1, p2 in zip(path[:-1], path[1:]):

    dx = abs(p2[0] - p1[0])
    dy = abs(p2[1] - p1[1])

    if dx > 1 or dy > 1:
        bad_steps += 1


minimum_clearance_m = (
    minimum_clearance_px
    * map_data.resolution
)


print("========== VALIDACIÓN LPA* ==========")
print(f"Nodos: {len(path)}")
print(f"Saltos inválidos: {bad_steps}")
print(
    f"Distancia mínima a obstáculos: "
    f"{minimum_clearance_px:.2f} px"
)
print(
    f"Distancia mínima equivalente: "
    f"{minimum_clearance_m:.3f} m"
)

if bad_steps == 0:
    print("Continuidad Grid: OK")

print("Trayectoria cruda: VÁLIDA")
