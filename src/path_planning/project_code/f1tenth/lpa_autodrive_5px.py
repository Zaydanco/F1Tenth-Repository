#!/usr/bin/env python3

import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent

sys.path.insert(0, str(ROOT))

from f1tenth.autodrive_common import (
    load_autodrive_map,
    grid_from_map,
    image_to_grid,
    grid_to_image,
    grid_to_world,
)

from python_motion_planning.global_planner.graph_search.lpa_star import (
    LPAStar,
)


# ============================================================
# Configuración
# ============================================================

INFLATION_RADIUS_PX = 5

START_PIXEL = (90, 139)
GOAL_PIXEL = (91, 144)


# ============================================================
# Cargar mapa
# ============================================================

map_data = load_autodrive_map(
    inflation_radius_px=INFLATION_RADIUS_PX
)

start_grid = image_to_grid(
    map_data,
    START_PIXEL,
)

goal_grid = image_to_grid(
    map_data,
    GOAL_PIXEL,
)


def pixel_is_free(pixel):
    x, y = pixel

    if not (
        0 <= x < map_data.width
        and 0 <= y < map_data.height
    ):
        return False

    return not bool(map_data.blocked[y, x])


if not pixel_is_free(START_PIXEL):
    raise RuntimeError(
        f"START {START_PIXEL} no está en espacio libre."
    )

if not pixel_is_free(GOAL_PIXEL):
    raise RuntimeError(
        f"GOAL {GOAL_PIXEL} no está en espacio libre."
    )


# ============================================================
# Crear corte virtual
# ============================================================

blocked_original = map_data.blocked.copy()

cut_y = int(
    round(
        (START_PIXEL[1] + GOAL_PIXEL[1]) / 2
    )
)

cut_x = int(
    round(
        (START_PIXEL[0] + GOAL_PIXEL[0]) / 2
    )
)

if blocked_original[cut_y, cut_x]:
    raise RuntimeError(
        "El centro esperado del corte no está en el corredor libre."
    )


# Encontrar el corredor libre completo de esa fila.
left = cut_x

while (
    left >= 0
    and not blocked_original[cut_y, left]
):
    left -= 1

left += 1


right = cut_x

while (
    right < map_data.width
    and not blocked_original[cut_y, right]
):
    right += 1

right -= 1


# Cerrar completamente el corredor.
map_data.blocked[
    cut_y,
    left:right + 1
] = True


print()
print("========== CORTE VIRTUAL ==========")
print(f"Fila imagen: y = {cut_y}")
print(f"Desde x = {left} hasta x = {right}")
print(f"Ancho: {right - left + 1} píxeles")
print()


# ============================================================
# Crear Grid y ejecutar LPA*
# ============================================================

env = grid_from_map(map_data)

print("========== LPA* ==========")
print(f"START Grid: {start_grid}")
print(f"GOAL  Grid: {goal_grid}")
print(f"Mapa: {map_data.width} x {map_data.height}")
print(f"Inflación: {INFLATION_RADIUS_PX} px")
print("Calculando trayectoria...")
print()

planner = LPAStar(
    start=start_grid,
    goal=goal_grid,
    env=env,
    heuristic_type="euclidean",
)


try:
    cost, path, _ = planner.plan()
except ValueError as error:
    raise RuntimeError(
        "LPA* no encontró una trayectoria. "
        "El conjunto OPEN quedó vacío."
    ) from error


if not path:
    raise RuntimeError(
        "LPA* terminó pero devolvió una trayectoria vacía."
    )


cost_m = cost * map_data.resolution


print("========== RESULTADO ==========")
print(f"Nodos de la trayectoria: {len(path)}")
print(f"Costo Grid: {cost:.3f}")
print(f"Longitud aproximada: {cost_m:.3f} m")


# ============================================================
# Validación
# ============================================================

for grid_point in path:
    px, py = grid_to_image(
        map_data,
        grid_point,
    )

    if map_data.blocked[py, px]:
        raise RuntimeError(
            f"La trayectoria toca un obstáculo en {grid_point}."
        )

print("Colisiones detectadas: 0")


# ============================================================
# Convertir trayectoria
# ============================================================

path_image = np.asarray(
    [
        grid_to_image(
            map_data,
            point,
        )
        for point in path
    ],
    dtype=float,
)

path_world = [
    grid_to_world(
        map_data,
        point,
    )
    for point in path
]


# ============================================================
# Guardar CSV
# ============================================================

csv_path = HERE / "lpa_autodrive_raw_5px.csv"

with csv_path.open(
    "w",
    newline="",
    encoding="utf-8",
) as file:

    writer = csv.writer(file)

    writer.writerow(
        [
            "index",
            "grid_x",
            "grid_y",
            "world_x",
            "world_y",
        ]
    )

    for index, (
        grid_point,
        world_point,
    ) in enumerate(
        zip(path, path_world)
    ):

        writer.writerow(
            [
                index,
                grid_point[0],
                grid_point[1],
                f"{world_point[0]:.6f}",
                f"{world_point[1]:.6f}",
            ]
        )


print(f"CSV guardado: {csv_path}")


# ============================================================
# Visualización
# ============================================================

visual = np.where(
    map_data.blocked,
    0,
    255,
).astype(np.uint8)


fig, ax = plt.subplots(
    figsize=(6, 12)
)

ax.imshow(
    visual,
    cmap="gray",
    origin="upper",
)

ax.plot(
    path_image[:, 0],
    path_image[:, 1],
    linewidth=2,
    label="LPA*",
)

ax.scatter(
    START_PIXEL[0],
    START_PIXEL[1],
    marker="o",
    s=70,
    label="START",
    zorder=5,
)

ax.scatter(
    GOAL_PIXEL[0],
    GOAL_PIXEL[1],
    marker="X",
    s=80,
    label="GOAL",
    zorder=5,
)

ax.plot(
    [left, right],
    [cut_y, cut_y],
    linewidth=3,
    label="Corte virtual",
)

ax.set_title(
    "LPA* - AutoDRIVE - trayectoria cruda"
)

ax.set_xlabel("x - píxel")
ax.set_ylabel("y - píxel")

ax.set_xlim(
    0,
    map_data.width - 1,
)

ax.set_ylim(
    map_data.height - 1,
    0,
)

ax.grid(
    alpha=0.20
)

ax.legend()

plt.tight_layout()

png_path = HERE / "lpa_autodrive_raw_5px.png"

plt.savefig(
    png_path,
    dpi=200,
)

print(f"Imagen guardada: {png_path}")
print()

plt.show()
