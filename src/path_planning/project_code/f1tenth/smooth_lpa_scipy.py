#!/usr/bin/env python3

import csv
import math
import sys
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import splprep, splev


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT))

from f1tenth.autodrive_common import (
    load_autodrive_map,
    grid_to_world,
)


# ============================================================
# CONFIGURACIÓN
# ============================================================

RAW_CSV = HERE / "lpa_autodrive_raw_5px.csv"

OUTPUT_CSV = HERE / "lpa_autodrive_smooth_final.csv"
OUTPUT_PNG = HERE / "lpa_autodrive_smooth_final.png"

# LPA* fue calculado con 5 px.
# La curva final se valida contra 2 px.
VALIDATION_INFLATION_PX = 2

START_PIXEL = (90, 139)
GOAL_PIXEL = (91, 144)

# Mayor valor = curva más suave/agresiva.
SMOOTHING_CANDIDATES = [
    5000,
    4000,
    3000,
    2500,
    2000,
    1500,
    1200,
    1000,
    800,
    600,
    500,
    400,
    300,
    200,
    150,
    100,
    75,
    50,
    25,
    10,
    5,
    1,
    0,
]

CURVE_SAMPLES = 1000


# ============================================================
# MAPA DE VALIDACIÓN
# ============================================================

map_data = load_autodrive_map(
    inflation_radius_px=VALIDATION_INFLATION_PX
)


# ============================================================
# RECREAR CORTE VIRTUAL
# ============================================================

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

left = cut_x

while (
    left >= 0
    and not map_data.blocked[cut_y, left]
):
    left -= 1

left += 1


right = cut_x

while (
    right < map_data.width
    and not map_data.blocked[cut_y, right]
):
    right += 1

right -= 1


map_data.blocked[
    cut_y,
    left:right + 1
] = True


# ============================================================
# CARGAR LPA* DE 5 PX
# ============================================================

raw_path = []

with RAW_CSV.open(
    "r",
    encoding="utf-8",
) as file:

    reader = csv.DictReader(file)

    for row in reader:
        raw_path.append(
            (
                float(row["grid_x"]),
                float(row["grid_y"]),
            )
        )


raw_path = np.asarray(
    raw_path,
    dtype=float,
)


print()
print("========== ENTRADA ==========")
print(f"Puntos LPA*: {len(raw_path)}")


# ============================================================
# PARAMETRIZACIÓN POR LONGITUD DE ARCO
# ============================================================

segment_lengths = np.linalg.norm(
    np.diff(raw_path, axis=0),
    axis=1,
)

cumulative = np.concatenate(
    (
        [0.0],
        np.cumsum(segment_lengths),
    )
)

u = cumulative / cumulative[-1]


# ============================================================
# FUNCIONES
# ============================================================

def grid_to_image_float(point):
    """Grid decimal -> coordenada decimal de imagen."""

    gx = float(point[0])
    gy = float(point[1])

    return np.array(
        [
            gx,
            map_data.height - 1 - gy,
        ],
        dtype=float,
    )


def image_point_is_free(point):
    """Comprueba un punto decimal contra el mapa."""

    px = int(round(float(point[0])))
    py = int(round(float(point[1])))

    if not (
        0 <= px < map_data.width
        and 0 <= py < map_data.height
    ):
        return False

    return not bool(
        map_data.blocked[py, px]
    )


def segment_is_free(start, end):
    """
    Muestreo denso de un segmento.
    Aproximadamente 4 comprobaciones por píxel.
    """

    start = np.asarray(start, dtype=float)
    end = np.asarray(end, dtype=float)

    delta = end - start

    maximum_delta = float(
        np.max(np.abs(delta))
    )

    samples = max(
        2,
        int(math.ceil(maximum_delta * 4.0)) + 1,
    )

    for t in np.linspace(
        0.0,
        1.0,
        samples,
    ):

        point = start + t * delta

        if not image_point_is_free(point):
            return False

    return True


def curve_is_free(curve_grid):

    curve_image = np.asarray(
        [
            grid_to_image_float(point)
            for point in curve_grid
        ]
    )

    for point in curve_image:
        if not image_point_is_free(point):
            return False

    for p1, p2 in zip(
        curve_image[:-1],
        curve_image[1:],
    ):
        if not segment_is_free(p1, p2):
            return False

    return True


def path_length_m(path):

    differences = np.diff(
        path,
        axis=0,
    )

    pixels = np.linalg.norm(
        differences,
        axis=1,
    ).sum()

    return float(
        pixels * map_data.resolution
    )


# ============================================================
# LONGITUD ORIGINAL
# ============================================================

raw_length = path_length_m(
    raw_path
)

print(
    f"Longitud LPA* 5 px: "
    f"{raw_length:.3f} m"
)


# ============================================================
# PESOS
# ============================================================

# Los extremos reciben mucho peso para mantener
# START y GOAL prácticamente fijos.
weights = np.ones(
    len(raw_path),
    dtype=float,
)

weights[:5] = 20.0
weights[-5:] = 20.0


# ============================================================
# PROBAR SUAVIZADOS
# ============================================================

selected_curve = None
selected_s = None

print()
print("========== PRUEBAS DE SUAVIZADO ==========")


for smoothing in SMOOTHING_CANDIDATES:

    print(
        f"s = {smoothing:5}: ",
        end="",
        flush=True,
    )

    try:

        tck, _ = splprep(
            [
                raw_path[:, 0],
                raw_path[:, 1],
            ],
            u=u,
            w=weights,
            k=3,
            s=float(smoothing),
        )

        u_new = np.linspace(
            0.0,
            1.0,
            CURVE_SAMPLES,
        )

        x_new, y_new = splev(
            u_new,
            tck,
        )

        curve = np.column_stack(
            (
                x_new,
                y_new,
            )
        )

        # Preservar exactamente START y GOAL.
        curve[0] = raw_path[0]
        curve[-1] = raw_path[-1]

    except Exception as error:

        print(
            f"ERROR: {error}"
        )

        continue


    if not curve_is_free(curve):

        print("COLISIÓN")

        continue


    curve_length = path_length_m(
        curve
    )


    # Evitar resultados patológicos.
    if curve_length > raw_length * 1.05:

        print(
            f"RECHAZADA ({curve_length:.3f} m)"
        )

        continue


    selected_curve = curve
    selected_s = smoothing

    print(
        f"OK ✅  ({curve_length:.3f} m)"
    )

    break


if selected_curve is None:
    raise RuntimeError(
        "Ningún nivel de suavizado produjo "
        "una trayectoria libre de colisiones."
    )


# ============================================================
# CLEARANCE
# ============================================================

free_mask = (
    ~map_data.blocked
).astype(np.uint8)

distance_map = cv2.distanceTransform(
    free_mask,
    cv2.DIST_L2,
    5,
)

minimum_clearance_px = float("inf")


for point in selected_curve:

    image_point = grid_to_image_float(
        point
    )

    px = int(round(image_point[0]))
    py = int(round(image_point[1]))

    minimum_clearance_px = min(
        minimum_clearance_px,
        float(distance_map[py, px]),
    )


minimum_clearance_m = (
    minimum_clearance_px
    * map_data.resolution
)


# ============================================================
# RESULTADOS
# ============================================================

smooth_length = path_length_m(
    selected_curve
)

difference = (
    raw_length - smooth_length
)

percentage = (
    difference
    / raw_length
    * 100.0
)


print()
print("========== RESULTADO FINAL ==========")
print(f"Suavizado elegido s: {selected_s}")
print(f"Puntos curva: {len(selected_curve)}")
print(f"Longitud LPA*: {raw_length:.3f} m")
print(f"Longitud suavizada: {smooth_length:.3f} m")
print(
    f"Reducción: {difference:.3f} m "
    f"({percentage:.2f} %)"
)
print("Colisiones: 0")
print(
    f"Clearance mínimo sobre mapa 2 px: "
    f"{minimum_clearance_px:.2f} px "
    f"({minimum_clearance_m:.3f} m)"
)
print("Trayectoria suavizada: VÁLIDA ✅")


# ============================================================
# WORLD + YAW
# ============================================================

world_path = np.asarray(
    [
        grid_to_world(
            map_data,
            point,
        )
        for point in selected_curve
    ],
    dtype=float,
)


yaw = np.zeros(
    len(world_path),
    dtype=float,
)


for i in range(
    len(world_path) - 1
):

    dx = (
        world_path[i + 1, 0]
        - world_path[i, 0]
    )

    dy = (
        world_path[i + 1, 1]
        - world_path[i, 1]
    )

    yaw[i] = math.atan2(
        dy,
        dx,
    )


yaw[-1] = yaw[-2]


# ============================================================
# GUARDAR CSV
# ============================================================

with OUTPUT_CSV.open(
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
            "yaw",
        ]
    )

    for index, (
        grid_point,
        world_point,
        angle,
    ) in enumerate(
        zip(
            selected_curve,
            world_path,
            yaw,
        )
    ):

        writer.writerow(
            [
                index,
                f"{grid_point[0]:.6f}",
                f"{grid_point[1]:.6f}",
                f"{world_point[0]:.6f}",
                f"{world_point[1]:.6f}",
                f"{angle:.6f}",
            ]
        )


# ============================================================
# VISUALIZACIÓN
# ============================================================

visual = np.where(
    map_data.blocked,
    0,
    255,
).astype(np.uint8)


raw_image = np.asarray(
    [
        grid_to_image_float(point)
        for point in raw_path
    ]
)


smooth_image = np.asarray(
    [
        grid_to_image_float(point)
        for point in selected_curve
    ]
)


fig, ax = plt.subplots(
    figsize=(7, 12)
)


ax.imshow(
    visual,
    cmap="gray",
    origin="upper",
)


ax.plot(
    raw_image[:, 0],
    raw_image[:, 1],
    linewidth=1,
    alpha=0.45,
    label="LPA* crudo (5 px)",
)


ax.plot(
    smooth_image[:, 0],
    smooth_image[:, 1],
    linewidth=2.5,
    label="B-Spline cúbico suavizado",
)


ax.scatter(
    START_PIXEL[0],
    START_PIXEL[1],
    s=75,
    marker="o",
    label="START",
    zorder=5,
)


ax.scatter(
    GOAL_PIXEL[0],
    GOAL_PIXEL[1],
    s=85,
    marker="X",
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
    "LPA* + B-Spline cúbico - trayectoria final"
)

ax.set_xlabel(
    "x - píxel"
)

ax.set_ylabel(
    "y - píxel"
)

ax.set_xlim(
    0,
    map_data.width - 1
)

ax.set_ylim(
    map_data.height - 1,
    0
)

ax.grid(
    alpha=0.20
)

ax.legend()

plt.tight_layout()


plt.savefig(
    OUTPUT_PNG,
    dpi=200,
)


print()
print(
    f"CSV: {OUTPUT_CSV}"
)

print(
    f"Imagen: {OUTPUT_PNG}"
)


plt.show()
