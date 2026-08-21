#!/usr/bin/env python3

import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import numpy as np


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT))


from f1tenth.autodrive_common import (
    load_autodrive_map,
    grid_from_map,
    image_to_grid,
)

from python_motion_planning.global_planner.graph_search.lpa_star import (
    LPAStar,
)


# ============================================================
# CONFIGURACIÓN
# ============================================================

PLANNING_INFLATION_PX = 5
VALIDATION_INFLATION_PX = 2

START_PIXEL = (90, 139)
GOAL_PIXEL = (91, 144)

RAW_CSV = HERE / "lpa_autodrive_raw_5px.csv"
SMOOTH_CSV = HERE / "lpa_autodrive_smooth_final.csv"

OUTPUT_GIF = HERE / "lpa_autodrive_animation_final.gif"


# ============================================================
# CARGAR MAPAS
# ============================================================

planning_map = load_autodrive_map(
    inflation_radius_px=PLANNING_INFLATION_PX
)

validation_map = load_autodrive_map(
    inflation_radius_px=VALIDATION_INFLATION_PX
)


# ============================================================
# CORTE VIRTUAL
# ============================================================

def add_virtual_cut(map_data):

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

    return cut_y, left, right


planning_cut_y, planning_left, planning_right = (
    add_virtual_cut(planning_map)
)

cut_y, left, right = add_virtual_cut(
    validation_map
)


# ============================================================
# EJECUTAR LPA* PARA OBTENER NODOS EXPANDIDOS
# ============================================================

env = grid_from_map(
    planning_map
)

start_grid = image_to_grid(
    planning_map,
    START_PIXEL,
)

goal_grid = image_to_grid(
    planning_map,
    GOAL_PIXEL,
)


planner = LPAStar(
    start=start_grid,
    goal=goal_grid,
    env=env,
    heuristic_type="euclidean",
)


cost, generated_path, _ = planner.plan()


if not generated_path:
    raise RuntimeError(
        "LPA* no encontró una trayectoria."
    )


expanded = [
    node.current
    for node in planner.EXPAND
]


# Cerrar cualquier figura interna creada por el planificador.
plt.close("all")


# ============================================================
# CARGAR CSV
# ============================================================

def load_grid_csv(path):

    points = []

    with path.open(
        "r",
        encoding="utf-8",
    ) as file:

        reader = csv.DictReader(file)

        for row in reader:

            points.append(
                (
                    float(row["grid_x"]),
                    float(row["grid_y"]),
                )
            )

    return np.asarray(
        points,
        dtype=float,
    )


raw_path = load_grid_csv(
    RAW_CSV
)

smooth_path = load_grid_csv(
    SMOOTH_CSV
)


# ============================================================
# GRID DECIMAL -> IMAGEN DECIMAL
# ============================================================

def grid_to_image_float(
    map_data,
    point,
):

    return (
        float(point[0]),
        float(
            map_data.height
            - 1
            - point[1]
        ),
    )


expanded_image = np.asarray(
    [
        grid_to_image_float(
            validation_map,
            point,
        )
        for point in expanded
    ],
    dtype=float,
)


raw_image = np.asarray(
    [
        grid_to_image_float(
            validation_map,
            point,
        )
        for point in raw_path
    ],
    dtype=float,
)


smooth_image = np.asarray(
    [
        grid_to_image_float(
            validation_map,
            point,
        )
        for point in smooth_path
    ],
    dtype=float,
)


# ============================================================
# MAPA VISUAL
# ============================================================

visual = np.where(
    validation_map.blocked,
    0,
    255,
).astype(np.uint8)


# ============================================================
# FIGURA
# ============================================================

fig, ax = plt.subplots(
    figsize=(6, 10)
)


ax.imshow(
    visual,
    cmap="gray",
    origin="upper",
)


ax.scatter(
    START_PIXEL[0],
    START_PIXEL[1],
    marker="o",
    s=70,
    label="START",
    zorder=10,
)


ax.scatter(
    GOAL_PIXEL[0],
    GOAL_PIXEL[1],
    marker="X",
    s=80,
    label="GOAL",
    zorder=10,
)


ax.plot(
    [left, right],
    [cut_y, cut_y],
    linewidth=3,
    label="Corte virtual",
)


expanded_scatter = ax.scatter(
    [],
    [],
    s=3,
    alpha=0.30,
    label="Nodos expandidos LPA*",
)


raw_line, = ax.plot(
    [],
    [],
    linewidth=1.5,
    alpha=0.65,
    label="LPA* crudo",
)


smooth_line, = ax.plot(
    [],
    [],
    linewidth=2.8,
    label="B-Spline cúbico",
)


ax.set_xlabel(
    "x - píxel"
)

ax.set_ylabel(
    "y - píxel"
)

ax.set_xlim(
    0,
    validation_map.width - 1
)

ax.set_ylim(
    validation_map.height - 1,
    0
)

ax.grid(
    alpha=0.15
)

ax.legend(
    loc="lower right"
)


# ============================================================
# ETAPAS
# ============================================================

INITIAL_FRAMES = 12
EXPLORE_TARGET_FRAMES = 90
RAW_PATH_FRAMES = 45
SMOOTH_PATH_FRAMES = 70
FINAL_FRAMES = 35


explore_step = max(
    1,
    len(expanded)
    // EXPLORE_TARGET_FRAMES
)


explore_indexes = list(
    range(
        explore_step,
        len(expanded) + 1,
        explore_step,
    )
)


if (
    not explore_indexes
    or explore_indexes[-1] != len(expanded)
):
    explore_indexes.append(
        len(expanded)
    )


raw_indexes = np.linspace(
    1,
    len(raw_image),
    RAW_PATH_FRAMES,
    dtype=int,
)


smooth_indexes = np.linspace(
    1,
    len(smooth_image),
    SMOOTH_PATH_FRAMES,
    dtype=int,
)


total_frames = (
    INITIAL_FRAMES
    + len(explore_indexes)
    + len(raw_indexes)
    + len(smooth_indexes)
    + FINAL_FRAMES
)


# ============================================================
# ANIMACIÓN
# ============================================================

def update(frame):

    empty = np.empty(
        (0, 2)
    )


    # --------------------------------------------------------
    # 1. MAPA
    # --------------------------------------------------------

    if frame < INITIAL_FRAMES:

        expanded_scatter.set_offsets(
            empty
        )

        raw_line.set_data(
            [],
            [],
        )

        smooth_line.set_data(
            [],
            [],
        )

        ax.set_title(
            "LPA* + B-Spline - mapa de planificación"
        )


    # --------------------------------------------------------
    # 2. EXPLORACIÓN LPA*
    # --------------------------------------------------------

    elif frame < (
        INITIAL_FRAMES
        + len(explore_indexes)
    ):

        local_frame = (
            frame
            - INITIAL_FRAMES
        )

        amount = explore_indexes[
            local_frame
        ]

        expanded_scatter.set_offsets(
            expanded_image[:amount]
        )

        expanded_scatter.set_alpha(
            0.30
        )

        raw_line.set_data(
            [],
            [],
        )

        smooth_line.set_data(
            [],
            [],
        )

        ax.set_title(
            f"LPA* - exploración: "
            f"{amount} nodos expandidos"
        )


    # --------------------------------------------------------
    # 3. TRAYECTORIA CRUDA
    # --------------------------------------------------------

    elif frame < (
        INITIAL_FRAMES
        + len(explore_indexes)
        + len(raw_indexes)
    ):

        local_frame = (
            frame
            - INITIAL_FRAMES
            - len(explore_indexes)
        )

        amount = raw_indexes[
            local_frame
        ]

        expanded_scatter.set_offsets(
            expanded_image
        )

        expanded_scatter.set_alpha(
            0.15
        )

        raw_line.set_alpha(
            0.90
        )

        raw_line.set_data(
            raw_image[:amount, 0],
            raw_image[:amount, 1],
        )

        smooth_line.set_data(
            [],
            [],
        )

        ax.set_title(
            "LPA* - trayectoria global cruda"
        )


    # --------------------------------------------------------
    # 4. B-SPLINE
    # --------------------------------------------------------

    elif frame < (
        INITIAL_FRAMES
        + len(explore_indexes)
        + len(raw_indexes)
        + len(smooth_indexes)
    ):

        local_frame = (
            frame
            - INITIAL_FRAMES
            - len(explore_indexes)
            - len(raw_indexes)
        )

        amount = smooth_indexes[
            local_frame
        ]

        expanded_scatter.set_offsets(
            empty
        )

        raw_line.set_alpha(
            0.35
        )

        raw_line.set_data(
            raw_image[:, 0],
            raw_image[:, 1],
        )

        smooth_line.set_data(
            smooth_image[:amount, 0],
            smooth_image[:amount, 1],
        )

        ax.set_title(
            "B-Spline cúbico - suavizando trayectoria"
        )


    # --------------------------------------------------------
    # 5. RESULTADO FINAL
    # --------------------------------------------------------

    else:

        expanded_scatter.set_offsets(
            empty
        )

        raw_line.set_data(
            [],
            [],
        )

        smooth_line.set_data(
            smooth_image[:, 0],
            smooth_image[:, 1],
        )

        ax.set_title(
            "LPA* + B-Spline - trayectoria final suavizada"
        )


    return (
        expanded_scatter,
        raw_line,
        smooth_line,
    )


animation = FuncAnimation(
    fig,
    update,
    frames=total_frames,
    interval=70,
    blit=False,
)


# ============================================================
# GUARDAR
# ============================================================

print()
print("========== GIF FINAL ==========")
print(f"Nodos expandidos LPA*: {len(expanded)}")
print(f"Puntos LPA* crudo: {len(raw_path)}")
print(f"Puntos B-Spline: {len(smooth_path)}")
print(f"Costo LPA*: {cost:.3f}")
print("Suavizado B-Spline: s = 1500")
print("Colisiones trayectoria final: 0")
print()
print("Generando GIF...")


animation.save(
    OUTPUT_GIF,
    writer=PillowWriter(
        fps=14
    ),
    dpi=120,
)


print()
print(
    f"GIF guardado en: {OUTPUT_GIF}"
)


plt.close(fig)
