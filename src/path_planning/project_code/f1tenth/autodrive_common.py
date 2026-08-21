#!/usr/bin/env python3
"""Herramientas comunes para planificación sobre el mapa SLAM de AutoDRIVE."""

from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import cv2
import numpy as np
import yaml

from python_motion_planning.utils import Grid


HERE = Path(__file__).resolve().parent
PATH_PLANNING_ROOT = HERE.parent.parent

YAML_PATH = (
    PATH_PLANNING_ROOT
    / "maps"
    / "Proyecto_F1Tenth_Map.yaml"
)


# En el PGM generado por map_saver:
# 254 = libre
# 205 = desconocido
#   0 = ocupado
FREE_VALUE = 254

# 2 píxeles × 0.05 m = 0.10 m de margen.
DEFAULT_INFLATION_RADIUS_PX = 2


@dataclass(frozen=True)
class AutoDriveMap:
    """Mapa SLAM y parámetros necesarios para planificación."""

    image: np.ndarray
    blocked: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float
    yaml_path: Path
    image_path: Path

    @property
    def height(self) -> int:
        return int(self.image.shape[0])

    @property
    def width(self) -> int:
        return int(self.image.shape[1])


def load_autodrive_map(
    yaml_path: Path = YAML_PATH,
    *,
    inflation_radius_px: int = DEFAULT_INFLATION_RADIUS_PX,
) -> AutoDriveMap:
    """
    Carga el mapa de AutoDRIVE.

    blocked[y, x] == True:
        celda no transitable.

    Solo los píxeles con valor 254 se consideran libres.
    """

    yaml_path = Path(yaml_path).expanduser().resolve()

    if not yaml_path.exists():
        raise FileNotFoundError(
            f"No se encontró el YAML: {yaml_path}"
        )

    with yaml_path.open("r", encoding="utf-8") as file:
        config = yaml.safe_load(file)

    image_path = Path(config["image"])

    if not image_path.is_absolute():
        image_path = (yaml_path.parent / image_path).resolve()

    image = cv2.imread(
        str(image_path),
        cv2.IMREAD_GRAYSCALE,
    )

    if image is None:
        raise FileNotFoundError(
            f"No se pudo abrir el mapa: {image_path}"
        )

    # Únicamente el blanco 254 es espacio conocido y libre.
    free = image == FREE_VALUE

    # Negro y gris desconocido quedan bloqueados.
    blocked = np.logical_not(free)

    if inflation_radius_px < 0:
        raise ValueError(
            "El radio de inflación no puede ser negativo."
        )

    if inflation_radius_px > 0:
        kernel_size = 2 * inflation_radius_px + 1

        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (kernel_size, kernel_size),
        )

        blocked = cv2.dilate(
            blocked.astype(np.uint8),
            kernel,
            iterations=1,
        ).astype(bool)

    # Garantizar que los límites nunca sean transitables.
    blocked[0, :] = True
    blocked[-1, :] = True
    blocked[:, 0] = True
    blocked[:, -1] = True

    resolution = float(config["resolution"])
    origin = config["origin"]

    return AutoDriveMap(
        image=image,
        blocked=blocked,
        resolution=resolution,
        origin_x=float(origin[0]),
        origin_y=float(origin[1]),
        yaml_path=yaml_path,
        image_path=image_path,
    )


def image_to_grid(
    map_data: AutoDriveMap,
    pixel: Sequence[float],
) -> tuple[int, int]:
    """
    Convierte píxel de imagen (origen arriba-izquierda)
    a coordenada Grid/ROS (origen abajo-izquierda).
    """

    px = int(round(pixel[0]))
    py = int(round(pixel[1]))

    gx = px
    gy = map_data.height - 1 - py

    return gx, gy


def grid_to_image(
    map_data: AutoDriveMap,
    grid: Sequence[float],
) -> tuple[int, int]:
    """Convierte coordenada Grid a píxel de imagen."""

    gx = int(round(grid[0]))
    gy = int(round(grid[1]))

    px = gx
    py = map_data.height - 1 - gy

    return px, py


def grid_to_world(
    map_data: AutoDriveMap,
    grid: Sequence[float],
) -> tuple[float, float]:
    """Convierte coordenada Grid a coordenada ROS en metros."""

    gx = float(grid[0])
    gy = float(grid[1])

    world_x = map_data.origin_x + gx * map_data.resolution
    world_y = map_data.origin_y + gy * map_data.resolution

    return world_x, world_y


def world_to_grid(
    map_data: AutoDriveMap,
    world: Sequence[float],
) -> tuple[int, int]:
    """Convierte coordenada ROS en metros a Grid."""

    world_x = float(world[0])
    world_y = float(world[1])

    gx = round(
        (world_x - map_data.origin_x)
        / map_data.resolution
    )

    gy = round(
        (world_y - map_data.origin_y)
        / map_data.resolution
    )

    return int(gx), int(gy)


def image_to_world(
    map_data: AutoDriveMap,
    pixel: Sequence[float],
) -> tuple[float, float]:
    """Convierte directamente píxel de imagen a coordenadas ROS."""

    return grid_to_world(
        map_data,
        image_to_grid(map_data, pixel),
    )


def is_grid_free(
    map_data: AutoDriveMap,
    grid: Sequence[float],
) -> bool:
    """Comprueba si una celda Grid es válida y está libre."""

    gx = int(round(grid[0]))
    gy = int(round(grid[1]))

    if not (
        0 <= gx < map_data.width
        and 0 <= gy < map_data.height
    ):
        return False

    px, py = grid_to_image(
        map_data,
        (gx, gy),
    )

    return not bool(map_data.blocked[py, px])


def grid_from_map(
    map_data: AutoDriveMap,
) -> Grid:
    """Construye un Grid compatible con los planificadores del repositorio."""

    env = Grid(
        map_data.width,
        map_data.height,
    )

    blocked_y, blocked_x = np.where(
        map_data.blocked
    )

    obstacles = {
        (
            int(px),
            int(map_data.height - 1 - py),
        )
        for py, px in zip(blocked_y, blocked_x)
    }

    env.update(obstacles)

    return env
