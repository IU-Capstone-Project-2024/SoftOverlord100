import numpy as np
import matplotlib.pyplot as plt
import pathlib


def add_map_filter(map_filepath, walls):
    map_points, empty_color = img_to_list(map_filepath)
    map_points = add_walls(map_points, walls, empty_color)


def img_to_list(name):
    with open(name, "rb") as f:
        lines = f.readlines()

    # Ignore commented lines
    for l in list(lines):
        if l[0] == ord("#"):
            lines.remove(l)

    # Convert data to a list of integers
    width, height = [int(c) for c in lines[1].split()]

    raster = []
    i = 0
    for _ in range(height):
        row = []
        for _ in range(width):
            row.append(ord(lines[3][i]))
            i += 1
        raster.append(row)

    return (np.reshape(raster, (height, width)), max(raster))


def add_walls(map_points, walls_points, empty_color):
    for walls_point in walls_points:
        if map_points[walls_point] == empty_color:
            map_points[walls_point] = 0
    return map_points


# if not work try to set your absolute path

print(pathlib.Path().resolve())
add_map_filter(
    "./map_visualiser/turtlebot3_world.pgm",
    [(y, x) for x in range(200, 250) for y in range(130, 170)],
)
