import numpy as np
import matplotlib.pyplot as plt

def visualise_map(map_filepath, walls):
    map_points, max_grey = img_to_list(map_filepath)
    map_points = add_walls(map_points, walls, max_grey//2)
    draw_map(map_points)

def img_to_list(name):
    with open(name) as f:
        lines = f.readlines()

    # Ignores commented lines
    for l in list(lines):
        if l[0] == "#":
            lines.remove(l)


    # Converts data to a list of integers
    data = []
    for line in lines[1:]:
        data.extend([int(c) for c in line.split()])

    return (np.reshape(data[3:], (data[1], data[0])), data[2])
    


def add_walls(map_points, walls_points, walls_grey):
    for walls_point in walls_points:
        if map_points[walls_point] == 254:
            map_points[walls_point] = walls_grey
    return map_points


def draw_map(map_points):
    plt.imshow(map_points)
    plt.show()


# if not work try to set your absolute path
visualise_map("ActualP2Image.pgm", 
                [(y, x) for x in range(200, 250) for y in range(130, 170)]
            )