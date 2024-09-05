import os
import numpy as np
from collections import defaultdict

from tqdm import tqdm

from utils.convertSemanticLabel import  convert_carla2nuScenes, mapping

categories = {
    0: "noise",
    1: "barrier",
    2: "bicycle",
    3: "bus",
    4: "car",
    5: "construction_vehicle",
    6: "motorcycle",
    7: "pedestrian",
    8: "traffic_cone",
    9: "trailer",
    10: "truck",
    11: "driveable_surface",
    12: "other_flat",
    13: "sidewalk",
    14: "terrain",
    15: "manmade",
    16: "vegetation"
}


def read_ply_file(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()
    end_header = next(i for i, line in enumerate(lines) if "end_header" in line)
    data_lines = lines[end_header + 1:]
    data = np.array([list(map(float, line.split())) for line in data_lines])
    data = data[:, -1]
    unique_elements, counts = np.unique(data, return_counts=True)
    labels_dict = dict(zip(unique_elements, counts))
    return labels_dict


def scan_directory(base_dir):
    category_distribution = defaultdict(int)
    all_files = []
    for root, dirs, files in os.walk(base_dir):
        if any(lidar in root for lidar in ["lidar_01", "lidar_02", "lidar_03", "lidar_04", "lidar_05"]):
            all_files.extend([os.path.join(root, file) for file in files if file.endswith('.ply')])

    for file_path in tqdm(all_files, desc="Processing PLY files"):
        labels_dict = read_ply_file(file_path)
        for key, count in labels_dict.items():
            category_distribution[key] += count

    converted_distribution = convert_carla2nuScenes(category_distribution, mapping)
    return {categories[k]: v for k, v in converted_distribution.items()}


if __name__ == "__main__":
    base_dir = './output'
    distribution = scan_directory(base_dir)
    print(distribution)
    for category, count in sorted(distribution.items()):
        print(f"{category}: {count}")