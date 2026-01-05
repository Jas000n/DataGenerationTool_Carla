import os
from collections import defaultdict
from concurrent.futures import ThreadPoolExecutor, as_completed

from tqdm import tqdm

from utils.convertSemanticLabel import convert_carla2nuScenes, mapping

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

LIDAR_DIRS = {"lidar_01", "lidar_02", "lidar_03", "lidar_04", "lidar_05"}


def read_ply_file(file_path: str) -> dict[int, int]:

    counts = defaultdict(int)
    with open(file_path, "r") as f:
        # 跳过 header
        for line in f:
            if "end_header" in line:
                break
        else:
            # 没找到 end_header，直接返回空
            return {}

        # 统计 body
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            try:
                label = int(float(parts[-1]))  # 兼容 "1" / "1.0"
            except (ValueError, IndexError):
                continue
            counts[label] += 1

    return dict(counts)


def _is_lidar_path(path: str) -> bool:

    parts = set(os.path.normpath(path).split(os.sep))
    return bool(parts & LIDAR_DIRS)


def collect_ply_files(base_dir: str) -> list[str]:
    all_files = []
    for root, _, files in os.walk(base_dir):
        if _is_lidar_path(root):
            for fn in files:
                if fn.endswith(".ply"):
                    all_files.append(os.path.join(root, fn))
    return all_files


def scan_directory(base_dir: str, max_workers: int | None = None) -> dict[str, int]:
    category_distribution = defaultdict(int)

    all_files = collect_ply_files(base_dir)
    if not all_files:
        return {}


    if max_workers is None:
        cpu = os.cpu_count() or 4
        max_workers = min(32, cpu * 4)

    with ThreadPoolExecutor(max_workers=max_workers) as executor:
        futures = [executor.submit(read_ply_file, fp) for fp in all_files]

        for fut in tqdm(as_completed(futures), total=len(futures), desc="Processing PLY files"):
            labels_dict = fut.result()
            for key, count in labels_dict.items():
                category_distribution[key] += count

    converted_distribution = convert_carla2nuScenes(category_distribution, mapping)

  
    return {categories[k]: v for k, v in converted_distribution.items()}


if __name__ == "__main__":
    base_dir = "./output"
    distribution = scan_directory(base_dir)  # 也可以 scan_directory(base_dir, max_workers=16)

    print(distribution)
    for category, count in sorted(distribution.items()):
        print(f"{category}: {count}")
