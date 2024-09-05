import os
from os import mkdir
from pathlib import Path

import open3d as o3d
import numpy as np
from collections import Counter
from utils import convertSemanticLabel
def read_ply_with_properties(path):
    with open(path, 'r') as file:
        lines = file.readlines()

    end_header_index = 0
    for i, line in enumerate(lines):
        if "end_header" in line:
            end_header_index = i + 1
            break

    data = []
    for line in lines[end_header_index:]:
        data.append(list(map(float, line.split())))

    return np.array(data)

def most_common(lst):
    data = Counter(lst)
    return data.most_common(1)[0][0]

# Function to calculate voxel center from its grid index
def voxel_center_from_index(index, resolution, min_bound):
    return [min_bound[i] + (index[i] + 0.5) * resolution for i in range(3)]

def voxelization_save(file_path,save_path, min_bound = [-32, -32, -3],max_bound = [32, 32, 5], resolution = 0.2):
# Load the data and point cloud
    path_to_ply = file_path
    data = read_ply_with_properties(path_to_ply)
    points = data[:, 0:3]
    points = lidar2ego(points,np.array([0,0,1.6]))
    categories = data[:, 5].astype(int)

    # Load points into an Open3D point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)



    # Create voxel grid within the defined bounds
    voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud_within_bounds(pcd, resolution, min_bound, max_bound)

    # Map each point to its corresponding voxel and category
    point_to_voxel_map = {}
    for point, category in zip(np.asarray(pcd.points), categories):
        if point[0]<max_bound[0] and point[0]>min_bound[0] and point[1]<max_bound[1] and point[1]>min_bound[1] and point[2]<max_bound[2] and point[2]>min_bound[2]:
            voxel_index = tuple(voxel_grid.get_voxel(point))
            if voxel_index in point_to_voxel_map:
                point_to_voxel_map[voxel_index].append(category)
            else:
                point_to_voxel_map[voxel_index] = [category]

    # Determine the majority category for each voxel
    voxel_categories = {voxel: most_common(categories) for voxel, categories in point_to_voxel_map.items()}
    # Color mapping normalized to [0, 1]
    category_colors = {
        0: (0, 0, 0),        # Unlabeled
        1: (70, 70, 70),     # Building
        2: (100, 40, 40),    # Fence
        3: (55, 90, 80),     # Other
        4: (220, 20, 60),    # Pedestrian
        5: (153, 153, 153),  # Pole
        6: (157, 234, 50),   # RoadLine
        7: (128, 64, 128),   # Road
        8: (244, 35, 232),   # SideWalk
        9: (107, 142, 35),   # Vegetation
        10: (0, 0, 142),     # Vehicles
        11: (102, 102, 156), # Wall
        12: (220, 220, 0),   # TrafficSign
        13: (70, 130, 180),  # Sky
        14: (81, 0, 81),     # Ground
        15: (150, 100, 100), # Bridge
        16: (230, 150, 140), # RailTrack
        17: (180, 165, 180), # GuardRail
        18: (250, 170, 30),  # TrafficLight
        19: (110, 190, 160), # Static
        20: (170, 120, 50),  # Dynamic
        21: (45, 60, 150),   # Water
        22: (145, 170, 100), # Terrain
    }
    for key in category_colors:
        category_colors[key] = tuple(value / 255 for value in category_colors[key])
    # Create a colored point cloud for visualization
    colored_pcd = o3d.geometry.PointCloud()
    voxel_centers = []
    colors = []

    for voxel_index, category in voxel_categories.items():
        center = voxel_center_from_index(voxel_index, resolution, min_bound)
        voxel_centers.append(center)
        color = category_colors.get(category, [0, 0, 0])
        colors.append(color)

    colored_pcd.points = o3d.utility.Vector3dVector(voxel_centers)
    colored_pcd.colors = o3d.utility.Vector3dVector(colors)

    # Visualize the colored point cloud
    #o3d.visualization.draw_geometries([colored_pcd])


    for key in voxel_categories.keys():
        value = convertSemanticLabel.convert_semantic_label(voxel_categories[key])
        voxel_categories[key]=value
    dict = {"gt_occ": voxel_categories, "resolution": resolution, "min_bound": min_bound,
        "max_bound": max_bound}
    np.save(save_path+"_info", dict)
    o3d.io.write_voxel_grid(save_path+"_voxel.ply", voxel_grid)
def lidar2ego(points,translation,rotation=None):
    # input should be (n,3)
    if(rotation is not None):
        print("lidar should be at same orientation with vehicle!!!!")
    translated_points = points + translation
    return translated_points
#the original func, from a single numpy to voxel
def voxelization_from_np_save(np_array, save_path, min_bound = [-32, -32, -3],max_bound = [32, 32, 5], resolution = 0.2):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np_array[:,:3])
    categories = np_array[:, 5].astype(int)
    # Create voxel grid within the defined bounds
    voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud_within_bounds(pcd, resolution, min_bound, max_bound)

    # Map each point to its corresponding voxel and category
    point_to_voxel_map = {}
    for point, category in zip(np.asarray(pcd.points), categories):
        if point[0] < max_bound[0] and point[0] > min_bound[0] and point[1] < max_bound[1] and point[1] > min_bound[
            1] and point[2] < max_bound[2] and point[2] > min_bound[2]:
            voxel_index = tuple(voxel_grid.get_voxel(point))
            if voxel_index in point_to_voxel_map:
                point_to_voxel_map[voxel_index].append(category)
            else:
                point_to_voxel_map[voxel_index] = [category]

    # Determine the majority category for each voxel
    voxel_categories = {voxel: most_common(categories) for voxel, categories in point_to_voxel_map.items()}
    # Color mapping normalized to [0, 1]
    category_colors = {
        0: (0, 0, 0),  # Unlabeled
        1: (70, 70, 70),  # Building
        2: (100, 40, 40),  # Fence
        3: (55, 90, 80),  # Other
        4: (220, 20, 60),  # Pedestrian
        5: (153, 153, 153),  # Pole
        6: (157, 234, 50),  # RoadLine
        7: (128, 64, 128),  # Road
        8: (244, 35, 232),  # SideWalk
        9: (107, 142, 35),  # Vegetation
        10: (0, 0, 142),  # Vehicles
        11: (102, 102, 156),  # Wall
        12: (220, 220, 0),  # TrafficSign
        13: (70, 130, 180),  # Sky
        14: (81, 0, 81),  # Ground
        15: (150, 100, 100),  # Bridge
        16: (230, 150, 140),  # RailTrack
        17: (180, 165, 180),  # GuardRail
        18: (250, 170, 30),  # TrafficLight
        19: (110, 190, 160),  # Static
        20: (170, 120, 50),  # Dynamic
        21: (45, 60, 150),  # Water
        22: (145, 170, 100),  # Terrain
    }
    for key in category_colors:
        category_colors[key] = tuple(value / 255 for value in category_colors[key])
    # Create a colored point cloud for visualization
    colored_pcd = o3d.geometry.PointCloud()
    voxel_centers = []
    colors = []

    for voxel_index, category in voxel_categories.items():
        center = voxel_center_from_index(voxel_index, resolution, min_bound)
        voxel_centers.append(center)
        color = category_colors.get(category, [0, 0, 0])
        colors.append(color)

    colored_pcd.points = o3d.utility.Vector3dVector(voxel_centers)
    colored_pcd.colors = o3d.utility.Vector3dVector(colors)

    # Visualize the colored point cloud
    # o3d.visualization.draw_geometries([colored_pcd])

    for key in voxel_categories.keys():
        value = convertSemanticLabel.convert_semantic_label(voxel_categories[key])
        voxel_categories[key] = value
    dict = {"gt_occ": voxel_categories, "resolution": resolution, "min_bound": min_bound,
            "max_bound": max_bound}
    np.save(save_path + "_info", dict)
    o3d.io.write_voxel_grid(save_path + "_voxel.ply", voxel_grid)

def align_from_path_save(curpath,sensor_specs,ticks,min_bound = [-32, -32, -3],max_bound = [32, 32, 5], resolution = 0.2):
    all_points = []
    all_categories = []
    curpath = str(curpath)
    os.makedirs(curpath+"/voxel", exist_ok=True)
    for key in sensor_specs.keys():

        data = read_ply_with_properties(curpath +"/"+ key +"/"+ (f"{ticks:04}.ply"))
        tmp_points = data[:, 0:3]
        tmp_categories = data[:, 5].astype(int)
        tmp_points = lidar2ego(tmp_points,[sensor_specs[key]["x"],sensor_specs[key]["y"],sensor_specs[key]["z"]])
        all_points.append(tmp_points)
        all_categories.append(tmp_categories)
    all_points = np.concatenate(all_points)
    all_categories = np.concatenate(all_categories)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(all_points)

    # Create voxel grid within the defined bounds
    voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud_within_bounds(pcd, resolution, min_bound, max_bound)

    # Map each point to its corresponding voxel and category
    point_to_voxel_map = {}
    for point, category in zip(np.asarray(pcd.points), all_categories):
        if point[0] < max_bound[0] and point[0] > min_bound[0] and point[1] < max_bound[1] and point[1] > min_bound[
            1] and point[2] < max_bound[2] and point[2] > min_bound[2]:
            voxel_index = tuple(voxel_grid.get_voxel(point))
            if voxel_index in point_to_voxel_map:
                point_to_voxel_map[voxel_index].append(category)
            else:
                point_to_voxel_map[voxel_index] = [category]

    # Determine the majority category for each voxel
    voxel_categories = {voxel: most_common(categories) for voxel, categories in point_to_voxel_map.items()}
    # Color mapping normalized to [0, 1]
    category_colors = {
        0: (0, 0, 0),  # Unlabeled
        1: (70, 70, 70),  # Building
        2: (100, 40, 40),  # Fence
        3: (55, 90, 80),  # Other
        4: (220, 20, 60),  # Pedestrian
        5: (153, 153, 153),  # Pole
        6: (157, 234, 50),  # RoadLine
        7: (128, 64, 128),  # Road
        8: (244, 35, 232),  # SideWalk
        9: (107, 142, 35),  # Vegetation
        10: (0, 0, 142),  # Vehicles
        11: (102, 102, 156),  # Wall
        12: (220, 220, 0),  # TrafficSign
        13: (70, 130, 180),  # Sky
        14: (81, 0, 81),  # Ground
        15: (150, 100, 100),  # Bridge
        16: (230, 150, 140),  # RailTrack
        17: (180, 165, 180),  # GuardRail
        18: (250, 170, 30),  # TrafficLight
        19: (110, 190, 160),  # Static
        20: (170, 120, 50),  # Dynamic
        21: (45, 60, 150),  # Water
        22: (145, 170, 100),  # Terrain
    }
    for key in category_colors:
        category_colors[key] = tuple(value / 255 for value in category_colors[key])
    # Create a colored point cloud for visualization
    colored_pcd = o3d.geometry.PointCloud()
    voxel_centers = []
    colors = []

    for voxel_index, category in voxel_categories.items():
        center = voxel_center_from_index(voxel_index, resolution, min_bound)
        voxel_centers.append(center)
        color = category_colors.get(category, [0, 0, 0])
        colors.append(color)

    colored_pcd.points = o3d.utility.Vector3dVector(voxel_centers)
    colored_pcd.colors = o3d.utility.Vector3dVector(colors)

    # Visualize the colored point cloud
    # o3d.visualization.draw_geometries([colored_pcd])

    for key in voxel_categories.keys():
        value = convertSemanticLabel.convert_semantic_label(voxel_categories[key])
        voxel_categories[key] = value
    dict = {"gt_occ": voxel_categories, "resolution": resolution, "min_bound": min_bound,
            "max_bound": max_bound}
    np.save(curpath+"/voxel/"+f"{ticks:04}"+"_info", dict)
    o3d.io.write_voxel_grid(curpath+"/voxel/" +f"{ticks:04}"+ "_voxel.ply", voxel_grid)



if  "__main__" == __name__:
    voxelization_save("../e2e_parking/Town04_Opt/08_05_13_57_49/task0/lidar/0003.ply","../e2e_parking/Town04_Opt/08_05_13_57_49/task0/lidar/11000")