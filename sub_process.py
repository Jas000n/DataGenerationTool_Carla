import json
import math
import os
import sys
from pathlib import Path
from queue import Queue
from datetime import datetime
import yaml
from utils.generate_video import images_to_video

import carla
import random
import pygame
import time

from utils.ply2voxel import voxelization_save, align_from_path_save


def init_pygame():
    pygame.init()
    size = (200, 200)
    pygame.display.set_mode(size)
def save_unit_data(sensor_data_frame,cur_save_path,ticks,lidar_specs,ego_vehicle):

    data_frame = sensor_data_frame
    ticks = int(ticks/5-1)
    cur_save_path = Path(cur_save_path)

    for sensor in data_frame.keys():
        if sensor.startswith('depth') and not sensor.startswith("depth_e2e"):
            data_frame[sensor].save_to_disk(
                str(cur_save_path / sensor / (f"{ticks:04}.png")))
        if sensor.startswith('camera'):
            data_frame[sensor].save_to_disk(
                str(cur_save_path / sensor / (f"{ticks:04}.png")))
        elif sensor.startswith('lidar'):
            data_frame[sensor].save_to_disk(
                str(cur_save_path / sensor / (f"{ticks:04}.ply")))
    align_from_path_save(cur_save_path,lidar_specs,ticks )
    # save measurements
    imu_data = data_frame['imu']
    gnss_data = data_frame['gnss']
    vehicle_transform = ego_vehicle.get_transform()
    vehicle_velocity = ego_vehicle.get_velocity()
    vehicle_control = ego_vehicle.get_control()

    data = {
        'x': vehicle_transform.location.x,
        'y': vehicle_transform.location.y,
        'z': vehicle_transform.location.z,
        'pitch': vehicle_transform.rotation.pitch,
        'yaw': vehicle_transform.rotation.yaw,
        'roll': vehicle_transform.rotation.roll,
        'speed': (3.6 * math.sqrt(vehicle_velocity.x ** 2 + vehicle_velocity.y ** 2 + vehicle_velocity.z ** 2)),
        'Throttle': vehicle_control.throttle,
        'Steer': vehicle_control.steer,
        'Brake': vehicle_control.brake,
        'Reverse': vehicle_control.reverse,
        'Hand brake': vehicle_control.hand_brake,
        'Manual': vehicle_control.manual_gear_shift,
        'Gear': {-1: 'R', 0: 'N'}.get(vehicle_control.gear, vehicle_control.gear),
        'acc_x': imu_data.accelerometer.x,
        'acc_y': imu_data.accelerometer.y,
        'acc_z': imu_data.accelerometer.z,
        'gyr_x': imu_data.gyroscope.x,
        'gyr_y': imu_data.gyroscope.y,
        'gyr_z': imu_data.gyroscope.z,
        'compass': imu_data.compass,
        'lat': gnss_data.latitude,
        'lon': gnss_data.longitude
    }

    if not os.path.exists(cur_save_path / 'measurements'):
        os.makedirs(cur_save_path / 'measurements')
    measurements_file = cur_save_path / 'measurements' / f"{ticks:04}.json"
    with open(measurements_file, 'w') as f:
        json.dump(data, f, indent=4)

def sensor_callback(sensor_data, sensor_queue, sensor_name):
    sensor_queue.put((sensor_data, sensor_name))
def spawn_camera(world,vehicle,camera_id,camera_specs,sensor_list,sensor_queue):
    blueprint_library = world.get_blueprint_library()
    sensor_bp = blueprint_library.find(camera_specs["type"])
    camera_location = carla.Location(x=camera_specs['x'], y=camera_specs['y'], z=camera_specs['z'])
    camera_rotation = carla.Rotation(pitch=camera_specs['pitch'], roll=camera_specs['roll'], yaw=camera_specs['yaw'])
    camera_transform = carla.Transform(camera_location, camera_rotation)

    if "fov" in camera_specs and camera_specs["fov"] is not None:
        sensor_bp.set_attribute("fov", str(camera_specs["fov"]))
    if "width" in camera_specs and camera_specs["width"] is not None:
        sensor_bp.set_attribute('image_size_x', str(camera_specs["width"]))
    if "height" in camera_specs and camera_specs["height"] is not None:
        sensor_bp.set_attribute('image_size_y', str(camera_specs["height"]))
    sensor = world.spawn_actor(sensor_bp, camera_transform, attach_to=vehicle,
                              attachment_type=carla.AttachmentType.Rigid)

    sensor.listen(lambda data: sensor_callback(data, sensor_queue, camera_id))
    sensor_list.append(sensor)
def spawn_semantic_lidar(world,vehicle,lidar_id,lidar_specs,sensor_list,sensor_queue):
    blueprint_library = world.get_blueprint_library()
    lidar_bp = blueprint_library.find('sensor.lidar.ray_cast_semantic')
    lidar_bp.set_attribute('rotation_frequency', str(lidar_specs['rotation_frequency']))
    lidar_bp.set_attribute('points_per_second', str(lidar_specs['points_per_second']))
    lidar_bp.set_attribute('channels', str(lidar_specs['channels']))
    lidar_bp.set_attribute('upper_fov', str(lidar_specs['upper_fov']))
    lidar_bp.set_attribute('lower_fov', str(lidar_specs['lower_fov']))
    lidar_bp.set_attribute('range', str(lidar_specs['range']))
    lidar_bp.set_attribute("horizontal_fov", str(lidar_specs['horizontal_fov']))

    lidar_location = carla.Location(x=lidar_specs['x'], y=lidar_specs['y'], z=lidar_specs['z'])
    lidar_rotation = carla.Rotation(pitch=lidar_specs['pitch'], roll=lidar_specs['roll'], yaw=lidar_specs['yaw'])
    lidar_transform = carla.Transform(lidar_location, lidar_rotation)

    lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=vehicle,
                                       attachment_type=carla.AttachmentType.Rigid)

    lidar.listen(lambda data: sensor_callback(data, sensor_queue, lidar_id))
    sensor_list.append(lidar)


def configure_traffic_manager(client, global_distance=2.0, global_sensitivity=0.5):
    """
    Configure the traffic manager settings for vehicle behavior in the simulation.

    :param client: Carla client object.
    :param global_distance: Global safe distance to leading vehicle.
    :param global_sensitivity: Global driving sensitivity.
    """
    # 获取交通管理器实例，默认端口8000
    traffic_manager = client.get_trafficmanager(8000)

    # 设置全局车辆间的安全距离
    traffic_manager.set_global_distance_to_leading_vehicle(global_distance)

    # 设置驾驶敏感度（0.0 = 最不敏感，1.0 = 最敏感）
    traffic_manager.global_percentage_speed_difference(global_sensitivity)
    return traffic_manager
def update_spectator_to_vehicle(world, vehicle, offset=carla.Location( z=2)):
    spectator = world.get_spectator()
    transform = vehicle.get_transform()
    spectator_transform = carla.Transform(transform.location + offset, transform.rotation)
    spectator.set_transform(spectator_transform)

def check_for_h_key():
    toggle = False
    for event in pygame.event.get():
        if event.type == pygame.KEYDOWN and event.key == pygame.K_h:
            toggle = True
    return toggle

def try_spawn_vehicle(world, blueprint, spawn_point, retries=5):
    for _ in range(retries):
        vehicle = world.try_spawn_actor(blueprint, spawn_point)
        if vehicle is not None:
            return vehicle
    return None

def main(map="Town01",weather=None):
    second_per_scene = 20
    save_frequency = 5 #save data at frequency 5hz
    init_pygame()
    ############ setup world  ###############################
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(20.0)
        tm = configure_traffic_manager(client)
        world = client.load_world(map)
        if weather != None:
            world.set_weather(weather)
        traffic_lights = world.get_actors().filter('traffic.traffic_light')
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.1
        world.apply_settings(settings)
    except Exception as e:
        print("Cannot find carla on port 2000")
        sys.exit(100)
    #################  spawn ego ########
    blueprint_library = world.get_blueprint_library()

    all_vehicle_blueprints = list(blueprint_library.filter('vehicle.*'))
    all_truck_blueprints = list(blueprint_library.filter('vehicle.carlamotors.firetruck')) + list(
        blueprint_library.filter('vehicle.carlamotors.european_hgv'))
    all_vehicle_blueprints = [bp for bp in all_vehicle_blueprints if bp not in all_truck_blueprints]

    random.shuffle(all_vehicle_blueprints)
    car_bp = blueprint_library.find('vehicle.tesla.model3')

    all_vehicles = []
    if not car_bp:
        print("Car blueprint 'vehicle.tesla.model3' not found.")
        pygame.quit()
        sys.exit(-1)

    spawn_point = random.choice(world.get_map().get_spawn_points())
    ego_vehicle = try_spawn_vehicle(world, car_bp, spawn_point)
    if not ego_vehicle:
        print("Failed to spawn main vehicle.")
        pygame.quit()
        return
    all_vehicles.append(ego_vehicle)
    print('Created %s' % ego_vehicle.type_id)
    #################### generate NPC #####################
    num_npcs = 15
    for _ in range(num_npcs):

        npc_blueprint = random.choice(blueprint_library.filter('vehicle.*'))


        location = spawn_point.location
        location.x += random.uniform(-5, 5)
        location.y += random.uniform(-5, 5)
        spawn_point.location = location

        npc = world.try_spawn_actor(npc_blueprint, spawn_point)
        if npc is not None:
            all_vehicles.append(npc)
            print(f"NPC spawned at {spawn_point.location}")
    if len(all_vehicles)<10:
        print("not enough npc!")
        sys.exit(1)
    for v in all_vehicles:
        print("set!")
        v.set_autopilot(True, tm.get_port())


    # set all traffic lights green
    for traffic_light in traffic_lights:
        traffic_light.set_state(carla.TrafficLightState.Green)
        traffic_light.freeze(True)
    sensor_list = []
    sensor_queue = Queue()

    ################# setup sensors ############
    sensor_data_frame = {}
    with open('sensor_setup.yaml', 'r') as file:
        data = yaml.safe_load(file)

        cam_specs = data['cam_specs']
        lidar_specs = data['lidar_specs']
        #
        for key, value in lidar_specs.items():
            spawn_semantic_lidar(world,ego_vehicle,key,value,sensor_list,sensor_queue)
        for key, value in cam_specs.items():
            spawn_camera(world,ego_vehicle,key,value,sensor_list,sensor_queue)
        bp_gnss = world.get_blueprint_library().find('sensor.other.gnss')
        gnss = world.spawn_actor(bp_gnss, carla.Transform(), attach_to=ego_vehicle,
                                       attachment_type=carla.AttachmentType.Rigid)
        gnss.listen(lambda data: sensor_callback(data, sensor_queue, "gnss"))
        sensor_list.append(gnss)

        # imu
        bp_imu = world.get_blueprint_library().find('sensor.other.imu')
        imu = world.spawn_actor(bp_imu, carla.Transform(), attach_to=ego_vehicle,
                                      attachment_type=carla.AttachmentType.Rigid)
        imu.listen(lambda data: sensor_callback(data, sensor_queue, "imu"))
        sensor_list.append(imu)

    ticks = 0
    tracking_enabled = True
    try:
        print("Start driving！！！")
        now = datetime.now()
        formatted_time = now.strftime('%Y_%m_%d_%H_%M_%S')
        while ticks < second_per_scene*10:

            world.tick()
            for i in range(0, len(sensor_list)):
                s_data = sensor_queue.get(block=True, timeout=10)
                sensor_data_frame[s_data[1]] = s_data[0]
            ticks += 1
            print("ticked once!")
            if check_for_h_key():
                tracking_enabled = not tracking_enabled
                print('Tracking toggled:', 'On' if tracking_enabled else 'Off')


            if tracking_enabled:
                update_spectator_to_vehicle(world, ego_vehicle)
            if ticks%5 == 0: # 2hz as NuScenes setup
                print("should save data now!!!")
                save_unit_data(sensor_data_frame,"./output"+ "/" + formatted_time + "/task0",ticks,lidar_specs,ego_vehicle)
        images_to_video("./output"+ "/" + formatted_time + "/task0/camera_video_purpose","./output"+ "/" + formatted_time + "/task0/task.mp4")
        print("finished this round!!")
        flag = True


    except KeyboardInterrupt:
        print('\nSimulation stopped by user.')
    except Exception as e:
        print(e)
    finally:
        print("generated {} vehicles".format(len(all_vehicles)))
        world = client.get_world()

        current_map = world.get_map().name

        world = client.load_world(str(current_map).split("/")[-1])
        pygame.quit()
        del client
        if flag:
            sys.exit(0)
        else:
            sys.exit(1)


def random_weather():

    cloudiness = random.uniform(0, 100)
    precipitation = random.uniform(0, 100)
    sun_altitude_angle = random.uniform(-90, 90)

    weather = carla.WeatherParameters(
        cloudiness=cloudiness,
        precipitation=precipitation,
        sun_altitude_angle=sun_altitude_angle
    )

    return weather
if __name__ == '__main__':
    #Towns = ["Town01_Opt", "Town02_Opt", "Town03_Opt", "Town04_Opt", "Town05_Opt","Town10HD_Opt"]
    Towns = ["Town01_Opt", "Town02_Opt", "Town03_Opt", "Town05_Opt","Town10HD_Opt"]
    random_map = Towns[random.randint(0, 4)]
    #weather = random_weather()
    clear_weather = carla.WeatherParameters(
    cloudiness=0.0,
    precipitation=0.0,
    sun_altitude_angle=90.0  )
    #main(random_map)
    main(random_map, clear_weather)