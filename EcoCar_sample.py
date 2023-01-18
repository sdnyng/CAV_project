import glob
import os
import sys
import time

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla
import random


def main():
    actor_list = []
    sensor_list = []
    try:
        # Setup Client
        client = carla.Client('localhost', 2000)
        client.set_timeout(20.0)
        world = client.get_world()
        # blueprint
        blueprint_library = world.get_blueprint_library()
        # setting for weather
        weather = carla.WeatherParameters(cloudiness=0,
                                          precipitation=0,
                                          fog_density=0,
                                          sun_altitude_angle=30.0)
        world.set_weather(weather)

        # create the ego vehicle
        ego_vehicle_bp = blueprint_library.find('vehicle.jeep.wrangler_rubicon')
        ego_vehicle_bp.set_attribute('color', '0, 0, 0')
        transform = random.choice(world.get_map().get_spawn_points())
        ego_vehicle = world.spawn_actor(ego_vehicle_bp, transform)
        # change the location
        location = ego_vehicle.get_location()
        location.x += 8
        location.y += 10
        ego_vehicle.set_location(location)
        # set the vehicle autopilot mode
        ego_vehicle.set_autopilot(True)
        # collect all actors
        actor_list.append(ego_vehicle)

        # LiDAR setting
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('channels', str(128))
        lidar_bp.set_attribute('points_per_second', str(160000))
        lidar_bp.set_attribute('rotation_frequency', str(10))
        lidar_bp.set_attribute('range', str(200))
        lidar_bp.set_attribute('dropoff_general_rate', str(0.15))
        lidar_bp.set_attribute('noise_stddev', str(0))
        # create output path
        output_path_lidar = '../outputs/lidar_output'
        if not os.path.exists(output_path_lidar):
            os.makedirs(output_path_lidar)
        lidar_location = carla.Location(0, 0, 2)
        lidar_rotation = carla.Rotation(0, 0, 0)
        lidar_transform = carla.Transform(lidar_location, lidar_rotation)
        # spawn the lidar
        lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=ego_vehicle)
        lidar.listen(lambda point_cloud: point_cloud.save_to_disk(os.path.join(output_path_lidar, '%06d.ply' % point_cloud.frame)))
        sensor_list.append(lidar)

        # stock camera
        front_camera_bp = blueprint_library.find('sensor.camera.rgb')
        back_camera_bp = blueprint_library.find('sensor.camera.rgb')
        # camera position
        front_camera_transform = carla.Transform(carla.Location(x=1, z=2))
        back_camera_transform = carla.Transform(carla.Location(x=-3, z=2),carla.Rotation(yaw=180))
        front_camera = world.spawn_actor(front_camera_bp, front_camera_transform, attach_to=ego_vehicle)
        back_camera = world.spawn_actor(back_camera_bp, back_camera_transform, attach_to=ego_vehicle)
        output_path_front_camera = '../outputs/front_camera_output'
        output_path_back_camera = '../outputs/back_camera_output'
        if not os.path.exists(output_path_front_camera):
            os.makedirs(output_path_front_camera)
        if not os.path.exists(output_path_back_camera):
            os.makedirs(output_path_back_camera)
        # set the callback function
        front_camera.listen(lambda image: image.save_to_disk(os.path.join(output_path_front_camera, '%06d.png' % image.frame)))
        back_camera.listen(lambda image: image.save_to_disk(os.path.join(output_path_back_camera, '%06d.png' % image.frame)))
        sensor_list.append(front_camera)
        sensor_list.append(back_camera)

        # stock radar
        shortrange_radar_bp = blueprint_library.find('sensor.other.radar')
        longrange_radar_bp = blueprint_library.find('sensor.other.radar')

        shortrange_radar_bp.set_attribute('range', str(3))
        shortrange_radar_bp.set_attribute('vertical_fov', str(45))
        longrange_radar_bp.set_attribute('range', str(200))
        longrange_radar_bp.set_attribute('vertical_fov', str(10))

        front_middle_radar_transform = carla.Transform(carla.Location(x=1, y=0, z=0.5))
        front_left_radar_transform = carla.Transform(carla.Location(x=1, y=-0.6, z=0.5),carla.Rotation(yaw=-15))
        front_right_radar_transform = carla.Transform(carla.Location(x=1, y=0.6, z=0.5),carla.Rotation(yaw=15))
        back_left_radar_transform = carla.Transform(carla.Location(x=-3, y=-0.6, z=0.5),carla.Rotation(yaw=-135))
        back_right_radar_transform = carla.Transform(carla.Location(x=-3, y=0.6, z=0.5),carla.Rotation(yaw=135))
        blindspot_left_radar_transform = carla.Transform(carla.Location(x=-2, y=-1, z=0.5),carla.Rotation(yaw=-90))
        blindspot_right_radar_transform = carla.Transform(carla.Location(x=-2, y=1, z=0.5),carla.Rotation(yaw=90))

        front_middle_radar = world.spawn_actor(longrange_radar_bp, front_middle_radar_transform, attach_to=ego_vehicle)
        front_left_radar = world.spawn_actor(shortrange_radar_bp, front_left_radar_transform, attach_to=ego_vehicle)
        front_right_radar = world.spawn_actor(shortrange_radar_bp, front_right_radar_transform, attach_to=ego_vehicle)
        back_left_radar = world.spawn_actor(shortrange_radar_bp, back_left_radar_transform, attach_to=ego_vehicle)
        back_right_radar = world.spawn_actor(shortrange_radar_bp, back_right_radar_transform, attach_to=ego_vehicle)
        blindspot_left_radar = world.spawn_actor(shortrange_radar_bp, blindspot_left_radar_transform, attach_to=ego_vehicle)
        blindspot_right_radar = world.spawn_actor(shortrange_radar_bp, blindspot_right_radar_transform, attach_to=ego_vehicle)

        sensor_list.append(front_middle_radar)
        sensor_list.append(front_left_radar)
        sensor_list.append(front_right_radar)
        sensor_list.append(back_left_radar)
        sensor_list.append(back_right_radar)
        sensor_list.append(blindspot_left_radar)
        sensor_list.append(blindspot_right_radar)

        # NPC
        npc_transform = random.choice(world.get_map().get_spawn_points())
        npc_transform.location += carla.Location(x=40, y=-3.2)
        npc_transform.rotation.yaw = -180.0
        for i in range(0, 30):
            npc_transform.location.x += 20.0
            bp = random.choice(blueprint_library.filter('vehicle'))
            # This time we are using try_spawn_actor. If the spot is already
            # occupied by another object, the function will return None.
            npc = world.try_spawn_actor(bp, npc_transform)
            if npc is not None:
                actor_list.append(npc)
                npc.set_autopilot(True)
                print('created %s' % npc.type_id)

        time.sleep(5)

        while True:
            # set the spectator to follow the ego vehicle
            spectator = world.get_spectator()
            transform = carla.Transform(ego_vehicle.get_transform().transform(carla.Location(x=-4.5, z=2.5)),
                                        ego_vehicle.get_transform().rotation)
            spectator.set_transform(transform)


    finally:

        print('destroying actors')
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
        for sensor in sensor_list:
            sensor.destroy()
        print('done.')


if __name__ == '__main__':
    main()
