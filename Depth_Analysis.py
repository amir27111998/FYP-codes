import sys
import glob
import os
import cv2

# ==============================================================================
# -- Find CARLA module ---------------------------------------------------------
# ==============================================================================
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
from carla import ColorConverter as cc
import time
import numpy as np

actors = []
IM_HEIGHT = 480
IM_WIDTH = 720
FOV = 110


def processing(img_data):
    image = np.array(img_data.raw_data)
    image = image.reshape((IM_HEIGHT, IM_WIDTH, 4))
    image = image[:,:,:3]
    cv2.imshow('RGB DATA', image)
    cv2.waitKey(1)

count = 0


def processing_depth(image):
    global count
    image.save_to_disk(f'../_out/{count}_left.png', cc.Depth)
    image.convert(cc.LogarithmicDepth)
    image = np.array(image.raw_data)
    image = image.reshape((IM_HEIGHT, IM_WIDTH, 4))
    image = image[:, :, :3]
    count += 1
    cv2.imshow('DEPTH DATA', image)
    cv2.waitKey(1)

try:
    client = carla.Client('localhost', 2000)
    client.set_timeout(20)
    world = client.get_world()
    blueprints = world.get_blueprint_library()
    current_map = world.get_map()
    spawn_points = current_map.get_spawn_points()
    if current_map.name != 'Town01':
        client.load_world('Town01')
    # Blueprints Used
    vehicle_blueprint = blueprints.filter('vehicle.tesla.model3')[0]
    camera_rgb_blueprint = blueprints.filter('sensor.camera.rgb')[0]
    camera_rgb_blueprint.set_attribute('image_size_x', f'{IM_WIDTH}')
    camera_rgb_blueprint.set_attribute('image_size_y', f'{IM_HEIGHT}')
    camera_rgb_blueprint.set_attribute('fov', f'{FOV}')
    camera_depth_blueprint = blueprints.filter('sensor.camera.depth')[0]
    camera_depth_blueprint.set_attribute('image_size_x', f'{IM_WIDTH}')
    camera_depth_blueprint.set_attribute('image_size_y', f'{IM_HEIGHT}')
    camera_depth_blueprint.set_attribute('fov', f'{FOV}')
    vehicle = world.spawn_actor(vehicle_blueprint, spawn_points[1])
    actors.append(vehicle)
    # Sensors In CAPS
    CAMERA_RGB = world.spawn_actor(
        camera_rgb_blueprint,
        carla.Transform(carla.Location(x=2.0, y=0.0, z=1.2)),
        attach_to=vehicle
    )
    CAMERA_RGB.listen(lambda data : processing(data))
    actors.append(CAMERA_RGB)

    CAMERA_DEPTH = world.spawn_actor(
        camera_depth_blueprint,
        carla.Transform(carla.Location(x=2.0, y=0.0, z=1.2)),
        attach_to=vehicle
    )
    CAMERA_DEPTH.listen(lambda data: processing_depth(data))
    actors.append(CAMERA_DEPTH)
    vehicle.set_autopilot(True)
    time.sleep(10)
except Exception as e:
    print('Error: ',e)
finally:
    for actor in actors:
        actor.destroy()