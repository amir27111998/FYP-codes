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
from inv_perspective_transform import INP_POINTS, displayPoints, getTransformedImage

# ==============================================================================
# -- PACKAGES IMPORTED---------------------------------------------------------
# ==============================================================================





actors = []
IM_HEIGHT = 480
IM_WIDTH = 720
FOV = 110

cv2.setNumThreads(10)

def processing(img_data):
    image = np.array(img_data.raw_data)
    image = image.reshape((IM_HEIGHT, IM_WIDTH, 4))
    image = np.array(image[:,:,:3])
    marked_image = displayPoints(INP_POINTS, image)
    transformed_image = getTransformedImage(image, INP_POINTS)
    cv2.imshow('ORIGINAL RGB DATA', image)
    cv2.imshow('MARKED IMAGE', marked_image)
    cv2.imshow("TRANSFORMED IMAGE", transformed_image)
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
    vehicle = world.spawn_actor(vehicle_blueprint, spawn_points[7])
    actors.append(vehicle)
    # Sensors In CAPS
    CAMERA_RGB = world.spawn_actor(
        camera_rgb_blueprint,
        carla.Transform(carla.Location(x=0.8, y=0.0, z=1.7)),
        attach_to=vehicle
    )
    CAMERA_RGB.listen(lambda data : processing(data))
    actors.append(CAMERA_RGB)
    vehicle.set_autopilot(True)
    time.sleep(50)


except Exception as e:
    print('Error: ',e)
finally:
    for actor in actors:
        actor.destroy()