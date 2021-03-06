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
from inv_perspective_transform import  displayPoints, getTransformedImage

# ==============================================================================
# -- PACKAGES IMPORTED---------------------------------------------------------
# ==============================================================================
# INP_POINTS = [[220, 270], [550, 270], [-400, 480], [1120, 480]]

INP_POINTS = [[130, 270], [770, 270], [-600, 500], [1470, 500]]

OBJECT = [[330, 238], [386, 280]]
#

PXS_PER_METER = 29.923
actors = []
IM_HEIGHT = 480
IM_WIDTH = 720
FOV = 110

cv2.setNumThreads(10)


def transformed_points(matrix, points):
    p_x = (matrix[0][0] * points[0] + matrix[0][1] * points[1] + matrix[0][2]) / (
          (matrix[2][0] * points[0] + matrix[2][1] * points[1] + matrix[2][2]))
    p_y = (matrix[1][0] * points[0] + matrix[1][1] * points[1] + matrix[1][2]) / (
          (matrix[2][0] * points[0] + matrix[2][1] * points[1] + matrix[2][2]))
    return [p_x, p_y]


def distance_formula(length, px_per_meter):
    distance = length//px_per_meter
    return distance


def processing(img_data):
    image = np.array(img_data.raw_data)
    image = image.reshape((IM_HEIGHT, IM_WIDTH, 4))
    image = np.array(image[:,:,:3])
    cv2.imshow('ORIGINAL RGB DATA', image)
    cv2.rectangle(image, OBJECT[0], OBJECT[1], (0, 0, 150), 2)
    marked_image = displayPoints(INP_POINTS, image)
    transformed = getTransformedImage(image, INP_POINTS)
    up_pts = transformed_points(transformed['TRANSFORMATION'], OBJECT[1])
    cv2.imshow('MARKED IMAGE', marked_image)
    cv2.circle(transformed['IMAGE'], [int(up_pts[0]), int(up_pts[1])], 2, (0, 255, 0), 2)
    length = 480 - int(up_pts[1])
    print('length: ', length, ' px_per_meter: ', PXS_PER_METER,
          f' distance: {distance_formula(length, PXS_PER_METER)} m', )
    cv2.line(transformed['IMAGE'], [int(up_pts[0]), IM_HEIGHT], [int(up_pts[0]), int(up_pts[1])], (155, 20, 55), 2)
    cv2.imshow("TRANSFORMED IMAGE", transformed['IMAGE'])
    cv2.waitKey(1)


def destroy_all_actors(actors):
    for actor in actors:
        actor.destroy()

try:
    client = carla.Client('localhost', 2000)
    client.set_timeout(20)
    world = client.get_world()
    blueprints = world.get_blueprint_library()
    current_map = world.get_map()
    spawn_points = current_map.get_spawn_points()
    if current_map.name != 'Town01':
        client.load_world('Town01')
        time.sleep(2)
    # Blueprints Used
    vehicle_blueprint = blueprints.filter('vehicle.tesla.model3')[0]
    vehicle_2_blueprint = blueprints.filter('vehicle.chevrolet.impala')[0]
    camera_rgb_blueprint = blueprints.filter('sensor.camera.rgb')[0]
    camera_rgb_blueprint.set_attribute('image_size_x', f'{IM_WIDTH}')
    camera_rgb_blueprint.set_attribute('image_size_y', f'{IM_HEIGHT}')
    camera_rgb_blueprint.set_attribute('fov', f'{FOV}')
    vehicle = world.spawn_actor(vehicle_blueprint, spawn_points[5])
    actors.append(vehicle)
    # Sensors In CAPS
    CAMERA_RGB = world.spawn_actor(
        camera_rgb_blueprint,
        carla.Transform(carla.Location(x=0.8, y=0.0, z=1.7)),
        attach_to=vehicle
    )
    CAMERA_RGB.listen(lambda data: processing(data))
    actors.append(CAMERA_RGB)
    vehicle.set_autopilot(False)
    time.sleep(120)


except Exception as e:
    print('Error: ',e)
finally:
    destroy_all_actors(actors)