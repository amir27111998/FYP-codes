import sys
import glob
import os
import cv2
import math
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
# INP_POINTS = [[0, 280], [720, 280], [-2800, 650], [2200, 650]]

INP_POINTS = [[0, 280], [720, 280], [-5500, 1000], [3700, 1000]]

OBJECT = [[330, 248], [380, 280]]
# ADD 20

PXS_PER_METER = 30
actors = []
IM_HEIGHT = 480
IM_WIDTH = 720
FOV = 110

STATIC_DISTANCES = [50, 40, 30, 20, 15, 10, 5, 3, 2, 1, 0.4, 0.2]
BRAKES =           [0.04, 0.06, 0.02, 0.01, 0.01, 0.04, 0.06, 0.02, 0.01, 0.01, 0.01, 0.01]
CO=0

cv2.setNumThreads(10)


def transformed_points(matrix, points):
    p_x = (matrix[0][0] * points[0] + matrix[0][1] * points[1] + matrix[0][2]) / (
          (matrix[2][0] * points[0] + matrix[2][1] * points[1] + matrix[2][2]))
    p_y = (matrix[1][0] * points[0] + matrix[1][1] * points[1] + matrix[1][2]) / (
          (matrix[2][0] * points[0] + matrix[2][1] * points[1] + matrix[2][2]))
    return [p_x, p_y]


def distance_formula(length, px_per_meter):
    distance = length/px_per_meter
    return distance


def actual_distance(vehicle, obj2):
    vehicle_location = vehicle.get_location()
    object_location = obj2.get_location()
    formula = math.sqrt((vehicle_location.x - object_location.x) ** 2 +
                        (vehicle_location.y - object_location.y) ** 2 +
                        (vehicle_location.z - object_location.z) ** 2)
    return formula

def decision_making(distance, status, current_velocity):
    if status:
        print('Distance: ', distance)
        target_velocity=carla.Vector3D(5,5,0)
        if distance < 10.1 and distance > 5.0:
            target_velocity = current_velocity*0.50
            print('Less than 10 m')
        elif distance < 5.1 and distance > 2.0:
            target_velocity = current_velocity*0.65
            print('Less than 5 m')
        elif distance < 2.1 and distance > 1.5:
            target_velocity = current_velocity*0.80
            print('Less than 2 m')
        elif distance < 1.6 and distance > 1.0:
            target_velocity = current_velocity * 0.85
            print('Less than 1.5 m')
        elif distance < 1.1 and distance > 0:
            target_velocity = current_velocity * 0.94
            print('Less than 1 m')
        return target_velocity


def processing(img_data, vehicle):
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
    length = IM_HEIGHT - int(up_pts[1])
    # print('length: ', length, ' px_per_meter: ', PXS_PER_METER,
    #       f' distance: {distance_formula(length, PXS_PER_METER)} m', )
    cv2.line(transformed['IMAGE'], [int(up_pts[0]), IM_HEIGHT], [int(up_pts[0]), int(up_pts[1])], (155, 20, 55), 2)
    cv2.imshow("TRANSFORMED IMAGE", transformed['IMAGE'])
    cv2.waitKey(1)


def destroy_all_actors(actors):
    for actor in actors:
        actor.destroy()

try:
    client = carla.Client('localhost', 2000)
    client.set_timeout(10)
    world = client.get_world()
    blueprints = world.get_blueprint_library()
    current_map = world.get_map()
    spawn_points = current_map.get_spawn_points()
    if current_map.name != 'Town01':
        client.load_world('Town01')
        time.sleep(2)
    # Blueprints Used
    controls = carla.VehicleControl()
    vehicle_blueprint = blueprints.filter('vehicle.tesla.model3')[0]
    vehicle_2_blueprint = blueprints.filter('vehicle.chevrolet.impala')[0]
    camera_rgb_blueprint = blueprints.filter('sensor.camera.rgb')[0]
    camera_rgb_blueprint.set_attribute('image_size_x', f'{IM_WIDTH}')
    camera_rgb_blueprint.set_attribute('image_size_y', f'{IM_HEIGHT}')
    camera_rgb_blueprint.set_attribute('fov', f'{FOV}')
    vehicle = world.spawn_actor(vehicle_blueprint, spawn_points[3])
    actors.append(vehicle)
    # Sensors In CAPS
    CAMERA_RGB = world.spawn_actor(
        camera_rgb_blueprint,
        carla.Transform(carla.Location(x=1.6, z=1.7)),
        attach_to=vehicle
    )
    CAMERA_RGB.listen(lambda data: processing(data, vehicle))
    actors.append(CAMERA_RGB)
    # vehicle.set_autopilot(True)
    start=0
    # controls.brake=0.8
    # vehicle.apply_control(controls)
    # vehicle.set_target_velocity(carla.Vector3D(0,5,0))
    # brake = 0.1

    controls.throttle = 0.8
    while start <= 30:
        # if start >= 15:
        #     controls.brake = 0
        #     # controls.reverse = True
        #     controls.steer = 0
        #     vehicle.set_target_velocity(carla.Vector3D(-1, -1, -1))
        #
        # vehicle.apply_control(controls)
        # print(decision_making(STATIC_DISTANCES[CO], 1, vehicle.get_velocity()))
        # vehicle.set_target_velocity(decision_making(STATIC_DISTANCES[CO], 1, vehicle.get_velocity()))
        # if start%10==0:
        #     controls.brake=BRAKES[CO]

        vehicle.apply_control(controls)
        time.sleep(1)
        start=start+1
        controls.throttle = controls.throttle*0.60
        CO = (CO + 1) % len(STATIC_DISTANCES)
        # print(vehicle.get_velocity())
    # time.sleep(15)
    # vehicle.set_target_velocity(carla.Vector3D(2,2,0))
    # time.sleep(20)



except Exception as e:
    print('Error: ',e)
finally:
    destroy_all_actors(actors)