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
import matplotlib.pyplot as plt

actors = []
IM_HEIGHT = 480
IM_WIDTH = 720
FOV = 110
queue = []


count = 0


def create_depth_map(left, right):
    left = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
    right = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
    # Stereo SGBM matcher
    left_matcher_SGBM = cv2.StereoSGBM_create(minDisparity=0,
                                              numDisparities=1 * 16,
                                              blockSize=11,
                                              P1=8 * 3 * 6 ** 2,
                                              P2=32 * 3 * 6 ** 2,
                                              mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)

    disp_left = (left_matcher_SGBM.compute(left, right).astype(np.float32) / 16)*255
    cv2.imshow('Disparity', disp_left)
    # plt.imshow(disp_left)
    # plt.show()


def displaying_camera_image(img, name, prev_img, prev_name, count):

    img.save_to_disk(f'../_out/{name}/{count}.jpg')
    prev_img.save_to_disk(f'../_out/{prev_name}/{count}.jpg')
    image = np.array(img.raw_data)
    image = image.reshape((IM_HEIGHT, IM_WIDTH, 4))
    image = image[:, :, :3]

    prev_image = np.array(prev_img.raw_data)
    prev_image = prev_image.reshape((IM_HEIGHT, IM_WIDTH, 4))
    prev_image = prev_image[:, :, :3]
    # print(image, prev_image)
    cv2.imshow('RGB DATA '+name.upper(), image)
    cv2.imshow('RGB DATA '+prev_name.upper(), prev_image)
    create_depth_map(image,prev_image)
    cv2.waitKey(1)


# def processing(image):
#     global count
#     print(image.frame_number)
    # check=os.path.exists(f'../_out/right/{count}.jpg') and \
    #       os.path.exists(f'../_out/left/{count}.jpg')
    # if check:
    #     count+=1
    # saving_camera_image(image, 'left', count)


def processing(image,name):
    global count
    if len(queue) > 0:
        prev_data = queue.pop(0)
        prev_image = prev_data[0]
        prev_name = prev_data[1]
        # check = os.path.exists(f'../_out/right/{count}.jpg') and \
        #         os.path.exists(f'../_out/left/{count}.jpg')
        # if check:
        #     count += 1
        # saving_camera_image(image, 'right', count)
        if prev_image.frame_number==image.frame_number:
            # saving_camera_image(image, name, count)
            # saving_camera_image(prev_image, prev_name, count)
            displaying_camera_image(image, name, prev_image, prev_name, count)
            # print('Amir')
            count=count+1
        # print('Result : ', previmage.frame_number==image.frame_number, end='\n')
    queue.append((image,name))




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
    # LEFT CAMERA
    CAMERA_RGB_LEFT = world.spawn_actor(
        camera_rgb_blueprint,
        carla.Transform(carla.Location(x=2.0, y=0, z=1.0)),
        attach_to=vehicle
    )
    CAMERA_RGB_LEFT.listen(lambda data : processing(data,'left'))
    actors.append(CAMERA_RGB_LEFT)

    # RIGHT CAMERA
    CAMERA_RGB_RIGHT = world.spawn_actor(
        camera_rgb_blueprint,
        carla.Transform(carla.Location(x=2.0, y=0.5, z=1.0)),
        attach_to=vehicle
    )
    CAMERA_RGB_RIGHT.listen(lambda data: processing(data, 'right'))
    actors.append(CAMERA_RGB_RIGHT)
    vehicle.set_autopilot(True)
    time.sleep(0.5)

except Exception as e:
    print('Error: ',e)
except SystemExit as x:
    print('System Error: ',x)
finally:
    print(len(queue))
    for actor in actors:
        print(actor)
        actor.destroy()
    print('All Destroyed')
    cv2.destroyAllWindows()