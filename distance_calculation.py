import numpy as np
import cv2
image = cv2.imread('../_out/7_left.png')
FACTOR = 20
x=(300,350)
y=(200,240)
pothole = image[y[0]:y[1],x[0]:x[1]]
normalized_pothole = (pothole[-1,:,-1] + pothole[-1,:,-2] * 256 + pothole[-1,:,0] * 256 * 256) / (256 * 256 * 256 - 1)
in_meters_pothole = 1000 * normalized_pothole
print(pothole.shape)
cv2.rectangle(image, (x[0],y[0]),(x[1],y[1]),(255,0,0),2)
# cv2.reprojectImageTo3D()
# cv2.waitKey(20000)


m=(300,350)
n=(450,475)
car = image[n[0]:n[1],m[0]:m[1]]
print(car.shape)
cv2.rectangle(image, (m[0],n[0]),(m[1],n[1]),(0,0,255),2)

normalized_car = (car[0,:,-1] + car[0,:,-2] * 256 + car[0,:,0] * 256 * 256) / (256 * 256 * 256 - 1)
in_meters_car = 1000 * normalized_car
# print(in_meters_car)
# print(in_meters_pothole)
diff = abs(pothole.shape[1]-car.shape[1])
zeros = np.zeros((diff))

if pothole.shape[1]>car.shape[1]:
    in_meters_car = np.concatenate((in_meters_car, zeros), axis=0)
elif pothole.shape[1]<car.shape[1]:
    in_meters_pothole = np.concatenate((in_meters_pothole, zeros), axis=0)
# print(min(np.abs(in_meters_pothole-in_meters_car)))
print('Pothole Distance: ',min(np.abs(in_meters_pothole)))

cv2.imshow('ImageOrg',image)
cv2.imshow('Image_converted',image[y[0]:y[1],x[0]:x[1]])
cv2.imshow('Image_2',image[n[0]:n[1],m[0]:m[1]])
cv2.waitKey(4000)











