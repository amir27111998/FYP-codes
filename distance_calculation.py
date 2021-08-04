import numpy as np
import cv2
from inv_perspective_transform import displayPoints
image = cv2.imread('./save2.jpg')
image = cv2.resize(image, (720,480))
FACTOR = 20
x=(320,475)
y=(150,320)
INP_POINTS = [[130, 270], [770, 270], [-600, 500], [1470, 500]]
def getTransformedImage(image, inp_points):
    height, width = image.shape[:2]
    dst_points = np.float32([[0, 0], [width, 0], [0, height], [width, height]])
    TRANSFORMATION = cv2.getPerspectiveTransform(np.float32(inp_points), dst_points)
    TRANSFORMED_IMAGE = cv2.warpPerspective(image, TRANSFORMATION, (width, height))
    return TRANSFORMED_IMAGE


cv2.rectangle(image, (x[0],y[0]),(x[1],y[1]),(255,0,0),2)

cv2.imshow('ORIGINAL IMAGE',image)
points = displayPoints(INP_POINTS, image)
transform_image = getTransformedImage(image, INP_POINTS)
cv2.imshow('IMAGE POINTS',points)
cv2.imshow('IMAGE TRANSFORMED',transform_image)
cv2.waitKey(0)











