import cv2
import sys
import numpy as np

INP_POINTS = [[260, 290], [490, 290], [3, 467], [710, 467]]


def displayPoints(inp_points, image):
    for k in inp_points:
        cv2.circle(image, k, 2, (250, 0, 0), 3)
    return image


def getTransformedImage(image, inp_points):
    height, width = image.shape[:2]
    dst_points = np.float32([[0, 0], [width, 0], [0, height], [height, width]])
    TRANSFORMATION = cv2.getPerspectiveTransform(np.float32(inp_points), dst_points)
    TRANSFORMED_IMAGE = cv2.warpPerspective(image, TRANSFORMATION, (width, height))
    return {'IMAGE': TRANSFORMED_IMAGE, 'TRANSFORMATION': TRANSFORMATION}

