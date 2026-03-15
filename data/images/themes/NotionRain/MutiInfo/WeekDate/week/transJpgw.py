import cv2 as cv
from glob import glob
import os

import numpy as np


files = glob('./*.jpg')
print(files)

shapes = []

for file in files:
    img = cv.imread(file)
    shapes.append(img.shape)
    path, filename = os.path.split(file)
    width  = 76
    height = 37
    dim = (width, height)
    cv.imwrite(os.path.join("output", filename), cv.resize(img, dim, interpolation = cv.INTER_AREA))
    
