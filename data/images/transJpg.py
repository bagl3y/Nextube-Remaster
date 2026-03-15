import cv2 as cv
import os
import numpy as np


filenames = os.walk('./')

for path, dir_list, file_list in filenames:
    for file in file_list:
        if file.endswith('.jpg'):
            jpg_file_path = os.path.join(path, file)
            print(jpg_file_path)
            img = cv.imread(jpg_file_path)
            cv.imwrite(jpg_file_path, img)

