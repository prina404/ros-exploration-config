import os
import numpy as np
import cv2

if __name__=='__main__':
    basePath = os.path.abspath('../worlds/img')
    print(f"proccessing imgs in: {basePath}")
    for file in os.listdir(basePath):
        if not file.endswith('.png'):
                continue
        im = cv2.imread(os.path.join(basePath, file), cv2.IMREAD_GRAYSCALE)
        im = im[1:-1,1:-1]
        im = cv2.copyMakeBorder(im, 1, 1, 1, 1, cv2.BORDER_CONSTANT, value=0)
        cv2.imwrite(os.path.join(basePath, file), im)
