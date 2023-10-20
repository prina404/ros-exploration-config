import os
import cv2
import numpy as np
import matplotlib.pyplot as plt

def crop_small_plots(imgSrc: np.ndarray) -> np.ndarray:
    im = imgSrc.copy()

    im[im!=205] = 255
    im[im==205] = 0  

    coords = cv2.findNonZero(im) 
    x, y, w, h = cv2.boundingRect(coords) 
    rect = imgSrc[y:y+h, x:x+w] 
    rect = cv2.copyMakeBorder(rect, 40, 40, 40, 40, cv2.BORDER_CONSTANT, value=(205))

    return rect


def mergePictures():
    baseFolder = os.path.abspath('../output/')
    imgFolder = os.path.abspath('../worlds/img')
    completed_maps  = set()
    for dirPath, _, fileNames in os.walk(baseFolder):
        for f in fileNames:
            if f == 'Map.png':
                mapName = dirPath.split("/")[-3]
                completed_maps.add(mapName)
                run = dirPath.split("/")[-2]
                
                im = cv2.imread(os.path.join(dirPath, f), cv2.IMREAD_GRAYSCALE)
                source_im = cv2.imread(os.path.join(imgFolder, mapName + '.png'), cv2.IMREAD_GRAYSCALE)
                
                im = crop_small_plots(im)
                h1, w1 = source_im.shape
                im = cv2.resize(im, (w1, h1))
                h2, w2 = im.shape
                
                vis = np.zeros((max(h1, h2), w1+w2), np.uint8)
                vis[:h1, :w1] = source_im
                vis[:h2, w1:w1+w2] = im
                saveName = f'{mapName}_{run}.png'
                saveDir = os.path.abspath('./final_imgs')
                if saveName not in os.listdir(saveDir):
                	plt.imsave(os.path.join(saveDir, saveName), vis, cmap='gray')
    for name in os.listdir(baseFolder):
        if name not in completed_maps:
            print(f"DNF {name}")

if __name__ == "__main__":
    mergePictures()
