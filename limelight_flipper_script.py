import cv2
import numpy as np

LINE_THICKNESS = 5
FLIP_NUMBER_THRESHOLD = 0
    
# runPipeline() is called every frame by Limelight's backend.
def runPipeline(image, llrobot): #image is BGR, llrobot is a double array
    if llrobot[0] > FLIP_NUMBER_THRESHOLD:
        image = cv2.flip(image, 0)
    
    height, width, _ = image.shape
    mid_x = width // 2
    cv2.line(image, (mid_x - (LINE_THICKNESS // 2), 0), (mid_x - (LINE_THICKNESS // 2), height), (0, 0, 255), LINE_THICKNESS)
  
    largestContour = np.array([[]])
    llpython = []
    return largestContour, image, llpython