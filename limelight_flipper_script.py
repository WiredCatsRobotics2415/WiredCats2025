import cv2
import numpy as np

LINE_THICKNESS = 5
FLIP_NUMBER_THRESHOLD = 0
    
# runPipeline() is called every frame by Limelight's backend.
def runPipeline(image, llrobot): #image is BGR, llrobot is a list of doubles
    if llrobot[0] > FLIP_NUMBER_THRESHOLD:
        image = cv2.flip(image, 0)
    
    height, width, _ = image.shape
    mid_x = width // 2
    cv2.line(image, (mid_x - (LINE_THICKNESS // 2), 0), (mid_x - (LINE_THICKNESS // 2), height), (0, 0, 255), LINE_THICKNESS)

    llpython = [round(frame.mean())]
    
    largestContour = np.array([[]])
    return largestContour, image, llpython

if __name__ == '__main__':
    vc = cv2.VideoCapture(0)
    if not vc.isOpened():
        print("Cannot open camera")
        exit()
    flip_key = 0
    while True:
        ret, frame = vc.read()
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        key = cv2.waitKey(1)
        if key == ord('q'):
            break
        if key == ord('o'):
            flip_key = FLIP_NUMBER_THRESHOLD-1
        if key == ord('l'):
            flip_key = FLIP_NUMBER_THRESHOLD+1
        llrobot = [flip_key]

        _, image, llpython = runPipeline(frame, llrobot)
        cv2.imshow("Image", image)

        print(llpython)