# This program demonstrates advanced usage of the OpenCV library by
# using the SimpleBlobDetector feature along with camera threading.
# The program displays two windows: one for adjusting the mask,
# and one that displays the detected blobs in the (masked) image.
# Adjust the HSV values until blobs are detected from the camera feed.
# There's also a params file in the same folder that can be adjusted.

# Helpful links:
# https://www.learnopencv.com/blob-detection-using-opencv-python-c/
# https://docs.opencv.org/3.4.1/da/d97/tutorial_threshold_inRange.html
# https://docs.opencv.org/3.4.1/d0/d7a/classcv_1_1SimpleBlobDetector.html
# https://docs.opencv.org/3.4.1/d2/d29/classcv_1_1KeyPoint.html
# https://www.pyimagesearch.com/2015/12/21/increasing-webcam-fps-with-python-and-opencv/

#x 0-640
#y 0-480

import cv2 as cv
import time
from ThreadedWebcam import ThreadedWebcam


FPS_SMOOTHING = 0.9

# Window names
WINDOW1 = "Adjustable Mask - Press Esc to quit"
WINDOW2 = "Detected Blobs - Press Esc to quit"

# Default HSV ranges
# Note: the range for hue is 0-180, not 0-255
minH =  158; minS = 78; minV =  98;
maxH = 180; maxS = 255; maxV = 255;


# Initialize the threaded camera
# You can run the unthreaded camera instead by changing the line below.
# Look for any differences in frame rate and latency.
camera = ThreadedWebcam() # UnthreadedWebcam()
camera.start()

# Initialize the SimpleBlobDetector
params = cv.SimpleBlobDetector_Params()
detector = cv.SimpleBlobDetector_create(params)

# Attempt to open a SimpleBlobDetector parameters file if it exists,
# Otherwise, one will be generated.
# These values WILL need to be adjusted for accurate and fast blob detection.
fs = cv.FileStorage("params.yaml", cv.FILE_STORAGE_READ); #yaml, xml, or json
if fs.isOpened():
    detector.read(fs.root())
else:
    print("WARNING: params file not found! Creating default file.")

    fs2 = cv.FileStorage("params.yaml", cv.FILE_STORAGE_WRITE)
    detector.write(fs2)
    fs2.release()

fs.release()

# Create windows
cv.namedWindow(WINDOW1)
cv.namedWindow(WINDOW2)

# Create trackbars


fps, prev = 0.0, 0.0
while True:
    # Calculate FPS
    now = time.time()
    fps = (fps*FPS_SMOOTHING + (1/(now - prev))*(1.0 - FPS_SMOOTHING))
    prev = now

    # Get a frame
    frame = camera.read()

    # Blob detection works better in the HSV color space
    # (than the RGB color space) so the frame is converted to HSV.
    frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    # Create a mask using the given HSV range
    minH =  158; minS = 78; minV =  98;
    maxH = 180; maxS = 255; maxV = 255;
    mask = cv.inRange(frame_hsv, (minH, minS, minV), (maxH, maxS, maxV))

    # Run the SimpleBlobDetector on the mask.
    # The results are stored in a vector of 'KeyPoint' objects,
    # which describe the location and size of the blobs.
    keypoints = detector.detect(mask)



    for i in keypoints:
        print(i.pt,i.size)

    # For each detected blob, draw a circle on the frame
    frame_with_keypoints = cv.drawKeypoints(mask, keypoints, None, color = (0, 255, 0), flags = cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # Write text onto the frame
    cv.putText(frame_with_keypoints, "FPS: {:.1f}".format(fps), (5, 15), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0))
    cv.putText(frame_with_keypoints, "{} blobs".format(len(keypoints)), (5, 35), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0))

    # Display the frame
    cv.imshow(WINDOW1, frame)
    cv.imshow(WINDOW2, frame_with_keypoints)

    # Check for user input
    c = cv.waitKey(1)
    if c == 27 or c == ord('q') or c == ord('Q'): # Esc or Q
        camera.stop()
        break

camera.stop()
