# This class is a wrapper for the camera which adds threading functionality.
# See https://www.pyimagesearch.com/2015/12/21/increasing-webcam-fps-with-python-and-opencv/ for more details.
from ThreadedWebcam import ThreadedWebcam
from threading import Thread
import cv2 as cv
import time

class ThreadedBlob:
	def __init__(self, src=0, name="ThreadedBlob"):
		self.name = name
		self.stopped = False
		print("Starting Camera")
		self.camera = ThreadedWebcam()
		self.camera.start()

		params = cv.SimpleBlobDetector_Params()
		self.detector = cv.SimpleBlobDetector_create(params)
		fs = cv.FileStorage("params.yaml", cv.FILE_STORAGE_READ); #yaml, xml, or json
		if fs.isOpened():
		    self.detector.read(fs.root())
		else:
		    print("WARNING: params file not found! Creating default file.")

		    fs2 = cv.FileStorage("params.yaml", cv.FILE_STORAGE_WRITE)
		    detector.write(fs2)
		    fs2.release()

		fs.release()

        self.minH =  158; self.minS = 78; self.minV =  98;
		self.maxH = 180; self.maxS = 255; self.maxV = 255;



	# Starts the camera thread.
	def start(self):
		t = Thread(target=self._update, name=self.name, args=())
		t.daemon = True
		t.start()
		return self

	# Returns the latest camera frame.
	def read(self):
		return self.keypoints

	# Stops the camera thread.
	def stop(self):
		self.stopped = True
		self.camera.stop()

	# Private function that constantly reads the camera stream.
	# Do not call this function externally.
	def _update(self):

		while not self.stopped:
			frame = self.camera.read()

			# Blob detection works better in the HSV color space
			# (than the RGB color space) so the frame is converted to HSV.
			frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

			# Create a mask using the given HSV range
			mask = cv.inRange(frame_hsv, (self.minH, self.minS, self.minV), (self.maxH, self.maxS, self.maxV))

			# Run the SimpleBlobDetector on the mask.
			# The results are stored in a vector of 'KeyPoint' objects,
			# which describe the location and size of the blobs.
			self.keypoints = self.detector.detect(mask)
