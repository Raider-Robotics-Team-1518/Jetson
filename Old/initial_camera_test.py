import numpy as np
import cv2

'''
This test program will work with a USB camera but not the on-board TX2 camera
Determine the device with `ls /dev/video*` before and after plugging in the USB
camera, then updating the video# below with the correct USB camera device number.
'''
cap = cv2.VideoCapture("/dev/video1") # check this
while cv2.waitKey(1) != 27:
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Display the resulting frame
    cv2.imshow('frame',gray)

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
