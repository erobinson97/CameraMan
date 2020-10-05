# Created by Ethan Robinson
# Created on October 5th, 2020
#
# This is some test code that tracks an object using my laptop webcam.
#
# Coded using OpenCV 4.4.0

import cv2
# import pdb; pdb.set_trace()
import faulthandler


def drawBox(img, bbox):
    x = int(bbox[0])
    y = int(bbox[1])
    w = int(bbox[2])
    h = int(bbox[3])
    cv2.rectangle(img, (x, y), ((x+w), (y+h)), (255, 0, 255), 3, 1)
    cv2.putText(img, "Tracking!", (75, 75),
                cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 255, 0), 2)


cap = cv2.VideoCapture(0)
faulthandler.enable()
# Coment one out and uncomment another to switch between trackers
# track = cv2.TrackerMOSSE_create()
track = cv2.TrackerCSRT_create()
# track = cv2.TrackerGOTURN_create()


cv2.namedWindow("Tracking", cv2.WINDOW_GUI_NORMAL)

# This grabbs the first frame
grabbed, frame1 = cap.read()

if(grabbed):
    cv2.imshow("Tracking", frame1)

    # Lets the user draw a box on the window
    bbox = cv2.selectROI("Tracking", frame1, False)
    track.init(frame1, bbox)


# Starts the while loop if the first frame was captured
while grabbed:

    # Grabs, decodes and returns the next video frame. Frame is stored in 'img'
    success, img = cap.read()
    cv2.imshow("Tracking", img)

    success, bbox = track.update(img)

    if success:
        drawBox(img, bbox)
    else:
        cv2.putText(img, "OBJECT_LOST!", (75, 75),
                    cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 0, 225), 2)

    # Create a window and show the captured image
    cv2.imshow("Tracking", img)

    # Press 'Q' to exit the window
    if cv2.waitKey(1) & 0xff == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
