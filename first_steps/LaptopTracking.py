import cv2

cap = cv2.VideoCapture(0)

# Depreciated module
# tracker = cv2.TrackerMOSSE_create()

tracker = cv2.TrackerMOSSE()
cv2.namedWindow("Tracking", cv2.WINDOW_GUI_NORMAL)

# This grabbs the first frame
grabbed, frame1 = cap.read()

if(grabbed):
    cv2.imshow("Tracking", frame1)

    # Lets the user draw a box on the window
    bbox = cv2.selectROI("Tracking", frame1, False)
    cv2.Tracker(frame1, bbox)


while grabbed:
    timer = cv2.getTickCount()

    success, img = cap.read()

    fps = cv2.getTickFrequency()/(cv2.getTickCount() - timer)

    cv2.putText(img, str(fps), (75, 50), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0,0,225),2)

    # Create a window and show the captured image
    cv2.imshow("Tracking", img)

    # Press 'Q' to exit the window
    if cv2.waitKey(1) & 0xff ==ord('q'):
        break

cap.release()
cv2.destroyAllWindows()