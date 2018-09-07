import numpy as np
import cv2
import detect_dice_per_frame as ddpf

cap = cv2.VideoCapture('/home/mark/Desktop/raw_dice_04.avi')
detector = ddpf.DetectDicePerFrame()

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    detector.draw_max_box_on_dice(frame)

    # Display the resulting frame
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()