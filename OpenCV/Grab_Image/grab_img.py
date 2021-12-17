#!/usr/bin/python3
from time import ctime
import cv2


def main():
    cap = cv2.VideoCapture(0)
    cap.set(3, 1280)
    cap.set(4, 720)

    while True:
        _, img = cap.read()
        img = cv2.flip(img, 1)

        cv2.imwrite('./NoMask/mask_' + str(ctime()) + '.png', img)  # Save the drawing
        cv2.imshow("image", img)

        # ESC to close
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            break

    cap.release()  # free the webcam for other uses if needed
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
