#!/usr/bin/python3

import json
import cv2
import numpy as np
from cv2 import FONT_ITALIC

"""
Script to segment the colors on a video from the pc webcam or image
"""

path_to_img = "green_car.jpg"
image_file = path_to_img + ".json"


def onTrackbar(threshold):
    pass


def main():
    # capture = cv2.VideoCapture(0)  # connect to webcam
    image = cv2.imread(path_to_img)
    image = cv2.resize(image, (850, 535))  # resize the capture window
    # image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    window_frame = "frame"

    cv2.namedWindow(window_frame)

    # create the trackbars
    cv2.createTrackbar("min B", window_frame, 0, 255, onTrackbar)
    cv2.createTrackbar("max B", window_frame, 255, 255, onTrackbar)
    cv2.createTrackbar("min G", window_frame, 0, 255, onTrackbar)
    cv2.createTrackbar("max G", window_frame, 255, 255, onTrackbar)
    cv2.createTrackbar("min R", window_frame, 0, 255, onTrackbar)
    cv2.createTrackbar("max R", window_frame, 255, 255, onTrackbar)

    # dictionary with ranges
    ranges_pcss = {"b": {"min": 100, "max": 256},
                   "g": {"min": 100, "max": 256},
                   "r": {"min": 100, "max": 256},
                   }

    # print
    print("Press w to save a new json file or q to quit")

    # cycle for continuously capture the image
    while True:

        # read the image from webcam
        # _, image = capture.read()
        # image = cv2.resize(image, (850, 535))  # resize the capture window
        # image = cv2.flip(image, 1)  # flip video capture

        # read the values on the trackbars
        min_b_pcss = cv2.getTrackbarPos("min B", window_frame)
        max_b_pcss = cv2.getTrackbarPos("max B", window_frame)
        min_g_pcss = cv2.getTrackbarPos("min G", window_frame)
        max_g_pcss = cv2.getTrackbarPos("max G", window_frame)
        min_r_pcss = cv2.getTrackbarPos("min R", window_frame)
        max_r_pcss = cv2.getTrackbarPos("max R", window_frame)

        # attribute the read values to the ranges dictionary
        ranges_pcss["b"]["min"] = min_b_pcss
        ranges_pcss["b"]["max"] = max_b_pcss
        ranges_pcss["g"]["min"] = min_g_pcss
        ranges_pcss["g"]["max"] = max_g_pcss
        ranges_pcss["r"]["min"] = min_r_pcss
        ranges_pcss["r"]["max"] = max_r_pcss

        # numpy arrays
        mins_pcss = np.array([ranges_pcss['b']['min'], ranges_pcss['g']['min'], ranges_pcss['r']['min']])
        maxs_pcss = np.array([ranges_pcss['b']['max'], ranges_pcss['g']['max'], ranges_pcss['r']['max']])

        # transform the image and show it
        mask = cv2.inRange(image, mins_pcss, maxs_pcss)  # colors mask
        image_segmenter = cv2.bitwise_and(image, image, mask=mask)

        # image text
        cv2.putText(image, "Video Capture", (50, 50), FONT_ITALIC, 1, (0, 0, 0), 2)
        cv2.putText(image_segmenter, "Mask", (50, 50), FONT_ITALIC, 1, (255, 255, 255), 2)

        # concatenate and show
        image_concatenate = cv2.hconcat((image, image_segmenter))
        cv2.imshow(window_frame, image_concatenate)

        """
        interactive keys (k) -----------------------------------------
        """
        # ESC to close
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            break

        # w to save the json file
        if k == ord("w"):
            # save json file
            file_name = image_file
            with open(file_name, 'w') as file_handle:
                print('Limits file saved in directory by the name ' + file_name)
                json.dump(ranges_pcss, file_handle)
            break

        # q to quit
        if k == ord("q"):
            break

    # capture.release()  # free the webcam for other use
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
