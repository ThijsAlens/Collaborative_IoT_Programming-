
import cv2
import os
import random
from PIL import Image
import numpy as np

def process_image(image):
    
    # Convert image
    #cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')

    cv_image = image

    # Blur to reduce noise
    blurred = cv2.GaussianBlur(cv_image, (11, 11), 0)

    # Convert to HSV
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # Define color range for tennis ball (yellow-green)
    lower_yellow = np.array([28, 100, 100])
    upper_yellow = np.array([38, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # Morphological operations to clean up the mask
    mask = cv2.erode(mask, None, iterations=3)
    mask = cv2.dilate(mask, None, iterations=3)


    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    found_ball = False    

    masks_circles = np.zeros_like(mask)
    for contour in contours:

        area = cv2.contourArea(contour)

        # Filter out small or excessively large shapes
        if 500 < area < 50000:
            # Fit a circle around the contour
            (x, y), radius = cv2.minEnclosingCircle(contour)
            center = (int(x), int(y))
            radius = int(radius)
            circle_area = np.pi * (radius ** 2)

            # Calculate pixel density inside the circle
            mask_circle = np.zeros_like(mask)
            mask_visualisation = np.zeros_like(mask)
            cv2.circle(mask_circle, center, radius, 255, -1)
            cv2.circle(mask_visualisation, center, radius, 255, 3)
            points_inside = cv2.countNonZero(mask & mask_circle)
            density = points_inside / circle_area
            if(density > 0.6):
                masks_circles = masks_circles + mask_visualisation
                found_ball = True

    return found_ball


def main(args = None):

    dir_path = "data/"


    image1_path = os.path.join(dir_path, "image1.jpg")
    print(os.path.exists(image1_path))
    print(image1_path)
    image1 = cv2.imread(image1_path)
    image2 = cv2.imread(dir_path + "image2.png")
    image3 = cv2.imread(dir_path + "image3.png")


    detected_ball1 = process_image(image1)
    detected_ball2 = process_image(image2)    
    detected_ball3 = process_image(image3)
    cv2.waitKey()

    print(detected_ball1)
    print(detected_ball2)
    print(detected_ball3)

main()




