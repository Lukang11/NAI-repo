import cv2 as cv
import time

def print_hi(name):
    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi, {name}')  # Press Ctrl+F8 to toggle the breakpoint.

if __name__ == '__main__':
    capture = cv.VideoCapture(1)
    while True:
        start = time.time()
        isTrue, frame = capture.read()
        flip = cv.flip(frame, 1)
        cv.imshow('Camera', flip)

        gray = cv.cvtColor(flip,cv.COLOR_BGR2GRAY)
        cv.imshow('Grey', gray)
        blur = cv.GaussianBlur(gray,(5,5), cv.BORDER_DEFAULT)
        canny = cv.Canny(gray , 125, 175)
        cv.imshow("Canny edges", canny)

        contours ,hierarchy = cv.findContours(canny,cv.RETR_LIST,cv.CHAIN_APPROX_SIMPLE)

        cv.drawContours(flip, contours, -1, (0, 0, 255), 1)
        cv.imshow("Counturing", flip)

        max_area = 0
        max_area_index = 0
        for i, c in enumerate(contours):
            area = cv.contourArea(c)
            if area > max_area:
                max_area = area
                max_area_index = i

        print("Largest contour area:", max_area)
        stop = time.time()
        image_contours = flip.copy()
        cv.drawContours(image_contours, contours, max_area_index, (0, 255, 0), 2)
        cv.imshow("Largest Contour", image_contours)
        print("Time to find and draw the biggest contour was :", stop - start)
        key=cv.waitKey(1)

        if key==ord("q"):
            break

capture.release()
cv.destroyAllWindows()
