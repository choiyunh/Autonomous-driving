import cv2
import numpy as np

def rgbscale(img):
    return cv2.cvtColor(img, cv2.COLOR_RGBA2RGB)

def region_of_interest(img, vertices, color3=(255, 255, 255), color1=255):
    mask = np.zeros_like(img)
    if len(img.shape) > 2:
        color = color3
    else:
        color = color1
    cv2.fillPoly(mask, vertices, color)
    ROI_image = cv2.bitwise_and(img, mask)
    return ROI_image


def detect_stopline(x):
    frame = x.copy()
    img = frame.copy()
    min_stopline_length = 250
    max_distance = 70

    # blur
    kernel_size = 5
    blur_frame = cv2.GaussianBlur(frame, (kernel_size, kernel_size), 0)

    # roi
    vertices = np.array([[
        (80, frame.shape[0]),
        (120, frame.shape[0] - 70),
        (frame.shape[1] - 120, frame.shape[0] - 70),
        (frame.shape[1] - 80, frame.shape[0])
    ]], dtype=np.int32)

    roi = region_of_interest(blur_frame, vertices)
    #     roi = blur_frame  # for test

    # yellow RGB
    lower_yellow, upper_yellow = (195, 195, 70), (255, 255, 180)
    img_mask = cv2.inRange(roi, lower_yellow, upper_yellow)
    img_result = cv2.bitwise_and(roi, roi, mask=img_mask)

    # gray
    gray = cv2.cvtColor(img_result, cv2.COLOR_BGR2GRAY)

    # binary
    ret, dest = cv2.threshold(gray, 160, 255, cv2.THRESH_BINARY)

    # canny
    low_threshold, high_threshold = 70, 210
    edge_img = cv2.Canny(np.uint8(dest), low_threshold, high_threshold)

    # find contours, opencv4
    # contours, hierarchy = cv2.findContours(edge_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # find contours, opencv3
    _, contours, hierarchy = cv2.findContours(edge_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        stopline_info = [0, 0, 0, 0]
        for contour in contours:
            epsilon = 0.01 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            # result = cv2.drawContours(frame, [approx], 0, (0,255,0), 4)
            x, y, w, h = cv2.boundingRect(contour)
            if stopline_info[2] < w:
                stopline_info = [x, y, w, h]
            # cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 3)
            rect = cv2.minAreaRect(approx)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            result = cv2.drawContours(frame, [box], 0, (0, 255, 0), 3)

        cx, cy = stopline_info[0] + 0.5 * stopline_info[2], stopline_info[1] + 0.5 * stopline_info[3]
        center = np.array([cx, cy])
        stopline_length = stopline_info[2]
        bot_point = np.array([frame.shape[1] // 2, frame.shape[0]])
        distance = np.sqrt(np.sum(np.square(center - bot_point)))

        # OUTPUT
        print('length : {},  distance : {}'.format(stopline_length, distance))

        if stopline_length > min_stopline_length and distance < max_distance:
            cv2.imshow('stopline', result)
            print('STOPLINE Detected')
            return True

    cv2.imshow('stopline', img)
    #print('No STOPLINE.')
    return False


def detect_stoplineB(x):
    frame = x.copy()
    img = frame.copy()
    min_stopline_length = 250
    max_distance = 70

    # gray
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # blur
    kernel_size = 5
    blur_frame = cv2.GaussianBlur(gray, (kernel_size, kernel_size), 0)

    # roi
    vertices = np.array([[
        (80, frame.shape[0]),
        (140, frame.shape[0] - 70),
        (frame.shape[1] - 140, frame.shape[0] - 70),
        (frame.shape[1] - 80, frame.shape[0])
    ]], dtype=np.int32)

    roi = region_of_interest(blur_frame, vertices)

    # filter
    img_mask = cv2.inRange(roi, 160, 220)
    img_result = cv2.bitwise_and(roi, roi, mask=img_mask)

    # cv2.imshow('bin', img_result)

    # binary
    ret, dest = cv2.threshold(img_result, 160, 255, cv2.THRESH_BINARY)

    # canny
    low_threshold, high_threshold = 70, 210
    edge_img = cv2.Canny(np.uint8(dest), low_threshold, high_threshold)

    # find contours, opencv4
    # contours, hierarchy = cv2.findContours(edge_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # find contours, opencv3
    _, contours, hierarchy = cv2.findContours(edge_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        stopline_info = [0, 0, 0, 0]
        for contour in contours:
            epsilon = 0.01 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            # result = cv2.drawContours(frame, [approx], 0, (0,255,0), 4)
            x, y, w, h = cv2.boundingRect(contour)
            if stopline_info[2] < w:
                stopline_info = [x, y, w, h]
            # cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 3)
            rect = cv2.minAreaRect(approx)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            result = cv2.drawContours(frame, [box], 0, (0, 255, 0), 3)

        cx, cy = stopline_info[0] + 0.5 * stopline_info[2], stopline_info[1] + 0.5 * stopline_info[3]
        center = np.array([cx, cy])
        stopline_length = stopline_info[2]
        bot_point = np.array([frame.shape[1] // 2, frame.shape[0]])
        distance = np.sqrt(np.sum(np.square(center - bot_point)))

        # OUTPUT
        print('length : {},  distance : {}'.format(stopline_length, distance))

        if stopline_length > min_stopline_length and distance < max_distance:
            cv2.imshow('stopline', result)
            print('STOPLINE Detected')
            return True

    cv2.imshow('stopline', img)
    # print('No STOPLINE.')
    return False
