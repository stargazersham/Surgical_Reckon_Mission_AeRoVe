import numpy as np
import cv2

def filtered_rect(rect_list):
    new_rect_list = []
    for rect in rect_list:
        if rect[2]<120 or rect[3] < 120:
            pass
        else:
            new_rect_list.append(rect)
    new_rect_list = np.array(new_rect_list)
    return new_rect_list

def list_rectangle(contors):
    rectangle_list = []
    for cnt in contors:
        x,y,w,h = cv2.boundingRect(cv2.UMat(cnt))
        rectangle_list.append([x,y,w,h])
    rectangle_list = np.array(rectangle_list)
    rectangle_list = filtered_rect(rectangle_list)
    return rectangle_list

def final_rect_for_test(crop_image):
    rect_1 = cv2.resize(crop_image, (28,28),interpolation = cv2.INTER_AREA)
    rect_1 = rect_1/255
    rect_1 = np.reshape(rect_1,(1,28,28,1))
    return rect_1

def cnvt_square(x1,x2,y1,y2,img):
    #x1 -= 5
    #y1 -= 5
    #x2 += 5
    #y2 += 5
    l = x2 - x1
    b = y2 - y1
    if (l > b):
        c = (y1 + y2)/2
        y1 = c - l/2
        y2 = c + l/2
    else:
        c = (x1 + x2)/2
        x1 = c -b/2
        x2 = c + b/2
    if(x1 < 0):
        x1 = 0
    if(y1 < 0):
        y1 = 0
    if(x2 > img.shape[1]):
        x2 = img.shape[1]
    if(y2 > img.shape[0]):
        y2 = img.shape[0]
    return int(x1), int(x2), int(y1), int(y2)

def search_rectangle(img):
    x1 = 0
    x1_found = False
    y1_found = False
    y1 = 0
    rx = img.shape[1]
    ry = img.shape[0]
    x2 = 0
    y2 = 0
    for i in range (0, rx):
        if(img[:,i].any() > 0):
            if (not x1_found):
                x1 = i
                x1_found = True
            else:
                if(x2 < i):
                    x2 = i
    for i in range (0, ry):
        if(img[i,:].any() > 0):
            if (not y1_found):
                y1 = i
                y1_found = True
            else:
                if(y2 < i):
                    y2 = i
    return cnvt_square(x1, x2, y1, y2, img)

def square_center(x1,x2,y1,y2):
    return [(x1+x2)/2 , (y1+y2)/2]

def frame_location(result, target):
    final_result = []
    temp_result = []
    for i in result:
        temp_result.append([i[0][0], i[1]])
    temp_x = 0
    for i in range(0,3):
        if (temp_result[temp_x][0] > temp_result[i][0]):
            temp_x = i
    final_result.append(temp_result[temp_x])
    for i in range(0,3):
        if (i != temp_x):
            for j in range (0,3):
                if(j != temp_x and j != i):
                    if(temp_result[i][0] < temp_result[j][0]):
                        final_result.append(temp_result[i])
                        final_result.append(temp_result[j])
    return position_finder(final_result, target)

def position_finder(result, target):
    for i in range(0,3):
        if result[i][1] == target:
            return i - 1
    return 'Not found'

def main_prediction_fxn(image, model, maping, target):
    final_result = []
    img_gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    ret,thresh1 = cv2.threshold(img_gray,30,255,cv2.THRESH_BINARY)
    img_not = cv2.bitwise_not(thresh1)
    _, contour,hierarchy = cv2.findContours(img_not, 1, 2)

    rect_list = list_rectangle(contour)
    for i in range (0, rect_list.shape[0]):
        rect1 = rect_list[i]
        x,y,w,h = rect1[0],rect1[1],rect1[2],rect1[3]
        center = square_center(x, x + w, y, y + h)
        crop_image = thresh1[y+20:y+h-20,x+20:x+w-20].copy()

        x1, x2, y1, y2 = search_rectangle(crop_image)
        final_crop = crop_image[y1:y2, x1:x2]
        rect_1 = final_rect_for_test(final_crop)

        y = model.predict(rect_1)
        result = np.array(np.where(y >= 0.5))
        z = chr(maping[result[1,0],1])
        final_result.append([center,z])
    return frame_location(final_result, target)
