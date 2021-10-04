import numpy as np
import cv2
import math


def frame_distance(img_path):

    cam_x = 1000
    cam_y = 800
    k = 0.8
    apt = 700

    find = np.array([0.0, 0.0, 0.0])

    i = 0
    err = 0
    alpha = 50
    beta = 0

    loc = 0.0

    img2 = cv2.imread(img_path,0)
    
    img2 = cv2.addWeighted(img2, alpha, np.zeros(img2.shape, img2.dtype), 0, beta)

    _, th1 = cv2.threshold(img2, 10 , 255, cv2.THRESH_BINARY)
    cv2.line(th1, (-100,10), (cam_x+100,10), (0, 0, 0), apt, cv2.LINE_AA)
    cv2.line(th1, (-100,cam_y-10), (cam_x+100,cam_y-10), (0, 0, 0), apt, cv2.LINE_AA)

    _, contours,_ = cv2.findContours(th1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    for contour in contours:
        approx = cv2.approxPolyDP(contour, 0.0105* cv2.arcLength(contour, True), True)#0.0105
        cv2.drawContours(th1, [approx], 0, (127, 0, 0), 8)
        x = approx.ravel()[0]
        y = approx.ravel()[1] 
        
        if len(approx) == 4:
            x1 ,y1, w, h = cv2.boundingRect(approx)
            pos = int(x + w/2) - int(cam_x/2)
            area = w * h
            
            if (1000 < area and abs(pos) < 0.85 * cam_x/2.0):
                find[i] = pos
                i = i + 1
            
            else:
                pass
        
        else:
            pass

    if (find[0] != 0 and find[1] == 0 and find[2] == 0):
        if (find[0] < 0):
            loc = 2.0 * k * float(find[0]) / float(cam_x) - 0.5

        elif (find[0] > 0):
            loc = 2.0 * k * float(find[0]) / float(cam_x) + 0.5

        elif (find[0] == 0):
            loc = pos
        
        else: 
            pass


    if (find[0] != 0 and find[1] != 0 and find[2] == 0):
        loc = 2.0 * k * float((find[0]+find[1])/2.0) / float(cam_x) 

    elif (find[0] == 0 or find[2] != 0):
        err = 1
    
    else:
        pass

    if (err == 0):
        return loc
        
    elif (err == 1):
        return 1000 

    else:
        pass
