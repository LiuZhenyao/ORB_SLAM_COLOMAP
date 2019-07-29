import os
import cv2
import numpy as np

img_path = "/home/shu/Downloads/JD/2019_06_26_ExtractedTUKLData_Log123/Sensor_6/orb_slot_1/15347732073177.png"

img = cv2.imread(img_path)

# ix,iy = -1,-1
ix = []
iy = []

# mouse callback function
def draw_circle(event,x,y,flags,param):
    global ix,iy
    if event == cv2.EVENT_LBUTTONDBLCLK:
        cv2.circle(img,(x,y),2,(255,0,0),-1)
        ix.append(x)
        iy.append(y)

# Create a black image, a window and bind the function to window
# img = np.zeros((512,512,3), np.uint8)
cv2.namedWindow('image')
cv2.setMouseCallback('image',draw_circle)

while(1):
    if len(ix) >= 2:
        break
    cv2.imshow('image',img)
    k = cv2.waitKey(20) & 0xFF
    if k == 27:
        break
    elif k == ord('a'):
        print ix,iy
print ix, iy

cv2.destroyAllWindows()


a = (np.float(iy[0] - iy[1])) / (ix[0] - ix[1])
b = iy[0] - a * ix[0]

for y in range(img.shape[0]):
    for x in range(img.shape[1]):
        if y >= a*x+b:
            img[y, x] = 0
        elif y < a*x+b:
            img[y, x] = 255

# cv2.namedWindow('image')
# cv2.imshow('image',img)
# cv2.waitKey(0)

cv2.imwrite("/home/shu/Downloads/JD/2019_06_26_ExtractedTUKLData_Log123/Sensor_6/mask_sensor6_slot_1.png", img)


path = '/home/shu/Downloads/JD/2019_06_26_ExtractedTUKLData_Log123/Sensor_6/orb_slot_1'
path2 = '/home/shu/Downloads/JD/2019_06_26_ExtractedTUKLData_Log123/Sensor_6/orb_slot_1_masked'
img_list = os.listdir(path)

mask = cv2.imread('/home/shu/Downloads/JD/2019_06_26_ExtractedTUKLData_Log123/Sensor_6/mask_sensor6_slot_1.png')

for name in img_list:
    img = cv2.imread(os.path.join(path, name))
    img[mask == 0] = 0

    cv2.imwrite(os.path.join(path2, name), img)


# --------------------------------------------------------------#

# import os
# import cv2
# import numpy as np
#
# img_path = "/home/shu/Downloads/JD/2019_06_26_ExtractedTUKLData_Log123/Sensor_6/orb_slot_0/15347731875845.png"
#
# img = cv2.imread(img_path)
# pts1 = np.float32([[314, 239], [919, 115], [1263, 310], [882, 612]])
# pts2 = np.float32([[0,0], [0, 1280], [800, 0], [800, 1280]])
# matrix = cv2.getPerspectiveTransform(pts1, pts2)
#
# result = cv2.warpPerspective(img, matrix, (800, 1280))
#
# cv2.namedWindow('image')
# cv2.imshow("image",result)
# cv2.waitKey(0)



