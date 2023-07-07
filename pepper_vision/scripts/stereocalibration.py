import numpy as np
import cv2 as cv
import glob



# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((7*9,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:7].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
images = glob.glob('/home/rick/Immagini/front*.jpg')
for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (9,7), None)
 # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)
    # Draw and display the corners
        cv.drawChessboardCorners(img, (9,7), corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(500)
        cv.destroyAllWindows()



ret0, mtx0, dist0, rvecs0, tvecs0 = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)


criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((7*9,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:7].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
images = glob.glob('/home/rick/Immagini/depth*.jpg')
for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (9,7), None)
 # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)
    # Draw and display the corners
        cv.drawChessboardCorners(img, (9,7), corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(500)
        cv.destroyAllWindows()

ret1, mtx1, dist1, rvecs1, tvecs1 = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)








# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((7*9,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:7].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
rgbpoints = [] 
depthpoints = []# 2d points in image plane.
images = glob.glob('/home/rick/Immagini/front*')

for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    depth_name = "/home/rick/Immagini/depth" + fname.removeprefix("/home/rick/Immagini/front") 
    img2= cv.imread(depth_name)
    gray2 = cv.cvtColor(img2, cv.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (9,7), None)
    ret1, corners1 = cv.findChessboardCorners(gray2, (9,7), None)
 # If found, add object points, image points (after refining them)
    if ret == True and ret1 == True:
        
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        corners3 = cv.cornerSubPix(gray2,corners1, (11,11), (-1,-1), criteria)

        rgbpoints.append(corners2)
        depthpoints.append(corners3)
    # Draw and display the corners
        cv.drawChessboardCorners(img, (9,7), corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(500)
        cv.destroyAllWindows()

        cv.drawChessboardCorners(img2, (9,7), corners3, ret)
        cv.imshow('img1', img2)
        cv.waitKey(500)
        cv.destroyAllWindows()






ret2, K1, D1, K2, D2, R, T, E, F = cv.stereoCalibrate(objpoints, rgbpoints, depthpoints, mtx0, dist0, mtx1, dist1, imageSize=(320,240), flags=cv.CALIB_FIX_INTRINSIC | cv.CALIB_USE_INTRINSIC_GUESS)

print("K1 :\n", K1)

print("D1 :\n", D1)

print("K2 :\n", K2)

print("D2 :\n", D2)

print("R :\n" , R)

print("T : \n", T)

print("E : \n", E)

print("F : \n", F)



