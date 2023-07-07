


import numpy as np
import cv2 as cv
import glob
# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((7*9,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:7].T.reshape(-1,2)*20
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
images = glob.glob('/home/rick/Immagini/*.jpg')
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

print("K1: \n", mtx0 )
print("D1: \n", dist0 )
print("R1: \n", rvecs0 )
print("T1: \n", tvecs0 )



img = cv.imread('/home/rick/Immagini/rgb2.jpg')
h, w = img.shape[:2]
newcameramtx1, roi1 = cv.getOptimalNewCameraMatrix(mtx0, dist0, (w,h), 1, (w,h))

# undistort
mapx, mapy = cv.initUndistortRectifyMap(mtx0, dist0, None, newcameramtx1, (w,h), 5)
dst = cv.remap(img, mapx, mapy, cv.INTER_LINEAR)
# crop the image
x, y, w, h = roi1
dst = dst[y:y+h, x:x+w]
cv.imwrite('calibresult0.png', dst)

mean_error = 0
for i in range(len(objpoints)):
 imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs0[i], tvecs0[i], mtx0, dist0)
 error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
 mean_error += error
print( "total error: {}".format(mean_error/len(objpoints)) )
print("\n")


criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((7*9,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:7].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
images = glob.glob('/home/rick/Immagini/infrared/*.jpg')
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

print("K2: \n", mtx1 )
print("D2: \n", dist1 )
print("R2: \n", rvecs1 )
print("T2: \n", tvecs1 )





img = cv.imread('/home/rick/Immagini/infrared/depth2.jpg')
h, w = img.shape[:2]
newcameramtx1, roi1 = cv.getOptimalNewCameraMatrix(mtx1, dist1, (w,h), 1, (w,h))

# undistort
mapx, mapy = cv.initUndistortRectifyMap(mtx1, dist1, None, newcameramtx1, (w,h), 5)
dst = cv.remap(img, mapx, mapy, cv.INTER_LINEAR)
# crop the image
x, y, w, h = roi1
dst = dst[y:y+h, x:x+w]
cv.imwrite('calibresult1.png', dst)


mean_error = 0
for i in range(len(objpoints)):
 imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs1[i], tvecs1[i], mtx1, dist1)
 error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
 mean_error += error
print( "total error: {}".format(mean_error/len(objpoints)) )


"""
Ccamera=[0.051,0.039,0.199]

R1, _ = cv.Rodrigues(rvecs1[1])
R0, _ = cv.Rodrigues(rvecs0[1])

# Create extrinsic matrix [R | tvec]
extrinsic_matrix1 = np.concatenate((R1, tvecs1[1].reshape(3,1)), axis=1)

extrinsic_matrix0 = np.concatenate((R0, tvecs0[1].reshape(3,1)), axis=1)

print("Extrinsic Matrix:")
print(extrinsic_matrix1)

inverse_extrinsic_matrix1 = np.linalg.pinv(extrinsic_matrix1)

print(inverse_extrinsic_matrix1)
print(Ccamera)

Cworld=np.matmul(inverse_extrinsic_matrix1,Ccamera)

Ccamera = np.matmul(extrinsic_matrix0, Cworld)

print(Ccamera)

"""