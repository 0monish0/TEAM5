import cv2
import numpy as np
import glob

# Chessboard dimensions (number of inner corners)
chessboard_size = (9, 6)

# Prepare object points: (0,0,0), (1,0,0), ..., (8,5,0)
objp = np.zeros((chessboard_size[0]*chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)

# Arrays to store object points and image points
objpoints = []  # 3D points in real world
imgpoints = []  # 2D points in image plane

# Load calibration images
images = glob.glob('calib_images/*.jpg')  # change extension if needed

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    if ret:
        objpoints.append(objp)
        # Refine corner locations
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1),
                                    criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        imgpoints.append(corners2)

        # Draw and display corners
        cv2.drawChessboardCorners(img, chessboard_size, corners2, ret)
        cv2.imshow('Corners', img)
        cv2.waitKey(100)

cv2.destroyAllWindows()

# Calibrate camera
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

print("Camera matrix (intrinsic parameters):")
print(mtx)
print("\nDistortion coefficients:")
print(dist)

# Save calibration results for later use
np.savez("camera_calib.npz", mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)
