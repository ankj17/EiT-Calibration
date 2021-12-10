import cv2
import os
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
import scipy

class Calibration():
    def __init__(self):
        self.cameraMatrix = np.empty((3,3))
        self.distCoeffs = np.empty((1,5))
        self.boardWidth = 5
        self.boardHeight = 6
        self.squareSize = 1e-2
        self.patternSize = (self.boardWidth, self.boardHeight)
        self.t_tcp2board = np.array([29.0e-3, -58.0e-3, 35.0e-3])
        self.criteria = (cv2.TermCriteria_MAX_ITER +
                cv2.TermCriteria_EPS,
                500, # max number of iterations
                0.0001) # min accuracy)
        
    def calibrateCamera(self, images):
        imagePoints = []
        objectPoints = []
        for img in images:
            img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(img_gray, self.patternSize, flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE+ cv2.CALIB_CB_FAST_CHECK)

            # Define object points
            objp = []
            for i in range(self.boardHeight):
                for j in range(self.boardWidth):
                    objp.append([float(j)*self.squareSize, float(i)*self.squareSize, float(0.0)])
            objp = np.array(objp)
            objp = objp.astype('float32')
           
            # Determine pose of the chessboard
            if ret == True:
                corners = cv2.cornerSubPix(img_gray, corners, (11,11), (-1,-1), self.criteria)
            
                # Draw detected corners
                cv2.drawChessboardCorners(img_gray, self.patternSize, corners, ret)
                imagePoints.append(corners)
                objectPoints.append(objp)

        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objectPoints, imagePoints, img_gray.shape[::-1], None, None, None, None, flags=cv2.CALIB_FIX_K4)
        
        np.copyto(self.distCoeffs, dist)
        np.copyto(self.cameraMatrix, mtx)       

    def chessboardPose(self, img):     
        img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(img_gray, self.patternSize, flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE+ cv2.CALIB_CB_FAST_CHECK)

        #dictionary = cv2.aruco.DICT_4x4_50
        #parameters = cv2.aruco.DetectorParameters.create()
        #corners, ids, rejected = cv2.aruco.detectMarkers(img, dictionary, parameters)

        # Define object points
        objp = []
        for i in range(self.boardHeight):
            for j in range(self.boardWidth):
                objp.append([float(j)*self.squareSize, float(i)*self.squareSize, float(0.0)])
        objp = np.array(objp)
        objp = objp.astype('float32')

        # Determine pose of the chessboard
        if ret == True:
            corners = cv2.cornerSubPix(img_gray, corners, (11,11), (-1,-1), self.criteria)
            retval, rvec, tvec = cv2.solvePnP(objp, corners, self.cameraMatrix, self.distCoeffs)

            # Draw detected corners
            cv2.drawChessboardCorners(img_gray, self.patternSize, corners, ret)

        # Show detected corners
        #cv2.imshow("img", img_gray)
        #cv2.waitKey(0)

        return rvec, tvec

    def tcp2board(self, vec):
        # Transformation from base to tcp
        r_base2tcp = np.array(vec[3:])
        t_base2tcp = np.array(vec[:3])
        R_base2tcp, _ = cv2.Rodrigues(r_base2tcp)

        # Transformation from tcp to board
        R_tcp2board = R.from_euler('xyz', [0.0, -math.pi, 0.0], degrees=False)
        R_tcp2board = np.array(R_tcp2board.as_matrix())

        # Create transformation matrices
        T_base2tcp = np.eye(4)
        T_base2tcp[:3, :3] = R_base2tcp
        T_base2tcp[:3, 3] = t_base2tcp
        T_tcp2board = np.eye(4)
        T_tcp2board[:3, :3] = R_tcp2board
        T_tcp2board[:3, 3] = self.t_tcp2board

        # Determine transformation from board to base
        T_base2board = np.matmul(T_base2tcp, T_tcp2board)
        T_board2base = np.linalg.inv(np.matrix(T_base2board))

        rvec = T_board2base[:3, :3]
        tvec = T_board2base[:3, 3]

        return rvec, tvec

    def handToEyeCalibration(self, images, rtvecs):
        rvecs_cam2board = []
        tvecs_cam2board = []
        rvecs_base2board = []
        tvecs_base2board = []

        rtvecs_cam2board = []
        rtvecs_base2board = []

        for img in images:
            rvec, tvec = self.chessboardPose(img)     
            rvecs_cam2board.append(rvec)
            tvecs_cam2board.append(tvec)

        for vec in rtvecs:
            rvec, tvec = self.tcp2board(vec)
            rvecs_base2board.append(rvec)
            tvecs_base2board.append(tvec)
        
        R_base2cam, t_base2cam = cv2.calibrateHandEye(rvecs_base2board, tvecs_base2board, rvecs_cam2board, tvecs_cam2board, cv2.CALIB_HAND_EYE_PARK)

        T_base2cam = np.eye(4)
        T_base2cam[:3, :3] = R_base2cam
        T_base2cam[:3, 3] = t_base2cam[:,-1]

        T_cam2base = np.linalg.inv(T_base2cam)

        print("T_base2cam: ", T_base2cam)

        return T_cam2base

if __name__ == "__main__":
    cal = Calibration()

    ## test
    dir = "/home/andreas/Desktop/EiT-Calibration/camera_calibration/data"

    images = []
    rtvecs = []

    for root, dirs, files in os.walk(dir):
        for name in sorted(files):
            #print ("Current File Being Processed is: " + name)
            if name.endswith(".png"):
                img_path = os.path.join(root, name)
                img = cv2.imread(img_path)
                images.append(img)
            elif name.endswith(".csv"):
                csv_path = os.path.join(root, name)
                data = np.genfromtxt(csv_path, delimiter=',')
                #print("data: ", data)
                rtvecs.append(data)
                #cal.tcp2board(data)

    cal.calibrateCamera(images)

    T_cam2base = cal.handToEyeCalibration(images, rtvecs)

    print("T_cam2base: ", T_cam2base)   