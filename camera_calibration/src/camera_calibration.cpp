#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <stdlib.h>
#include <math.h>
#include <fstream>
#include "camera_calibration.h"

cv::Vec3f rotationMatrixToEulerAngles(cv::Mat &R)
{
    double sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );

    bool singular = sy < 1e-6;

    double x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return cv::Vec3f(x, y, z);
}

cv::Mat eulerAnglesToRotationMatrix(cv::Vec3f &theta)
{
    // Calculate rotation about x axis
    cv::Mat R_x = (cv::Mat_<double>(3,3) <<
               1,       0,              0,
               0,       cos(theta[0]),   -sin(theta[0]),
               0,       sin(theta[0]),   cos(theta[0])
               );

    // Calculate rotation about y axis
    cv::Mat R_y = (cv::Mat_<double>(3,3) <<
               cos(theta[1]),    0,      sin(theta[1]),
               0,               1,      0,
               -sin(theta[1]),   0,      cos(theta[1])
               );

    // Calculate rotation about z axis
    cv::Mat R_z = (cv::Mat_<double>(3,3) <<
               cos(theta[2]),    -sin(theta[2]),      0,
               sin(theta[2]),    cos(theta[2]),       0,
               0,               0,                  1);

    // Combined rotation matrix
    cv::Mat R = R_z * R_y * R_x;

    return R;
}

cv::Mat RTtoMatrix(cv::Mat R, cv::Mat t){
	cv::Mat T = (cv::Mat_<double>(4,4) << R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0,0),
															R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1,0),
															R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2,0),	
															0, 0, 0, 1);

	return T;				
}

cv::Mat transformationToRotationMatrix(cv::Mat T){
	cv::Mat rvec = (cv::Mat_<double>(3,3) << T.at<double>(0,0), T.at<double>(0,1), T.at<double>(0,2),
									T.at<double>(1,0), T.at<double>(1,1), T.at<double>(1,2),
									T.at<double>(2,0), T.at<double>(2,1), T.at<double>(2,2));
	return rvec;
}

double computeReprojectionError(std::vector<std::vector<cv::Point3f>> object_points, 
								std::vector<std::vector<cv::Point2f>> image_points, 
								std::vector<cv::Mat> rvecs, std::vector<cv::Mat> tvecs,
								cv::Mat cameraMatrix, cv::Mat distCoeffs){

	std::vector<cv::Point2f> image_points2;
	double total_error = 0;
	double mean_error = 0;
	double total_points = 0;

	for (int i = 0; i < object_points.size(); i++){
		cv::projectPoints(object_points[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, image_points2);
		total_error += cv::norm(image_points[i], image_points2, cv::NORM_L2);	
		total_points += int(object_points.size());
	}

	mean_error = total_error/total_points;

	return mean_error;
}

void calibrateCamera(std::string img_folder){
	int iter = 0;
	cv::Mat img;
	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;

	cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::MAX_ITER +
                cv::TermCriteria::EPS,
                500, // max number of iterations
                0.0001); // min accuracy

	// Define chessboard
	double board_height = 6;
	double board_width = 5;
	double square_size = 0.01;
	std::vector<std::vector<cv::Point3f>> object_points;
	std::vector<std::vector<cv::Point2f>> image_points;
	cv::Size board_size = cv::Size(5, 6);

	while(iter < 19)
	{		
		img = cv::imread(img_folder + std::to_string(iter) + ".png", 0);

			if (!(img.empty()))
			{
				std::vector <cv::Mat> rtvecs;

				// Detect chessboard
				std::vector<cv::Point2f> corners;

				bool board_detected = cv::findChessboardCorners(img, board_size, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE
									+ cv::CALIB_CB_FAST_CHECK);

				if (board_detected){
					// Draw chessboard corners
					cv::cornerSubPix(img, corners, cv::Size(11, 11), cv::Size(-1, -1), criteria);
					cv::drawChessboardCorners(img, board_size, corners, board_detected);

					// Define object points
					std::vector<cv::Point3f> objp;
					for (int i = 0; i < board_height; i++)
					for (int j = 0; j < board_width; j++)
						objp.push_back(cv::Point3f((double)j * square_size, (double)i * square_size, 0));

					image_points.push_back(corners);
					object_points.push_back(objp);				
				}		
			}
		iter++;
	}

	// Calibrate camera
	std::vector <cv::Mat> rvecs, tvecs;
	cv::calibrateCamera(object_points, image_points, img.size(), cameraMatrix, distCoeffs, rvecs, tvecs, cv::CALIB_FIX_K4);

	std::cout << "cameraMatrix: " << cameraMatrix << std::endl;
	std::cout << "distCoeffs: " << distCoeffs << std::endl;

	// Reprojection error
	double error = computeReprojectionError(object_points, image_points, rvecs, tvecs, cameraMatrix, distCoeffs);
	std::cout << "Reprojection error: " << error << std::endl;

}


std::vector<cv::Mat> convertCSVToMat(std::string filename){
	std::string line;
	double val;
	std::vector<double> rtvec;

    // Create an input filestream
    std::ifstream myFile(filename);

    // Make sure the file is open
    if(!myFile.is_open()) throw std::runtime_error("Could not open file");

	// Read data, line by line
    while(std::getline(myFile, line))
    {
        // Create a stringstream of the current line
        std::stringstream ss(line);
		val = std::stof(line);

		rtvec.push_back(val);
    }

	// Create rotation matrix and translation vectors
	std::vector<cv::Mat> rtvecs;
	cv::Mat tvec = (cv::Mat_<double>(3,1) << rtvec[0], rtvec[1], rtvec[2]);
	cv::Mat rvec = (cv::Mat_<double>(3,1) << rtvec[3], rtvec[4], rtvec[5]);
	cv::Rodrigues(rvec, rvec);

	// Rotation and translation from tcp to board
	cv::Mat tvec_tcp2board = (cv::Mat_<double>(3,1) << 29.0e-3, -58.0e-3, 35.0e-3);
	//cv::Vec3f theta(-M_PI/2, M_PI, 0.0);
	cv::Vec3f theta(0.0, -M_PI, 0.0);
	
	
	cv::Mat rvec_tcp2board = eulerAnglesToRotationMatrix(theta);

	// Get transformation from board to base
	cv::Mat T_base2tcp = RTtoMatrix(rvec, tvec);
	cv::Mat T_tcp2board = RTtoMatrix(rvec_tcp2board, tvec_tcp2board);

	cv::Mat T_base2board = T_base2tcp*T_tcp2board;
	cv::Mat T_board2base = T_base2board.inv();

	//cv::Mat T_board2base = T_base2tcp.inv();

	// Convert to rotation matrix and translation vector
	rvec = (cv::Mat_<double>(3,3) << T_board2base.at<double>(0,0), T_board2base.at<double>(0,1), T_board2base.at<double>(0,2),
									T_board2base.at<double>(1,0), T_board2base.at<double>(1,1), T_board2base.at<double>(1,2),
									T_board2base.at<double>(2,0), T_board2base.at<double>(2,1), T_board2base.at<double>(2,2));
	
	tvec = (cv::Mat_<double>(3,1) << T_board2base.at<double>(0,3), T_board2base.at<double>(1,3), T_board2base.at<double>(2,3));
	
	rtvecs.push_back(rvec);
	rtvecs.push_back(tvec);


	std::cout << "tvec: " << tvec << std::endl;

	return rtvecs;
}

std::vector <cv::Mat> chessboardPose(cv::Mat img_rgb){
	std::vector <cv::Mat> rtvecs;

	if (img_rgb.empty()){
		std::cout << "input image is empty" << std::endl;
		return rtvecs;
	}

	cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << 551.6570310249326, 0, 319.8393905438384,
 													0, 551.6960583443193, 240.7978933954921,
													0, 0, 1);
	cv::Mat distCoeffs = (cv::Mat_<double>(1,5) << -0.02728732995117321, 0.8338034682943555, 0.00258786736198274, -0.00151950357701773, -2.771420832332028);
	cv::Mat rvec;
	cv::Mat tvec;

	// Termination criteria
	cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::MAX_ITER +
                cv::TermCriteria::EPS,
                500, // max number of iterations
                0.0001); // min accuracy


	// Detect chessboard
	cv::Size board_size = cv::Size(5, 6);
	std::vector<cv::Point2f> corners;

	bool board_detected = cv::findChessboardCorners(img_rgb, board_size, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE
                        + cv::CALIB_CB_FAST_CHECK);

	if (!board_detected){
		std::cout << "no chessboard found" << std::endl;
		return rtvecs;
	}

	// Draw chessboard corners
	cv::cornerSubPix(img_rgb, corners, cv::Size(11, 11), cv::Size(-1, -1), criteria);
	cv::drawChessboardCorners(img_rgb, board_size, corners, board_detected);

	// Define chessboard
	double board_height = 6;
	double board_width = 5;
	double square_size = 1e-2;
	std::vector<std::vector<cv::Point3f>> object_points;
	std::vector<std::vector<cv::Point2f>> image_points;

	// Define object points
	std::vector<cv::Point3f> objp;
    for (int i = 0; i < board_height; i++)
      for (int j = 0; j < board_width; j++)
        objp.push_back(cv::Point3f((double)j * square_size, (double)i * square_size, 0));

	cv::Mat rvecs, tvecs;
	cv::solvePnP(objp, corners, cameraMatrix, distCoeffs, rvecs, tvecs);
	

	rtvecs.push_back(rvecs);
	rtvecs.push_back(tvecs);

	//cv::imshow("img", img_rgb);
	//cv::waitKey(0);

	std::cout << "tvec: " << tvec << std::endl;
	std::cout << "objp: " << objp << std::endl;

	return rtvecs;
}

cv::Mat handToEyeCalibration(std::string img_folder, std::string rtvec_folder){
	int nTransforms = 0;
	std::vector<cv::Mat> rvecs_cam2board;
	std::vector<cv::Mat> tvecs_cam2board;
	std::vector<cv::Mat> rvecs_base2board;
	std::vector<cv::Mat> tvecs_base2board;

	std::vector<cv::Mat> rtvecs_cam2board;
	std::vector<cv::Mat> rtvecs_base2board;

	cv::Mat img;
	
	while(nTransforms < 25)
	{		
		img = cv::imread(img_folder + std::to_string(nTransforms) + ".png", 0);

			if (!(img.empty()))
			{
				rtvecs_cam2board = chessboardPose(img);
				
				if (!(rtvecs_cam2board.empty()))
				{
					rtvecs_base2board = convertCSVToMat(rtvec_folder + std::to_string(nTransforms) + ".csv");			

					rvecs_cam2board.push_back(rtvecs_cam2board[0]);
					tvecs_cam2board.push_back(rtvecs_cam2board[1]);
					rvecs_base2board.push_back(rtvecs_base2board[0]);
					tvecs_base2board.push_back(rtvecs_base2board[1]);
				}
			}
		nTransforms++;
	}
	
	cv::Mat R_base2cam;
	cv::Mat t_base2cam;

	std::cout << "tvecs_base2board: " << tvecs_base2board[0] << std::endl;
	std::cout << "rvecs_base2board: " << rvecs_base2board[0] << std::endl;
	std::cout << "tvecs_cam2board: " << tvecs_cam2board[0] << std::endl;
	std::cout << "rvecs_cam2board: " << rvecs_cam2board[0] << std::endl;

	cv::calibrateHandEye(rvecs_base2board, tvecs_base2board, rvecs_cam2board, tvecs_cam2board, R_base2cam, t_base2cam, cv::CALIB_HAND_EYE_PARK); //Park

	
	cv::Mat T_base2cam = (cv::Mat_<double>(4,4) << R_base2cam.at<double>(0,0), R_base2cam.at<double>(0,1), R_base2cam.at<double>(0,2), t_base2cam.at<double>(0,0),
										R_base2cam.at<double>(1,0), R_base2cam.at<double>(1,1), R_base2cam.at<double>(1,2), t_base2cam.at<double>(1,0),
										R_base2cam.at<double>(2,0), R_base2cam.at<double>(2,1), R_base2cam.at<double>(2,2), t_base2cam.at<double>(2,0),	
										0, 0, 0, 1);	
	
	/*cv::Mat T_base2cam = (cv::Mat_<double>(4,4) << 1, 0, 0, t_base2cam.at<double>(0,0),
										0, 1, 0, t_base2cam.at<double>(1,0),
										0, 0, 1, t_base2cam.at<double>(2,0),	
										0, 0, 0, 1);	
	*/
	
	cv::Mat T_cam2base = T_base2cam.inv();
	
	std::cout << "T_base2cam: " << std::endl << T_base2cam << std::endl;
	std::cout << "T_cam2base: " << std::endl << T_cam2base << std::endl;

	
	
	return T_cam2base;
}

int main(int argc, char **argv)
{
	
	cv::Mat T_cam2base = handToEyeCalibration("../data/pose_", "../data/pose_");



	std::cout << "T_base2target" << std::endl;
	cv::Mat T_cam2target = (cv::Mat_<double>(4,4) << 1, 0, 0, -0.01415853668, 
													0, 1, 0, 0.0173048768193, 
													0, 0, 1, 0.601999998093, 
													0, 0, 0, 1);

	


	// Z + 166.5
	cv::Mat T_base2target = T_cam2target.inv() * T_cam2base;

	std::cout << T_base2target.inv() << std::endl;

	//572, -257

	//calculateCalibrationError("../data/pose_", "../data/pose_", T_cam2base);



	//calibrateCamera("../data/cali_");

	return 0; 
}
