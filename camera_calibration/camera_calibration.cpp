#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include "camera_calibration.h"

// Converts a given Euler angles to Rotation Matrix
cv::Mat euler2rot(const cv::Mat & euler)
{
  cv::Mat rotationMatrix(3,3,CV_32F);

  float x = euler.at<float>(0);
  float y = euler.at<float>(1);
  float z = euler.at<float>(2);

  // Assuming the angles are in radians.
  float ch = cos(z);
  float sh = sin(z);
  float ca = cos(y);
  float sa = sin(y);
  float cb = cos(x);
  float sb = sin(x);

  float m00, m01, m02, m10, m11, m12, m20, m21, m22;

  m00 = ch * ca;
  m01 = sh*sb - ch*sa*cb;
  m02 = ch*sa*sb + sh*cb;
  m10 = sa;
  m11 = ca*cb;
  m12 = -ca*sb;
  m20 = -sh*ca;
  m21 = sh*sa*cb + ch*sb;
  m22 = -sh*sa*sb + ch*cb;

  rotationMatrix.at<float>(0,0) = m00;
  rotationMatrix.at<float>(0,1) = m01;
  rotationMatrix.at<float>(0,2) = m02;
  rotationMatrix.at<float>(1,0) = m10;
  rotationMatrix.at<float>(1,1) = m11;
  rotationMatrix.at<float>(1,2) = m12;
  rotationMatrix.at<float>(2,0) = m20;
  rotationMatrix.at<float>(2,1) = m21;
  rotationMatrix.at<float>(2,2) = m22;

  return rotationMatrix;
}


std::vector<cv::Mat> convertCSVToMat(std::string filename){
	std::string line;
	float val;
	std::vector<float> rtvec;

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

	// Create rotation and translation vectors
	std::vector<cv::Mat> rtvecs;
	cv::Mat rvec = (cv::Mat_<float>(3,1) << rtvec[0], rtvec[1], rtvec[2]);
	cv::Mat tvec = (cv::Mat_<float>(3,1) << rtvec[3], rtvec[4], rtvec[5]);
	cv::Rodrigues(rvec, rvec);


	// Rotation and translation from tcp to board
	cv::Mat tvec_tcp2board = (cv::Mat_<float>(3,1) << -29.0, -58.0, 35.5);
	cv::Mat rvec_tcp2board = (cv::Mat_<float>(3,1) << 0.0, M_PI/2, 0.0);

	rvec_tcp2board = euler2rot(rvec_tcp2board);

	cv::Mat T1 = cv::Mat_<float>(4,4);
	T1.at<float>(0,0) = rvec.at<float>(0,0);
	T1.at<float>(0,1) = rvec.at<float>(0,1);
	T1.at<float>(0,2) = rvec.at<float>(0,2);
	T1.at<float>(1,0) = rvec.at<float>(1,0);
	T1.at<float>(1,1) = rvec.at<float>(1,1);
	T1.at<float>(1,2) = rvec.at<float>(1,2);
	T1.at<float>(2,0) = rvec.at<float>(2,0);
	T1.at<float>(2,1) = rvec.at<float>(2,1);
	T1.at<float>(2,2) = rvec.at<float>(2,2);
	T1.at<float>(0,3) = tvec.at<float>(0,0);
	T1.at<float>(1,3) = tvec.at<float>(0,1);
	T1.at<float>(2,3) = tvec.at<float>(0,2);
	T1.at<float>(3,0) = 0;
	T1.at<float>(3,1) = 0;
	T1.at<float>(3,2) = 0;
	T1.at<float>(3,3) = 1;

	cv::Mat T2 = cv::Mat_<float>(4,4);
	T2.at<float>(0,0) = rvec_tcp2board.at<float>(0,0);
	T2.at<float>(0,1) = rvec_tcp2board.at<float>(0,1);
	T2.at<float>(0,2) = rvec_tcp2board.at<float>(0,2);
	T2.at<float>(1,0) = rvec_tcp2board.at<float>(1,0);
	T2.at<float>(1,1) = rvec_tcp2board.at<float>(1,1);
	T2.at<float>(1,2) = rvec_tcp2board.at<float>(1,2);
	T2.at<float>(2,0) = rvec_tcp2board.at<float>(2,0);
	T2.at<float>(2,1) = rvec_tcp2board.at<float>(2,1);
	T2.at<float>(2,2) = rvec_tcp2board.at<float>(2,2);
	T2.at<float>(0,3) = tvec_tcp2board.at<float>(0,0);
	T2.at<float>(1,3) = tvec_tcp2board.at<float>(0,1);
	T2.at<float>(2,3) = tvec_tcp2board.at<float>(0,2);
	T2.at<float>(3,0) = 0;
	T2.at<float>(3,1) = 0;
	T2.at<float>(3,2) = 0;
	T2.at<float>(3,3) = 1;

	cv::multiply(T1, T2, T1);

	rvec = (cv::Mat_<float>(3,3) << T1.at<float>(0,0), T1.at<float>(0,1), T1.at<float>(0,2), T1.at<float>(1,0), T1.at<float>(1,1), T1.at<float>(1,2), T1.at<float>(2,0), T1.at<float>(2,1), T1.at<float>(2,2));
	tvec = (cv::Mat_<float>(3,1) << T1.at<float>(0,3), T1.at<float>(1,3), T1.at<float>(2,3));
	
	//std::cout << "T1: " << T1 << std::endl;
	//std::cout << rvec << std::endl;
	//std::cout << tvec << std::endl;
	
	std::cout << "tvec: " << tvec << std::endl;
	//cv::transpose(rvec_tcp2board, tvec_tcp2board);
	//cv::multiply(rvec, rvec_tcp2board, rvec);

	//tvec = tvec + tvec_tcp2board;

	
	rtvecs.push_back(rvec);
	rtvecs.push_back(tvec);
	
	//std::cout << rvec << std::endl;
	//std::cout << tvec << std::endl;

	return rtvecs;

}




std::vector <cv::Mat> chessboardPose(cv::Mat img_rgb){

	std::vector <cv::Mat> rtvecs;

	if (img_rgb.empty()){
		std::cout << "input image is empty" << std::endl;
		return rtvecs;
	}

	cv::Mat cameraMatrix = (cv::Mat_<float>(3,3) << 574.0527954101562, 0.0, 319.5, 0.0, 574.0527954101562, 239.5, 0.0, 0.0, 1.0);
	cv::Mat distCoeffs = (cv::Mat_<float>(1,5) << 0.0, 0.0, 0.0, 0.0, 0.0);
	cv::Mat P = (cv::Mat_<float>(3,4) << 574.0527954101562, 0.0, 319.5, 0.0, 0.0, 574.0527954101562, 239.5, 0.0, 0.0, 0.0, 1.0, 0.0);
	cv::Mat rvec;// = cv::Mat(3,1);
	cv::Mat tvec;// = cv::Mat(3,1);

	// Termination criteria
	cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::MAX_ITER +
                cv::TermCriteria::EPS,
                500, // max number of iterations
                0.0001); // min accuracy

	//cv::cvtColor(img_rgb, img_rgb, cv::COLOR_BGR2GRAY);

	
	// Detect chessboard
	cv::Size board_size = cv::Size(5, 6);
	std::vector<cv::Point2f> corners;

	bool board_detected = cv::findChessboardCorners(img_rgb, board_size, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE
                        + cv::CALIB_CB_FAST_CHECK);

	if (!board_detected){
		std::cout << "no chessboard found" << std::endl;
		return rtvecs;
	}
	//std::cout << "Chessboard detected" << std::endl;

	// Draw chessboard corners
	cv::cornerSubPix(img_rgb, corners, cv::Size(11, 11), cv::Size(-1, -1), criteria);
	cv::drawChessboardCorners(img_rgb, board_size, corners, board_detected);

	//cv::imshow("img", img_rgb);
	//cv::waitKey(0);


	float board_height = 6;
	float board_width = 5;
	float square_size = 1;
	std::vector<std::vector<cv::Point3f>> object_points;
	std::vector<std::vector<cv::Point2f>> image_points;

	// Define object points
	std::vector<cv::Point3f> objp;
    for (int i = 0; i < board_height; i++)
      for (int j = 0; j < board_width; j++)
        objp.push_back(cv::Point3f((float)j * square_size, (float)i * square_size, 0));

	image_points.push_back(corners);
    object_points.push_back(objp);

	// Calibrate camera
	std::vector <cv::Mat> rvecs, tvecs;
	cv::calibrateCamera(object_points, image_points, img_rgb.size(), cameraMatrix, distCoeffs, rvecs, tvecs, cv::CALIB_FIX_K4);
	
	//std::cout << "distCoeffs: " << distCoeffs << std::endl;
	//std::cout << "rvecs size: " << rvecs.size() << std::endl;
	//std::cout << "rvec" << rvecs[0] << std::endl;
	//std::cout << "tvec" << tvecs[0] << std::endl;
		
	rtvecs.push_back(rvecs[0]);
	rtvecs.push_back(tvecs[0]);
	
	return rtvecs;
}

std::vector<cv::Mat> handToEyeCalibration(std::string img_folder, std::string rtvec_folder){
	int nTransforms = 0;
	std::vector<cv::Mat> rvecs_cam2board;
	std::vector<cv::Mat> tvecs_cam2board;
	std::vector<cv::Mat> rvecs_base2board;
	std::vector<cv::Mat> tvecs_base2board;

	std::vector<cv::Mat> rtvecs_cam2board;
	std::vector<cv::Mat> rtvecs_base2board;

	cv::Mat img;
	
	while(nTransforms < 16)
	{		
		std::cout << "nTransforms: " << nTransforms << std::endl;
		img = cv::imread(img_folder + std::to_string(nTransforms) + ".png", 0);
		//cv::imshow("img", img);
		//cv::waitKey(0);
		
		
		//bool board_detected = false;
		//while(!board_detected)
		//{	

			if (!(img.empty()))
			{

				rtvecs_cam2board = chessboardPose(img);
				rtvecs_cam2board[0] = rtvecs_cam2board[0];
				rtvecs_cam2board[1] = rtvecs_cam2board[1];

				

				std::cout << rtvecs_cam2board.size() << std::endl;
				


				if (!(rtvecs_cam2board.empty()))
				{
					rtvecs_base2board = convertCSVToMat(rtvec_folder + std::to_string(nTransforms) + ".csv");


					rvecs_cam2board.push_back(rtvecs_cam2board[0]);
					tvecs_cam2board.push_back(rtvecs_cam2board[1]);
					rvecs_base2board.push_back(rtvecs_base2board[0]);
					tvecs_base2board.push_back(rtvecs_base2board[1]);

					
					//board_detected = true;
				}
			}
		//}
		nTransforms++;
	
		std::cout << std::endl;
	}
	
	cv::Mat R_cam2base;
	cv::Mat t_cam2base;

	/*for (int i = 0; i < tvecs_base2board.size(); i++)
		std::cout << tvecs_base2board[i];
	std::cout << std::endl;
*/
	/*for (int i = 0; i < rvecs_base2board.size(); i++)
		std::cout << rvecs_base2board[i];
	std::cout << std::endl;
*/

	cv::calibrateHandEye(rvecs_base2board, tvecs_base2board, rvecs_cam2board, tvecs_cam2board, R_cam2base, t_cam2base, cv::CALIB_HAND_EYE_ANDREFF);

	std::vector<cv::Mat> cam2base{R_cam2base, t_cam2base};

	std::cout << "R: " << R_cam2base << std::endl;
	std::cout << "t: " << t_cam2base << std::endl;
	
	return cam2base;
}


int main(int argc, char **argv)
{
	

	handToEyeCalibration("../data/cali_", "../data/cali_");

	

	return 0; 
}
