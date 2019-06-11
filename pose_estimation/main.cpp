/* This program is intended to estimate the pose of camera relative to the 3D object points in the screen coordinate
 * Estimated pose is from single view intake
 * Initial translation and rotation can be added
 * All input data is in .txt file format
*/

#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <iomanip>

// pose estimation with PnP problem approach
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>


using namespace std;
using namespace cv;

// Function prototyping

// 2D image points extractor from text file data.
vector<Point2d> imagePoints(ifstream& csvImage);

// 3D model object points extractor from text file data.
vector<Point3d> modelPoints(ifstream& csvModel);

// printing rotation vectors into text file
void printRvecs(ofstream& csvRvecs, Mat& rvecs);

// printing translation vectors into text file
void printTvecs(ofstream& csvTvecs, Mat& tvecs);

// printing 3x3 rotation matrix in rodrigues form into text file
void printRodriguesRotation(ofstream& csvR, Mat& rvec, Mat& R);

// create mat object for extrinsic matrix
Mat extrinsic_matrix(Mat& R, Mat& tvec);

// printing extrinsic matrix into text file
void print_extrinsic(ofstream& csvExtrinsicMatrix, Mat& E);

// create mat object for projection matrix
Mat projection_matrix(Mat& camera_matrix, Mat& extrinstic_matrix);

// printing projection matrix into text file
void print_projection_matrix(ofstream& csvProj, Mat& P);


int main(int argc, char **argv)
{

	// Intrinsic Camera matrix input
	// fx 0  cx 
	// 0  fy cy
	// 0  0   1

	ifstream csvIntrinsic("../input_data/camera_intrinsic.txt");

	double arrayIntrinsic[3][3];

	for (int row = 0; row < 3; row++)
	{
		for (int col = 0; col < 3; col++)
		{
			csvIntrinsic >> arrayIntrinsic[row][col];

		}

	}

	Mat camera_matrix = Mat(3, 3, CV_64F, arrayIntrinsic);

	cout << "Camera Matrix " << endl << camera_matrix << endl;

	csvIntrinsic.close();

	cout << endl;

	// distortion coefficient input
	ifstream csvDist_coeffs("../input_data/dist_coeffs.txt");

	double arrayDist_coeffs[5][1];

	for (int row = 0; row < 5; row++)
	{
		for (int col = 0; col < 1; col++)
		{
			csvDist_coeffs >> arrayDist_coeffs[row][col];

		}

	}

	Mat dist_coeffs = Mat(5, 1, CV_64F, arrayDist_coeffs);

	csvDist_coeffs.close();

	cout << "Distortion coefficients " << endl << dist_coeffs << endl;

	cout << endl;

	// 2D image points input in pixel
	ifstream im1;

	im1.open("../input_data/undistorted/x_axis/single_X-100.txt");

	vector<Point2d> image1 = imagePoints(im1);

	im1.close();

	cout << " image points : " << endl << image1 << endl;

	cout << endl;




	// 3D model points input in mm
	ifstream mod1;

	mod1.open("../input_data/model_points_bundle_All.txt");

	vector<Point3d> model_points = modelPoints(mod1);

	mod1.close();

	cout << " model points : " << endl << model_points << endl;

	cout << endl;


	//initial rotation input in radian
	ifstream csvInitialrotation("../input_data/initial_rotation.txt");

	double arrayRot[1][3];

	for (int row = 0; row < 1; row++)
	{
		for (int col = 0; col < 3; col++)
		{
			csvInitialrotation >> arrayRot[row][col];

		}

	}

	Mat rvec = Mat(1, 3, CV_64F, arrayRot);

	cout << "initial rotation: \n" << rvec << "\n" << endl;

	csvInitialrotation.close();

	cout << endl;

	//initial translation input in mm
	ifstream csvInitialtranslation("../input_data/initial_translation.txt");

	double arrayTrans[1][3];

	for (int row = 0; row < 1; row++)
	{
		for (int col = 0; col < 3; col++)
		{
			csvInitialtranslation >> arrayTrans[row][col];

		}

	}

	Mat tvec = Mat(1, 3, CV_64F, arrayTrans);

	cout << "initial translation: \n" << tvec << "\n" << endl;

	csvInitialtranslation.close();

	cout << endl;

	// initialize output file stream for pose estimation result



	// Function to solve the pose
	bool pose_image_1 = solvePnP(model_points, image1, camera_matrix, dist_coeffs, rvec, tvec, 0, SOLVEPNP_ITERATIVE);

	// formating output rotation vector from radian to degree
	ofstream csvRvecs("../output_data/rvecs/Rotation_Vectors.txt");

	printRvecs(csvRvecs, rvec);

	csvRvecs.close();

	// formating output translation into high precision decimal points
	ofstream csvTvecs("../output_data/tvecs/Translation_Vectors.txt");

	printTvecs(csvTvecs, tvec);

	csvTvecs.close();

	// finding 3x3 rotations in rodrigues form and outputing the result into .txt file format
	Mat R(3, 3, CV_64F);

	ofstream csvR("../output_data/rot3x3_rodrigues/Rotation3x3.txt");

	printRodriguesRotation(csvR, rvec, R);

	csvR.close();

	cout << endl;

	// combining 3x3 rotations and translations to make extrinsic matrix output in .txt file format
	ofstream csvExtrinsicMatrix("../output_data/extrinsic_matrix/Extrinsic_Matrix.txt");

	Mat E = extrinsic_matrix(R, tvec);

	print_extrinsic(csvExtrinsicMatrix, E);

	csvExtrinsicMatrix.close();

	cout << endl;

	// initialize projection identity matrix
	Mat P = projection_matrix(camera_matrix, E);

	// finding projection matrix and outputting the result in .txt file format
	ofstream csvProj("../output_data/projection_matrix/Projection_Matrix.txt");

	print_projection_matrix(csvProj, P);

	csvProj.close();

	cout << endl;

	system("pause");
	return 0;

}

//2D image points extractor from text file data.
vector<Point2d> imagePoints(ifstream& csvImage)
{
	vector<Point2d> imagePoints;

	double u, v;

	while (!csvImage.eof())
	{
		csvImage >> u >> v;
		imagePoints.push_back(Point2d(u, v));
	}


	return imagePoints;
}

//3D model object points extractor from text file data.
vector<Point3d> modelPoints(ifstream& csvModel)
{
	vector<Point3d> modelPoints;

	double k, l, m;

	while (!csvModel.eof())
	{
		csvModel >> k >> l >> m;
		modelPoints.push_back(Point3d(k, l, m));
	}


	return modelPoints;
}

// printing rotation vectors into text file
void printRvecs(ofstream& csvRvecs, Mat& rvecs)
{
	double phi = 3.14159;

	cout << "Rotation Vector (in degree):  " << endl << setprecision(17) << (rvecs) * 180 / phi << "\n" << endl;

	csvRvecs << setprecision(17) << rvecs.at<double>(0) * 180 / phi << " " << rvecs.at<double>(1) * 180 / phi << " " << rvecs.at<double>(2) * 180 / phi << endl;
}

// printing translation vectors into text file
void printTvecs(ofstream& csvTvecs, Mat& tvecs)
{
	cout << "Translation Vector (in mm):" << endl << setprecision(17) << tvecs << "\n" << endl;

	csvTvecs << setprecision(17) << tvecs.at<double>(0) << " " << tvecs.at<double>(1) << " " << tvecs.at<double>(2) << endl;
}

// printing 3x3 rotation matrix in rodrigues form into text file
void printRodriguesRotation(ofstream& csvR, Mat& rvec, Mat& R)
{

	Rodrigues(rvec, R);

	cout << "The 3x3 rotation matrix is: \n" << setprecision(17) << R << endl;

	csvR << setprecision(17) << R.at<double>(0, 0) << " " << R.at<double>(0, 1) << " " << R.at<double>(0, 2) << endl;
	csvR << setprecision(17) << R.at<double>(1, 0) << " " << R.at<double>(1, 1) << " " << R.at<double>(1, 2) << endl;
	csvR << setprecision(17) << R.at<double>(2, 0) << " " << R.at<double>(2, 1) << " " << R.at<double>(2, 2) << endl;

}

// create mat object for extrinsic matrix
Mat extrinsic_matrix(Mat& R, Mat& tvec)
{
	Mat E = (Mat_<double>(4, 4) << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), tvec.at<double>(0),
		R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), tvec.at<double>(1),
		R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), tvec.at<double>(2),
		0, 0, 0, 1);

	return E;
}

// printing extrinsic matrix into text file
void print_extrinsic(ofstream& csvExtrinsicMatrix, Mat& E)
{
	cout << "Extrinsic matrix is:  \n" << setprecision(17) << E << endl;

	csvExtrinsicMatrix << setprecision(17) << E.at<double>(0, 0) << " " << E.at<double>(0, 1) << " " << E.at<double>(0, 2) << " " << E.at<double>(0, 3) << endl;
	csvExtrinsicMatrix << setprecision(17) << E.at<double>(1, 0) << " " << E.at<double>(1, 1) << " " << E.at<double>(1, 2) << " " << E.at<double>(1, 3) << endl;
	csvExtrinsicMatrix << setprecision(17) << E.at<double>(2, 0) << " " << E.at<double>(2, 1) << " " << E.at<double>(2, 2) << " " << E.at<double>(2, 3) << endl;
	csvExtrinsicMatrix << setprecision(17) << E.at<double>(3, 0) << " " << E.at<double>(3, 1) << " " << E.at<double>(3, 2) << " " << E.at<double>(3, 3) << endl;
}

// create mat object for projection matrix
Mat projection_matrix(Mat& camera_matrix, Mat& extrinstic_matrix)
{
	Mat ProjIdentity = (Mat_<double>(3, 4) << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0);

	Mat P = camera_matrix * ProjIdentity * extrinstic_matrix;

	return P;
}

// printing projection matrix into text file
void print_projection_matrix(ofstream& csvProj, Mat& P)
{
	cout << "The projection matrix is: \n" << setprecision(17) << P << endl;

	csvProj << setprecision(17) << P.at<double>(0, 0) << " " << P.at<double>(0, 1) << " " << P.at<double>(0, 2) << " " << P.at<double>(0, 3) << endl;
	csvProj << setprecision(17) << P.at<double>(1, 0) << " " << P.at<double>(1, 1) << " " << P.at<double>(1, 2) << " " << P.at<double>(1, 3) << endl;
	csvProj << setprecision(17) << P.at<double>(2, 0) << " " << P.at<double>(2, 1) << " " << P.at<double>(2, 2) << " " << P.at<double>(2, 3) << endl;
}
