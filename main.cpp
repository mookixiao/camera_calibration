#include <io.h>
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

void getFilesName(string& fileDirectory, string& fileType, vector<string>& filesName)
{
    string buffer = fileDirectory + "/*" + fileType;

    intptr_t hFile;
    _finddata_t c_file;
    hFile = _findfirst(buffer.c_str(), &c_file);

    if (hFile == -1L)
        cout << "No " << fileType << " files in current directory!" << endl;
    else
    {
        string fullFilePath;
        do
        {
            fullFilePath.clear();
            fullFilePath = fileDirectory + "/" + c_file.name;
            filesName.push_back(fullFilePath);

        } while (_findnext(hFile, &c_file) == 0);
        _findclose(hFile);
    }
}

void cameraCalibration(vector<string>& filesName, const Size &cornersNum, const Size &gridSize,
                       Mat& cameraMatrix, Mat& distCoeffs, vector<Mat>& rvecsMat, vector<Mat>& tvecsMat)
{
    //// Detect 2D corners ////
    cout << "*****Corners detection starts*****" << endl;

    Size image_size;
    vector<Point2f> image_points;              // 2D corners of one image
    vector<vector<Point2f>> image_points_seq;  // 2D corners of all images
    for (int i = 0; i < filesName.size(); ++i)
    {

        cout << "Image " << filesName[i] << " processing..." << endl;
        Mat img = imread(filesName[i]);
        if (i == 0)  // get width and height only when reads the first image
        {
            image_size.width = img.cols;
            image_size.height = img.rows;
        }

        if (!findChessboardCorners(img, cornersNum, image_points))
        {
            cout << "Detect corners of " << filesName[i] <<  " failed. " << endl;
            waitKey(0);
            return;
        }
        else
        {
            Mat imgGray;
            cvtColor(img, imgGray, COLOR_RGB2GRAY);

            // subpix processing to get more accurate results
            cv::cornerSubPix(imgGray, image_points, cv::Size(10, 10),
                             cv::Size(-1, -1),
                             cv::TermCriteria(TermCriteria::COUNT | TermCriteria::EPS,
                                              20, 0.01));

            image_points_seq.push_back(image_points);
        }
    }

    //// 2D corners -> 3D corners ////
    vector<vector<Point3f>> object_points_seq;
    for (int t = 0; t < filesName.size(); ++t)
    {
        vector<Point3f> object_points;
        for (int i = 0; i < cornersNum.height; ++i)
        {
            for (int j = 0; j < cornersNum.width; ++j)
            {
                Point3f realPoint;
                realPoint.x = i * gridSize.width;
                realPoint.y = j * gridSize.height;
                realPoint.z = 0;  // due to all corners in the same plane
                object_points.push_back(realPoint);
            }
        }
        object_points_seq.push_back(object_points);
    }
    cout << "*****Corners detection ends*****" << endl;

    //// Calibration ////
    cout << "*****Calibration started*****" << endl;
    calibrateCamera(object_points_seq, image_points_seq, image_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat);
    cout << "*****Calibration ended*****" << endl;
}

void saveResults(const string &file, const Mat &cameraMatrix, const Mat &distCoeffs){
    ofstream resultFile(file, ios::out);

    if (!resultFile) {
        cout << "Opening " << file << " failed." << endl;
        return;
    }

    cout << "Intrinsic parameters of this camera: " << endl;
    cout << cameraMatrix << endl;
    resultFile << "Intrinsic parameters of this camera: " << endl;
    resultFile << cameraMatrix << endl;

    resultFile << endl;

    cout << "Distortion coefficients of this camera: " << endl;
    cout << distCoeffs << endl;
    resultFile << "Distortion coefficients of this camera: " << endl;
    resultFile << distCoeffs << endl;

    resultFile.close();

    cout << "results saved to " << file << endl;
}

int main(){
    string resultFile;  // 结果文件
    string fileDirectory;  // 待标定图片文件夹
    string fileType;  // 待标定图片类型

    cout << "Please input where to save results: " << endl;  // "./results.txt"
    cin >> resultFile;
    cout << "Please input where are the pictures: " << endl;  // "C:/Users/Mooki Xiao/OneDrive/code/cpp/camera_calibration/imgs"
    cin >> fileDirectory;
    cout << "Please input which type of pictures to processing: " << endl;  // ".jpg"
    cin >> fileType;

    // 获取文件名
    vector<string> filesName;
    getFilesName(fileDirectory, fileType, filesName);

    Size cornersNum = Size(7, 7);  // 角点行列数
    Size gridSize = Size2d(17.22, 17.22);  // 棋盘格大小

    Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0));  // 相机内参数矩阵
    Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));    // 相机畸变系数
    vector<Mat> rvecsMat;  // 旋转矩阵
    vector<Mat> tvecsMat;  // 平移矩阵

    cameraCalibration(filesName, cornersNum, gridSize, cameraMatrix, distCoeffs, rvecsMat, tvecsMat);
    saveResults(resultFile, cameraMatrix, distCoeffs);

    system("pause");
    return 0;
}