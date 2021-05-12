#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;

void magnitudeSobel()
{
    // load image from file
    cv::Mat img;
    img = cv::imread("../images/img1gray.png");

    // convert image to grayscale
    cv::Mat imgGray;
    cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

    // apply smoothing using the GaussianBlur() function from the OpenCV
    // ToDo : Add your code here
    cv::Mat blurredImage;// = imgGray.clone(); NOT NECESSARY
    cv::Size kernelSize(5, 5);
    float stdDev(2.0);
    cv::GaussianBlur(imgGray, blurredImage, kernelSize, stdDev);

    // create filter kernels using the cv::Mat datatype both for x and y
    // ToDo : Add your code here
    float entriesX[9] = {-1, 0, +1, -2, 0, +2, -1, 0, +1};
    cv::Mat sobelX(3, 3, CV_32F, entriesX);

    float entriesY[9] = {-1, -2, -1, 0, 0, 0, +1, +2, +1};
    cv::Mat sobelY(3, 3, CV_32F, entriesY); 

    // apply filter using the OpenCv function filter2D()
    // ToDo : Add your code here
    cv::Mat gradientX, gradientY;

    cv::filter2D(blurredImage, gradientX, -1, sobelX);//, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);
    cv::filter2D(blurredImage, gradientY, -1, sobelY);//, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);

    // compute magnitude image based on the equation presented in the lesson 
    // ToDo : Add your code here
    cv::Mat magnitude = imgGray.clone();

    for(int i = 0; i < magnitude.rows; i++){
        for(int j = 0; j < magnitude.cols; j++){
            magnitude.at<unsigned char>(i,j) = sqrt(pow(gradientX.at<unsigned char>(i,j), 2) + pow(gradientY.at<unsigned char>(i,j), 2));
        }
    }

    // show result
    string windowName = "Gaussian Blurring";
    cv::namedWindow(windowName, 1); // create window
    cv::imshow(windowName, magnitude);
    cv::waitKey(0); // wait for keyboard input before continuing
}

int main()
{
    magnitudeSobel();
}