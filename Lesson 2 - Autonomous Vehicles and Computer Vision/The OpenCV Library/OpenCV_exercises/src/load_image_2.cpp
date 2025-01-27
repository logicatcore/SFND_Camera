#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

using namespace std;

void loadImage2()
{
    string windowName = "First steps in OpenCV";
    cv::Mat img;
    cv::namedWindow(windowName, 1); // create window

    for (int i = 5; i <= 9; i++)
    {

        // create file name
        ostringstream imgNumber;                   // #include <sstream>
        imgNumber << setfill('0') << setw(4) << i; // #include <iomanip>
        string filename = "../images/img" + imgNumber.str() + ".png";

        // load and display image
        img = cv::imread(filename);
        cv::imshow(windowName, img);
        cv::waitKey(0); // wait for keyboard input before continuing
    }
}

int main()
{
    loadImage2();
    return 0;
}