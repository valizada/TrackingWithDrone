//
// Created by Fizuli Valizada on 27/02/2016.
//


#include "ardrone/ardrone.h"

using namespace std;
using namespace cv;

void startTracking();
void findSizeOfImage(Mat mat);
void checkAltitude(int maxAltitudeAllowed);

ARDrone myDrone;

int main(int argc, char *argv[]){

    if (!myDrone.open()){
        cout << "Failed to open connection to the drone." << endl;
        cout << "Check for WIFI connection" << endl;
        return -1;
    }

    // INFO: printing battery percentage
    std::cout << "Battery = " << myDrone.getBatteryPercentage() << "%" << std::endl;

    startTracking();

    myDrone.close();
    return 0;
}


void startTracking(){

    // keyboard input made when user is on application window
    int keyboard;
    // frame from the camera image
    Mat frame;
    // blured image
    Mat frameBlured;
    // frame converted into the HSV colour space
    Mat frameInHSV;
    // this matrix is used to store binary image
    Mat binary;
    // this matrix is used yo do morphological transformations on the binary image
    Mat morpohology;
    // structuring element for morphological transformation
    // I will use default 3*3 matrix
    // TODO: check the use of {3,3}, could be wrong
    Mat SE = getStructuringElement(MORPH_RECT, {3,3});

    // main processing of the frames from the camera and detection
    do{
        keyboard = waitKey(30);

        if ( keyboard == ' '){
            if (myDrone.onGround())
                myDrone.takeoff();
            else
                myDrone.landing();
        }

        frame = myDrone.getImage();
        imshow("Camera of my ARDrone", frame);

        // TODO: should I actually blure it, when I am going to convert it into HSV?
        // TODO: play with the size of the kernel
        GaussianBlur(frame, frameBlured, Size(5,5), 0, 0);
        imshow("Gaussian blured", frameBlured);

        cvtColor(frameBlured, frameInHSV, CV_BGR2HSV_FULL);
        imshow("Image in HSV", frameInHSV);

        // TODO: set correct HSV values
        // now find binary image using inRange function of openCV
        inRange(frameInHSV, {1,2,3}, {1,2,3}, binary);
        imshow("binay", binary);

        // TODO: repeated 2 times, check with 1
        // TODO: consider Top Hat, black hat operations instead of closing
        morphologyEx(binary, morpohology, MORPH_CLOSE, SE, Point(-1,-1), 2);
        imshow("closing 2 times", morpohology);



    } while ((char) keyboard != 'q' && (char) keyboard != 27);
}



void findSizeOfImage(Mat mat){
    int rows = mat.rows;
    int cols = mat.cols;

    cout << "SIZE" << endl;
    cout << "rows: " << rows << " cols: " << cols << endl;
    cv::Size s = mat.size();
    rows = s.height;
    cols = s.width;
    cout << "height: " << rows << " width: " << cols << endl;
    cout << "SIZE" << endl;
}

// TODO: set max altitude for the drone to go
void checkAltitude(int maxAltitudeAllowed){

}