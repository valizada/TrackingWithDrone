//
// Created by Fizuli Valizada on 27/02/2016.
//


#include "ardrone/ardrone.h"

using namespace std;
using namespace cv;

void startTracking();
void findSizeOfImage(Mat mat);
void checkAltitude(int maxAltitudeAllowed);

int main(int argc, char *argv[]){

    startTracking();
    destroyAllWindows();
    return 0;
}


void startTracking(){

    VideoCapture cap(0); // open the default camera

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
    // TEST: the use of {3,3}, could be wrong
    Mat SE = getStructuringElement(MORPH_RECT, {5,5});

    // main processing of the frames from the camera and detection
    do{
        keyboard = waitKey(30);


        cap.read(frame);
        resize(frame, frame, Size(640,360));

        imshow("Camera of my ARDrone", frame);

        // TODO: should I actually blure it, when I am going to convert it into HSV?
        // TODO: play with the size of the kernel
        GaussianBlur(frame, frameBlured, Size(5,5), 0, 0);
        imshow("Gaussian blured", frameBlured);

        cvtColor(frameBlured, frameInHSV, CV_BGR2HSV_FULL);
        imshow("Image in HSV", frameInHSV);

        // MUST: set correct HSV values
        // now find binary image using inRange function of openCV
        inRange(frameInHSV, Scalar(200,122,0), Scalar(255,255,255), binary);

        imshow("binay", binary);

        // TEST: repeated 2 times, TEST with 1
        // TODO: consider Top Hat, black hat operations instead of closing
        morphologyEx(binary, morpohology, MORPH_CLOSE, SE, Point(-1,-1), 2);
        imshow("closing 2 times", morpohology);

        // getting contours from the binary after morphology by using
        // void findContours(InputOutputArray image, OutputArrayOfArrays contours,
        // int mode, int method, Point offset=Point())

        // lets create OutputArrayOfArrays to hold all the points
        vector<vector<Point>> contours;
        // TODO: should check for mode and method
        findContours(morpohology, contours, CV_RETR_CCOMP, CHAIN_APPROX_SIMPLE);

        int numberOfContours = contours.size();

        // now let's loop through each and every contour set
        // eliminate some given our criteria, which I will indicate
        // above each condition and find biggest square contour to detect
        double maxArea = 10000;
        int indexOfContour = -1;

        for (int i = 0; i < numberOfContours; i++) {

            // first lets eliminate very small ones
            // our image is 360*640 = 230400 pixels, So I wont consider any
            // objects with less than 10 000 pixels, which is around 4% of image
            // subject to change
            // TEST: adjust min pixel size

            // When using contourArea we should define Oriented area flag as well,
            // If it is true, the function returns a signed area value,
            // depending on the contour orientation (clockwise or counter-clockwise).
            // Using this feature we can determine orientation of a contour by taking the
            // sign of an area. By default, the parameter is false, which means that the
            // absolute value is returned.
            double areaOfContour = contourArea(contours[i], true);

            // TEST: checking of area could be done after calling approxPolyDP,
            // which will give approx area, if not change aproxArea variable name too
            // TEST: instead of checking for area could check by width and height
            if (areaOfContour < 10000) continue;

            // then I will try to approximate contour,
            // to try to find square
            // it should have 4 points, but because of distortion it can be detected
            // as more, let's limit with 6, as usually more than 6 it is something
            // close to circle
            vector<Point> approxArea;

            // TODO: arcLength could be tuned
            approxPolyDP(
                    Mat(contours[i]),
                    approxArea,
                    arcLength(Mat(contours[i]), true) * 0.02,
                    true
            );


            if (approxArea.size() >= 4 && approxArea.size() <= 6){
                // let's find aspect ratio first
                Rect rectangle = boundingRect(approxArea);

                float width = rectangle.width;
                float height = rectangle.height;

                float aspectRatio = width/height;

                // let's check if this aspect ratio falls into SQUARE's aspect
                // ratio, which should be 1, but because of different angles
                // of the image I will accept anything between 0.8 and 1.2
                if (aspectRatio < 0.8 || aspectRatio > 1.2) continue;

                // now let's check for solidity, we will need to know
                // convex area of the contour

                // MUST: if it actually gets points of the convex contour
                // because it may return indices of the convex hull points
                vector<Point> hull;
                convexHull(contours[i], hull, false);
                double convexHullArea = contourArea(hull, true);

                double solidity = areaOfContour / convexHullArea;

                // if solidity is too small let's skip this contour
                if (solidity < 0.9) continue;
                // it passes aspect ratio and solidity tests,
                // I will try to update maxArea, simply said to find biggest contour
                if (areaOfContour > maxArea) {
                    indexOfContour = i;
                    maxArea = areaOfContour;
                    cout << "Max area: " << maxArea << endl;
                }
            }
        }
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