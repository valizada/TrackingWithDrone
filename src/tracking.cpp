//
// Created by Fizuli Valizada
//


#include "ardrone/ardrone.h"

using namespace std;
using namespace cv;

void startTracking();

void findSizeOfImage(Mat mat);

void checkAltitude(int maxAltitudeAllowed);

// velocities
// vx is the depth, so moving inside of the image
// vy is rotating to left and right
// vz is moving horizontally
// vr rotational speed
double vx = 0, vy = 0, vz = 0, vr = 0;
// this is used to adjust movement, PID coefficient
const double cooffecient = 0.005;
ARDrone myDrone;

int main(int argc, char *argv[]) {

    if (!myDrone.open()) {
        cout << "Failed to open connection to the drone." << endl;
        cout << "Check for WIFI connection" << endl;
        return -1;
    }

    // INFO: printing battery percentage
    std::cout << "Battery = " << myDrone.getBatteryPercentage() << "%" << std::endl;

//    myDrone.setFlatTrim();
//    myDrone.setCalibration(0);
    startTracking();
//    destroyAllWindows();
    myDrone.close();
    return 0;
}

// keyboard input made when user is on application window
int keyboard;
// frame from the camera image
Mat frame, frameBlured, frameInHSV, binary, morpohology;
// structuring element for morphological transformation
// I will use default 3*3 matrix
// for closing
Mat SE = getStructuringElement(MORPH_RECT, {5, 5});

// MUST: if did not find in 10 consequent frames, let's simply rotate it a bit
void startTracking() {
    double maxArea = 0;

    static bool track = false;
    // main processing of the frames from the camera and detection
    do {
        keyboard = waitKey(30);

        if (keyboard == ' ') {
            if (myDrone.onGround())
                myDrone.takeoff();
            else
                myDrone.landing();
        }

        frame = myDrone.getImage();
        // imshow("Camera of my ARDrone", frame);

        if (keyboard == 't' || keyboard == 'T') {
            track = !track;
            putText(frame, track ? "TRACKING" : "NOT TRACKING", Point(7, 12), FONT_HERSHEY_SIMPLEX, 0.5,
                    (track) ? Scalar(0, 0, 255) : Scalar(0, 255, 0), 1, LINE_AA);
        }

        // blured image
        GaussianBlur(frame, frameBlured, Size(5, 5), 0, 0);
        // imshow("Gaussian blured", frameBlured);

        // frame converted into the HSV colour space
        cvtColor(frameBlured, frameInHSV, CV_BGR2HSV_FULL);
        // imshow("Image in HSV", frameInHSV);

        // now find binary image using inRange function of openCV
        // Binary matrix is used to store binary image
        inRange(frameInHSV, Scalar(250, 129, 0), Scalar(255, 255, 255), binary);
        // imshow("binary", binary);

        // TEST: repeated 2 times, TEST with 1
        // TODO: consider Top Hat, black hat operations instead of closing
        // morpohology is used to do morphological transformations on the binary image
        morphologyEx(binary, morpohology, MORPH_CLOSE, SE, Point(-1, -1), 2);
        imshow("closing 2 times", morpohology);

        // getting contours from the binary after morphology by using
        // void findContours(InputOutputArray image, OutputArrayOfArrays contours,
        // int mode, int method, Point offset=Point())
        // Lets create OutputArrayOfArrays to hold all the points
        vector<vector<Point>> contours;
        // TODO: should check for mode and method
        findContours(morpohology, contours, CV_RETR_CCOMP, CHAIN_APPROX_SIMPLE);

        unsigned long numberOfContours = contours.size();
        // now let's loop through each and every contour set
        // eliminate some given our criteria, which I will indicate
        // above each condition and find biggest square contour to detect
        int indexOfContour = -1;

        vector<Point> resultApproxArea;

        cout << "START |" << "num of contours: " << numberOfContours << endl;
        for (size_t i = 0; i < numberOfContours; i++) {
            // When using contourArea we should define Oriented area flag as well,
            // If it is true, the function returns a signed area value,
            // depending on the contour orientation (clockwise or counter-clockwise).
            // Using this feature we can determine orientation of a contour by taking the
            // sign of an area. By default, the parameter is false, which means that the
            // absolute value is returned.
            double areaOfContour = contourArea(contours[i], false);

            // first lets eliminate very small ones
            // our image is 360*640 = 230400 pixels, So I wont consider any
            // objects with less than 10 000 pixels, which is around 4% of image
            // subject to change
            if (areaOfContour < 300) continue;

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
                    arcLength(Mat(contours[i]), true) * 0.01,
                    true
            );

//            cout << "APPROX CORNERS SIZE = " << approxArea.size() << endl;
            if (approxArea.size() >= 4 && approxArea.size() <= 7) {
                // let's find aspect ratio first
                Rect rectangle = boundingRect(approxArea);

                float width = rectangle.width;
                float height = rectangle.height;

                float aspectRatio = width / height;
                cout << "width: " << width << " " << " height: " << height << " aspect ration: " << aspectRatio << endl;

                // let's check if this aspect ratio falls into SQUARE's aspect
                // ratio, which should be 1, but because of different angles
                // of the image I will accept anything between 0.8 and 1.2
                if (aspectRatio < 0.73 || aspectRatio > 1.45) {
                    cout << "did NOT PASS ASPECT RATIO checking" << endl;
                    continue;
                }

                // now let's check for solidity, we will need to know
                // convex area of the contour
                vector<Point> hull;
                convexHull(contours[i], hull, false);
                double convexHullArea = contourArea(hull, true);

                double solidity = areaOfContour / convexHullArea;

                cout << "solidity: " << solidity << " where area: " << areaOfContour << " convex area: " <<
                convexHullArea << endl;
//                // if solidity is too small let's skip this contour
                if (solidity < 0.9) {
//                    cout << "NOT PASS SOLIDITY" << endl;
                    continue;
                }
                // it passes aspect ratio and solidity tests,
                // I will try to update maxArea, simply said to find biggest contour

                if (areaOfContour > maxArea) {
                    indexOfContour = i;
                    resultApproxArea = approxArea;
                    maxArea = areaOfContour;
                    // cout << "Max area: " << maxArea << endl;
                }
                // cout << "Max area: " << maxArea << " contour area: " << areaOfContour << endl;
            }
        } // for = looping over all the contours

        // if contour passed all the tests, so it is detected
        if (indexOfContour >= 0) {
            // let's find center of the object by using Moments
            Moments moment = moments(contours[indexOfContour], true);

            double marker_y = (int) (moment.m01 / moment.m00);
            double marker_x = (int) (moment.m10 / moment.m00);
            Rect rect = boundingRect(contours[indexOfContour]);
            rectangle(frame, rect, Scalar(0, 255, 0));

            if (track) {
                // center coordinates of the image
                // TODO: should be calculated just once
                int centerY = morpohology.rows / 2;
                int centerX = morpohology.cols / 2;

                // let's draw this center of the object to be tracked
                circle(frame, Point(marker_x, marker_y), 1, Scalar(0, 255, 0));
                // let's draw this center of the image
                circle(frame, Point(centerX, centerY), 1, Scalar(0, 255, 0));

                if (maxArea < 30000)
                    vx = 0.13;
                else {
                    putText(frame, "DANGER MOVING BACK", Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5,
                            (track) ? Scalar(0, 0, 255) : Scalar(0, 255, 0), 1, LINE_AA);
                    vx = -0.1;
                }
                vy = 0.0;
                vz = 0.004 * (centerY - marker_y);
                vr = 0.005 * (centerX - marker_x);
                cout << "centerX: " << centerX << " centerY: " << centerY << endl;
                cout << "marker_x: " << marker_x << " marker_y: " << marker_y << endl;
                cout << "tracks: vx = " << vx << " vy = " << vy << " vz = " << vz
                << " vr = " << vr << endl;
                cout << "Current altitude is: " << myDrone.getAltitude() << endl;
                if (myDrone.getAltitude() >= 2)
                    myDrone.move3D(0, 0, -0.1, 0);

                myDrone.move3D(vx, vy, vz, vr);
            }
        } //if : checking if object is found
        else {
            // every time it does not find anything let's rotate it a bit
            // MUST: to be changed after implementing Kalman filterin
//            if (track)
//                myDrone.move3D(0, 0, 0, 0.2);
        }

        imshow("result", frame);
        maxArea = 0;

        cout << "END" << endl;
    } while ((char) keyboard != 'q' && (char) keyboard != 27);
}


void findSizeOfImage(Mat mat) {
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
void checkAltitude(double maxAltitudeAllowed) {
    double altitude = myDrone.getAltitude();
    if (maxAltitudeAllowed - altitude == 0.2)
        myDrone.move3D(0, 0, -0.1, 0);
}