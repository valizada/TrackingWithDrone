//
// Created by Fizuli Valizada
//


#include "ardrone/ardrone.h"

using namespace std;
using namespace cv;

void startTracking();

void findSizeOfImage(Mat mat);

void checkAltitude(int maxAltitudeAllowed);

double vx = 0, vy = 0, vz = 0, vr = 0;
const double cooffecient = 0.005;
ARDrone myDrone;

// Thresholds
int minH = 0, maxH = 255;
int minS = 0, maxS = 255;
int minV = 0, maxV = 255;

bool objcetFound = false;

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

    // XML save data
    string filename("thresholds.xml");
    FileStorage fs(filename, cv::FileStorage::READ);

    // If there is a save file then read it
    if (fs.isOpened()) {
        maxH = fs["H_MAX"];
        minH = fs["H_MIN"];
        maxS = fs["S_MAX"];
        minS = fs["S_MIN"];
        maxV = fs["V_MAX"];
        minV = fs["V_MIN"];
        fs.release();
    }

    // Create a window
    cv::namedWindow("closing 2 times");
    // MUST: check of HUE should be 255 or 179
    cv::createTrackbar("H max", "closing 2 times", &maxH, 255);
    cv::createTrackbar("H min", "closing 2 times", &minH, 255);
    cv::createTrackbar("S max", "closing 2 times", &maxS, 255);
    cv::createTrackbar("S min", "closing 2 times", &minS, 255);
    cv::createTrackbar("V max", "closing 2 times", &maxV, 255);
    cv::createTrackbar("V min", "closing 2 times", &minV, 255);
    cv::resizeWindow("closing 2 times", 0, 0);


    startTracking();

    // Save thresholds
    fs.open(filename, cv::FileStorage::WRITE);
    if (fs.isOpened()) {
        cv::write(fs, "H_MAX", maxH);
        cv::write(fs, "H_MIN", minH);
        cv::write(fs, "S_MAX", maxS);
        cv::write(fs, "S_MIN", minS);
        cv::write(fs, "V_MAX", maxV);
        cv::write(fs, "V_MIN", minV);
        fs.release();
    }
//    destroyAllWindows();
    myDrone.close();
    return 0;
}

int keyboard;
Mat frame, frameBlured, frameInHSV, binary, morpohology;
Mat SE = getStructuringElement(MORPH_ELLIPSE, {7, 7});

// MUST: if did not find in 10 consequent frames, let's simply rotate it a bit
void startTracking() {
    double maxArea = 0;

    static cv::Mat1f prediction;
    // Kalman filter
    KalmanFilter kalman(4, 2, 0);

    // Sampling time [s]
    const double dt = 1.0;

    // Transition matrix (x, y, vx, vy)
    cv::Mat1f A(4, 4);
    A << 1.0, 0.0, dt, 0.0,
            0.0, 1.0, 0.0, dt,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0;
    kalman.transitionMatrix = A;

    // Measurement matrix (x, y)
    cv::Mat1f H(2, 4);
    H << 1, 0, 0, 0,
            0, 1, 0, 0;
    kalman.measurementMatrix = H;

    // Process noise covariance (x, y, vx, vy)
    cv::Mat1f Q(4, 4);
    Q << 1e-5, 0.0, 0.0, 0.0,
            0.0, 1e-5, 0.0, 0.0,
            0.0, 0.0, 1e-5, 0.0,
            0.0, 0.0, 0.0, 1e-5;
    kalman.processNoiseCov = Q;

    // Measurement noise covariance (x, y)
    cv::Mat1f R(2, 2);
    R << 1e-1, 0.0,
            0.0, 1e-1;
    kalman.measurementNoiseCov = R;

    double duration;
    int NotFoundCount = 0;
    static bool track = false;

    VideoCapture cap(0);


    // main processing of the frames from the camera and detection
    do {

        clock_t start;
        start = clock();

        keyboard = waitKey(30);

//        cap.read(frame);
//        resize(frame, frame, Size(640, 360));
        frame = myDrone.getImage();
        // imshow("Camera of my ARDrone", frame);

        if (keyboard == ' ') {
            if (myDrone.onGround()) {
                cout << "GROUND" << endl;
                putText(frame, "Flying", Point(330, 640), FONT_HERSHEY_SIMPLEX, 0.5,
                        (track) ? Scalar(0, 0, 255) : Scalar(0, 255, 0), 1, LINE_AA);
                myDrone.takeoff();
            }
            else {
                cout << "NOT GROUND" << endl;
                putText(frame, "On ground", Point(330, 640), FONT_HERSHEY_SIMPLEX, 0.5,
                        (track) ? Scalar(0, 0, 255) : Scalar(0, 255, 0), 1, LINE_AA);
                myDrone.landing();
            }
        }
        objcetFound = false;

        if (keyboard == 't' || keyboard == 'T') track = !track;

        // blured image
        GaussianBlur(frame, frameBlured, Size(5, 5), 0, 0);
        // imshow("Gaussian blured", frameBlured);

        // frame converted into the HSV colour space
        cvtColor(frameBlured, frameInHSV, CV_BGR2HSV_FULL);
        // imshow("Image in HSV", frameInHSV);

        // now find binary image using inRange function of openCV
        // Binary matrix is used to store binary image
        inRange(frameInHSV, Scalar(minH, minS, minV), Scalar(maxH, maxS, maxV), binary);
//        inRange(frameInHSV, Scalar(250, 129, 0), Scalar(255, 255, 255), binary);
        // imshow("binary", binary);

        // TEST: repeated 2 times, TEST with 1
        // TODO: consider Top Hat, black hat operations instead of closing
        morphologyEx(binary, morpohology, MORPH_CLOSE, SE, Point(-1, -1), 2);
        imshow("closing 2 times", morpohology);

        vector<vector<Point>> contours;
        // TODO: should check for mode and method
        findContours(morpohology.clone(), contours, CV_RETR_CCOMP, CHAIN_APPROX_SIMPLE);

        size_t numberOfContours = contours.size();
        // now let's loop through each and every contour set
        // eliminate some given our criteria, which I will indicate
        // above each condition and find biggest square contour to detect
        int indexOfContour = -1;
        vector<Point> resultApproxArea;


        double altitude = myDrone.getAltitude();
        cout << "Altitude: " << altitude << endl;
        if (altitude >= 1.5) {
            putText(frame, "TOO HIGH HIGH, MOVING DOWN", Point(50, 100), FONT_HERSHEY_SIMPLEX, 0.5,
                    (track) ? Scalar(0, 0, 255) : Scalar(0, 255, 0), 1, LINE_AA);
            cout << "TOO HIGH, ALTITUDE IS: " << myDrone.getAltitude() << endl;
            myDrone.move3D(0, 0, -0.1, 0);
            continue;
        }

        cout << "START" << endl;
        for (size_t i = 0; i < numberOfContours; i++) {
            double areaOfContour = contourArea(contours[i], false);

            if (areaOfContour < 300) continue;

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
                if (aspectRatio < 0.6 || aspectRatio > 2.5) {
                    cout << "did NOT PASS ASPECT RATIO checking" << endl;
                    continue;
                }

                // now let's check for solidity, we will need to know
                // convex area of the contour
                vector<Point> hull;
                convexHull(contours[i], hull, false);
                double convexHullArea = contourArea(hull, true);

                // TODO: try this area
//                double area = contourArea(Mat(approxArea), false);

                double solidity = areaOfContour / convexHullArea;

                cout << "solidity: " << solidity << " where area: " << areaOfContour << " convex area: " <<
                convexHullArea << endl;
                // if solidity is too small let's skip this contour
                if (solidity < 0.7) {
                    cout << "NOT PASS SOLIDITY" << endl;
                    continue;
                }
                // it passes aspect ratio and solidity tests,
                // I will try to update maxArea, simply said to find biggest contour

                // TODO: try this area
//                double area = contourArea(Mat(approxArea), false);
//                if (area > maxArea){
                if (areaOfContour > maxArea) {
                    indexOfContour = i;
                    resultApproxArea = approxArea;
                    maxArea = areaOfContour;
                    // cout << "Max area: " << maxArea << endl;
                }
                // cout << "Max area: " << maxArea << " contour area: " << areaOfContour << endl;
            }
        } // for = looping over all the contours

        int centerY = morpohology.rows / 2;
        int centerX = morpohology.cols / 2;

        // if contour passed all the tests, so it is detected
        if (indexOfContour >= 0) {
            objcetFound = true;
            cout << "MAKE COUNT 0" << endl;
            NotFoundCount = 0;
            // let's find center of the object by using Moments
            Moments moment = moments(contours[indexOfContour], true);

            double marker_y = (int) (moment.m01 / moment.m00);
            double marker_x = (int) (moment.m10 / moment.m00);
            // TODO: try this
//            Rect rect = boundingRect(resultApproxArea);

            Rect rect = boundingRect(contours[indexOfContour]);
            rectangle(frame, rect, Scalar(0, 255, 0));


            // Measurements
            cv::Mat measurement = (Mat1f(2, 1) << marker_x, marker_y);
            // Correction
            cv::Mat estimated = kalman.correct(measurement);
            if (track) {
                circle(frame, Point(marker_x, marker_y), 1, Scalar(0, 255, 0));
                // let's draw this center of the image
                circle(frame, Point(centerX, centerY), 1, Scalar(0, 255, 0));

                if (maxArea < 40000) {
                    vx = 0.13;
                    vy = 0.0;
                    vz = 0.004 * (centerY - marker_y);
                    vr = 0.005 * (centerX - marker_x);

                    putText(frame, "WITHOUT KALMAN", Point(30, 250), FONT_HERSHEY_SIMPLEX, 0.5,
                            (track) ? Scalar(0, 0, 255) : Scalar(0, 255, 0), 1, LINE_AA);
                    cout << "WITHOUT KALMAN" << endl;
                    cout << "centerX: " << centerX << " centerY: " << centerY << endl;
                    cout << "marker_x: " << marker_x << " marker_y: " << marker_y << endl;
                    cout << "tracks: vx = " << vx << " vy = " << vy << " vz = " << vz
                    << " vr = " << vr << endl;
                    cout << "Current altitude is: " << myDrone.getAltitude() << endl;
                    myDrone.move3D(vx, vy, vz, vr);
                }
                else {
                    putText(frame, "DANGER MOVING BACK", Point(30, 100), FONT_HERSHEY_SIMPLEX, 0.5,
                            (track) ? Scalar(0, 0, 255) : Scalar(0, 255, 0), 1, LINE_AA);
                    vx = -0.1;
                    vy = 0.0;
                    vz = 0.005 * (centerY - marker_y);
                    vr = 0.005 * (centerX - marker_x);
                    // MUST: try with all the values
                    myDrone.move3D(vx, 0, 0, 0);
                }
            }
        } //if : checking if object is found


        // Prediction
        cv::Mat1f prediction = kalman.predict();
        int radius = 1e+3 * kalman.errorCovPre.at<float>(0, 0);


        circle(frame, Point(prediction(0, 0), prediction(0, 1)), 1, Scalar(255, 0, 0));

        cout << "KALMAN PREDICTION RESULT: " << "x: " << prediction(0, 0) << " y: " << prediction(0, 1) << endl;

        if (!objcetFound && track) { // && NotFoundCount <= 30) {
            NotFoundCount++;
//            cout << "WITH KALMAN" << endl;
//            putText(frame, "WITH KALMAN", Point(30, 250), FONT_HERSHEY_SIMPLEX, 0.5,
//                    (track) ? Scalar(0, 0, 255) : Scalar(0, 255, 0), 1, LINE_AA);
//
//            if (prediction(0, 1) > 500 || prediction(0, 1) <= 0
//                || prediction(0, 0) > 300 || prediction(0, 0) <= 0) {
//                prediction(0, 0) = 0;
//                prediction(0, 1) = 0;
//            }
//
//            if (prediction(0, 1) != 0 && prediction(0, 0) != 0) {
//                vx = 0.13;
//                vy = 0.0;
//                vz = 0.003 * (centerY - prediction(0, 1));
//                vr = 0.003 * (centerX - prediction(0, 0));
//
//                cout << "centerX: " << centerX << " centerY: " << centerY << endl;
//                cout << "marker_x: " << prediction(0, 1) << " marker_y: " << prediction(0, 0) << endl;
//                cout << "tracks: vx = " << vx << " vy = " << vy << " vz = " << vz
//                << " vr = " << vr << endl;
//
//                cout << "Current altitude is: " << myDrone.getAltitude() << endl;
////                myDrone.move3D(vx, vy, vz, vr);
//            }
            // MUST: consider moving backwards
//            myDrone.move3D(0, 0, 0, 0.36);
        }

        if (!objcetFound && track && NotFoundCount > 30) {
            myDrone.move3D(0, 0, 0, 0.12 * (vr / fabs(vr)));
            NotFoundCount++;
        }

        putText(frame, track ? "TRACKING" : "NOT TRACKING", Point(7, 12), FONT_HERSHEY_SIMPLEX, 0.5,
                (track) ? Scalar(0, 0, 255) : Scalar(0, 255, 0), 1, LINE_AA);

        imshow("result", frame);
        maxArea = 0;

        clock_t end = clock();
        cout << "time start: " << start << " end: " << end << endl;
//        double time = difftime(end, start);

//        std::cout << "TIME: " << time << '\n';

        if (track)
            cout << "TRACK ON" << endl;
        else cout << "TRACK OFF" << endl;
        cout << "END NotFoundCount is: " << NotFoundCount << endl;

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