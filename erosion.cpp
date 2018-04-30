#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
static const std::string OPENCV_WINDOW = "Open-CV display window";
using namespace std;


cv::Mat camera_feed;
cv::Mat threshold;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS=50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 20*20;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;
//names that will appear at the top of each window

const string windowName3 = "After Morphological Operations";

void morphOps(cv::Mat &thresh){
    
    //create structuring element that will be used to "dilate" and "erode" image.
    //the element chosen here is a 3px by 3px rectangle

    cv::Mat erodeElement = cv::getStructuringElement( MORPH_RECT,Size(3,3));
    //dilate with larger element so make sure object is nicely visible
    cv::Mat dilateElement = cv::getStructuringElement( MORPH_RECT,Size(10,10));

    cv::erode(thresh,thresh,erodeElement);
    cv::erode(thresh,thresh,erodeElement);
    cv::erode(thresh,thresh,erodeElement);
    cv::erode(thresh,thresh,erodeElement);
    cv::erode(thresh,thresh,erodeElement);

    cv::dilate(thresh,thresh,dilateElement);
    cv::dilate(thresh,thresh,dilateElement);
    cv::dilate(thresh,thresh,dilateElement);
    cv::dilate(thresh,thresh,dilateElement);
    cv::dilate(thresh,thresh,dilateElement);
    cv::dilate(thresh,thresh,dilateElement);
    cv::dilate(thresh,thresh,dilateElement);




}

void trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed){

    cv::Mat temp;
    threshold.copyTo(temp);
    //these two vectors needed for output of findContours
    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;
    //find contours of filtered image using openCV findContours function
    findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
    //use moments method to find our filtered object
    double refArea = 0;
    bool objectFound = false;
    if (hierarchy.size() > 0) {
        int numObjects = hierarchy.size();
        //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
        if(numObjects<MAX_NUM_OBJECTS){
            for (int index = 0; index >= 0; index = hierarchy[index][0]) {

                Moments moment = moments((cv::Mat)contours[index]);
                double area = moment.m00;

                //if the area is less than 20 px by 20px then it is probably just noise
                //if the area is the same as the 3/2 of the image size, probably just a bad filter
                //we only want the object with the largest area so we safe a reference area each
                //iteration and compare it to the area in the next iteration.
                if(area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea){
                    x = moment.m10/area;
                    y = moment.m01/area;
                    objectFound = true;
                    refArea = area;
                }else objectFound = false;


            }
            //let user know you found an object
            if(objectFound ==true){
                putText(cameraFeed,"Tracking Object",Point(0,50),2,1,Scalar(0,255,0),2);
                //draw object location on screen
                drawObject(x,y,cameraFeed);}

        }else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
    }
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "bright_pixel_finder");
    ros::NodeHandle n; 
    cv::VideoCapture capture;
    int x=0,y=0;
    capture.open(1);
    capture.set(CV_CAP_PROP_FRAME_WIDTH, 638);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, 477);
    while(ros::ok()){
        catpure.read(camera_feed);
        cv::Mat im_grey;
        //cv::Mat im_bw;
        cv::cvtColor(feed,im_grey,CV_RGB2GRAY);
        cv::threshold(im_grey,threshold,167,255,THRESH_BINARY);
        morphOps(threshold);
        trackFilteredObject(x,y,threshold,camera_feed);
        //show frames 
        cv::imshow('thresholded Image',threshold);
        cv::imshow('original image',cameraFeed);        

        //delay 30ms so that screen can refresh.
        //image will not appear without this waitKey() command
        cv::waitKey(30);
    }
    


    //g_brightratio= 75; //choose a threshold to define what is "bright" enough
    ros::Duration timer(0.1);


    return 0;
}
