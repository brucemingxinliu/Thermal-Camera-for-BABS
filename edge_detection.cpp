//edge detection
// publish result of processed image on topic "/image_converter/output_video"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Float64.h>
#include "kernels.h"

static const std::string OPENCV_WINDOW = "Open-CV display window";
using namespace std;

int g_brightratio; //threshold to decide if a pixel qualifies as dominantly "bright"
int edgeThresh = 1;
int const max lowThreshold = 100;
int ratio = 3;
int kernel size = 3;
char* window_name = "Edge Map";
Mat src, src_gray;
Mat dst, detected_edges;



class ImageConverter {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher centroid_pub_;

public:

    ImageConverter(ros::NodeHandle &nodehandle)
    : it_(nh_) {
        // Subscribe to input video feed and publish output video feed
        image_sub_ = it_.subscribe("usb_cam/image_raw", 1,
                &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);

        cv::namedWindow(OPENCV_WINDOW);
        centroid_pub_ = nodehandle.advertise<std_msgs::Float64>("/centroid", 1);
    }

    ~ImageConverter() {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    //image comes in as a ROS message, but gets converted to an OpenCV type
    void imageCb(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr; //OpenCV data type
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16);//Changed RGB8 to MONO8 becuase the image is one plane.
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        // look for red pixels; turn all other pixels black, and turn red pixels white
        int npix = 0; //count the red pixels
        int isum = 0; //accumulate the column values of red pixels
        int jsum = 0; //accumulate the row values of red pixels
        int greyval;
        int gval;

    int testval;
        cv::Vec2b greypix; // OpenCV representation of an grey pixel
        //comb through all pixels (j,i)= (row,col)
        for (int i = 0; i < cv_ptr->image.cols; i++) {
            for (int j = 0; j < cv_ptr->image.rows; j++) {
                greypix = cv_ptr->image.at<cv::Vec2b>(j, i); //extract an RGB pixel
                //examine intensity of R, G and B components (0 to 255)
                greyval = greypix[0] ; 
                gval = greypix[1];

                
                //look for red values that are large compared to blue+green
              
                //if red (enough), paint this white:
                if (greyval > 125) {
                        cv_ptr->image.at<cv::Vec3b>(j, i)[0] = 255;
                        cv_ptr->image.at<cv::Vec3b>(j, i)[1] = 255;
                        cv_ptr->image.at<cv::Vec3b>(j, i)[2] = 255;
                    npix++; //note that found another bright pixel
                    isum += i; //accumulate row and col index vals
                    jsum += j;
                    //ROS_WARN("Greyval is :%d and Gval is : %d ", jsum , isum );
                }
                else { //else paint it black
                        cv_ptr->image.at<cv::Vec3b>(j, i)[0] = 0;
                        cv_ptr->image.at<cv::Vec3b>(j, i)[1] = 0;
                        cv_ptr->image.at<cv::Vec3b>(j, i)[2] = 0;
                    } 
            }
        }
        //cout << "npix: " << npix << endl;
        //paint in a blue square at the centroid:
        int half_box = 5; // choose size of box to paint
        int i_centroid, j_centroid;
        double x_centroid, y_centroid;
     //   if (npix > 0) {
            i_centroid = isum / npix; // average value of u component of red pixels
            j_centroid = jsum / npix; // avg v component
            x_centroid = ((double) isum)/((double) npix); //floating-pt version
            std_msgs::Float64 centroid_msg;
            centroid_msg.data = x_centroid;
            centroid_pub_.publish(centroid_msg);
            y_centroid = ((double) jsum)/((double) npix);
            ROS_INFO("u_avg: %f; v_avg: %f",x_centroid,y_centroid);
            //cout << "i_avg: " << i_centroid << endl; //i,j centroid of red pixels
            //cout << "j_avg: " << j_centroid << endl;
            for (int i_box = i_centroid - half_box; i_box <= i_centroid + half_box; i_box++) {
                for (int j_box = j_centroid - half_box; j_box <= j_centroid + half_box; j_box++) {
                    //make sure indices fit within the image 
                    if ((i_box >= 0)&&(j_box >= 0)&&(i_box < cv_ptr->image.cols)&&(j_box < cv_ptr->image.rows)) {
                        cv_ptr->image.at<cv::Vec2b>(j_box, i_box)[0] = 255; //(255,0,0) is pure blue
                        cv_ptr->image.at<cv::Vec2b>(j_box, i_box)[1] = 0;
                        cv_ptr->image.at<cv::Vec2b>(j_box, i_box)[2] = 0;
                    }
                }
            }

    //    }
        // Update GUI Window; this will display processed images on the open-cv viewer.
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::waitKey(3); //need waitKey call to update OpenCV image window
        // Also, publish the processed image as a ROS message on a ROS topic
        // can view this stream in ROS with: 
        //rosrun image_view image_view image:=/image_converter/output_video
        image_pub_.publish(cv_ptr->toImageMsg());

    }
}; //end of class definition

// void CannyThreshold(int, void*){
//     blur()

// }
    QImage canny(const QImage& input, float sigma, float tmin, float tmax) {//this could be a potential way to filter the bright pixels
        QImage res = convolution(gaussian_kernel(sigma), input), // Gaussian blur
    // Gradients
        gx = convolution(sobelx, res),
        gy = convolution(sobely, res);
        magnitude(res, gx, gy);
        // Non-maximum suppression
        quint8 *line;
        const quint8 *prev_line, *next_line, *gx_line, *gy_line;
        for (int y = 1; y < res.height() - 1; y++) {
            line = res.scanLine(y);
            prev_line = res.constScanLine(y - 1);
            next_line = res.constScanLine(y + 1);
            gx_line = gx.constScanLine(y);
            gy_line = gy.constScanLine(y);
        }
        for (int x = 1; x < res.width() - 1; x++) {
            double at = atan2(gy_line[x], gx_line[x]);
            const double dir = fmod(at + M_PI, M_PI) / M_PI * 8;
            if ((1 >= dir || dir > 7) && line[x - 1] < line[x] && line[x + 1] < line[x] ||
                (1 < dir || dir <= 3) && prev_line[x - 1] < line[x] && next_line[x + 1] < line[x] ||
                (3 < dir || dir <= 5) && prev_line[x] < line[x] && next_line[x] < line[x] ||
                (5 < dir || dir <= 7) && prev_line[x + 1] < line[x] && next_line[x - 1] < line[x])
            continue;
        line[x] = 0x00;
    }
    }
int main(int argc, char** argv) {
    ros::init(argc, argv, "bright_pixel_finder");
    ros::NodeHandle n; //        
    ImageConverter ic(n); // instantiate object of class ImageConverter

    g_brightratio= 80; //choose a threshold to define what is "bright" enough
    ros::Duration timer(0.1);
    double x, y, z;
    while (ros::ok()) {
        ros::spinOnce();
        timer.sleep();
    }
    return 0;
}
