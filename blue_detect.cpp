//this code for blue detect ,camera : realsense

//Realsense Header 

#include <librealsense/rs.hpp>
#include "example.hpp"

//OpenCV header

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

//STL header

#include <sstream>
#include <iostream>
#include <vector>

// Color Detection Header

#include "color_detect.hpp"

//ROS header
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "ros_wrapper.hpp"
 
using namespace std;
CvPoint mouseC; 
RosWrapper * g_ros_wrapper;
void onMouse(int event,int x,int y,int flags,void* param);

static void run_opencv(rs::device & dev)
{
    // Determine depth value corresponding to one meter
    const uint16_t one_meter = static_cast<uint16_t>(1.0f / dev.get_depth_scale());

    uint32_t display_mode = 0;
    float ts_height = 0.1;
    cout<<"deth info"<<endl;
    while(true)
    {
        // This call waits until a new coherent set of frames is available on a device
        // Calls to get_frame_data(...) and get_frame_timestamp(...) on a device will return stable values until wait_for_frames(...) is called
        dev.wait_for_frames();
        
       // Retrieve depth data, which was previously configured as a 640 x 480 image of 16-bit depth values
       // const uint16_t * depth_frame = reinterpret_cast<const uint16_t *>(dev.get_frame_data(rs::stream::depth));
        const uint8_t * color_frame = reinterpret_cast<const uint8_t *>(dev.get_frame_data(rs::stream::color));
       // const uint8_t * color_no_align_frame = reinterpret_cast<const uint8_t *>(dev.get_frame_data(rs::stream::color));
       //cv::Mat mColorNoAlignImage(480, 640, CV_8UC3, (void*)color_no_align_frame);
       // cv::cvtColor(mColorNoAlignImage, mColorNoAlignImage, cv::COLOR_RGB2BGR);

        cv::Mat mColorImage(480, 640, CV_8UC3, (void*) color_frame);
        cv::cvtColor(mColorImage, mColorImage, cv::COLOR_RGB2BGR);
        std::vector<cv::Rect> rects;
        color_detect::find_blue(mColorImage, rects, 
            800, // Minimum area threshold
            cv::Scalar(100, 150, 100), // Blue lower bound
            cv::Scalar(140, 255, 255) // Blue upper bound 
        );
        for(auto rect : rects) {
            cv::rectangle(mColorImage, rect.tl(), rect.br(), cv::Scalar(0, 255, 0), 2);
        }    
         circle(mColorImage,mouseC,10,cv::Scalar(0,0,255), 4);   
        // cv::namedWindow( "Color Image", CV_WINDOW_NORMAL );
        cv::imshow( "Color Image" , mColorImage);
       // cv::Mat mImageDepth(480, 640, CV_16UC1, (void*) depth_frame);
       // cv::Mat mScaledDepth;
       // mImageDepth.convertTo( mScaledDepth, CV_8U, 255.0 / one_meter  );

        char key = cv::waitKey(1);
        switch(key) {
            case 27:
                return;
                break;
            default:
                break;
        }

    }
}
int main(int argc, char * argv[]) 
{
    rs::log_to_console(rs::log_severity::warn);
    //rs::log_to_file(rs::log_severity::debug, "librealsense.log");
    mouseC = cvPoint(-100,-100);
     g_ros_wrapper = new RosWrapper(argc, argv, "blue_detect", "chatter");
    rs::context ctx;
    if(ctx.get_device_count() == 0) throw std::runtime_error("No device detected. Is it plugged in?");
    rs::device & dev = *ctx.get_device(0);

    dev.enable_stream(rs::stream::depth, rs::preset::best_quality);
    dev.enable_stream(rs::stream::color, rs::preset::best_quality);
//    try { dev.enable_stream(rs::stream::infrared2, rs::preset::best_quality); } catch(...) {}
    dev.start();
    cout<<"device work"<<endl;
    cv::namedWindow( "Color Image",  CV_WINDOW_NORMAL );    
    cvSetMouseCallback("Color Image",onMouse,NULL);
    run_opencv(dev);

    return EXIT_SUCCESS; 
}
/*
catch(const rs::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch(const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}   
*/void onMouse(int event,int x,int y,int flag,void* param){
    std_msgs::Float32MultiArray arr;
    if(event==CV_EVENT_LBUTTONDOWN||event==CV_EVENT_RBUTTONDOWN){
        arr.data.clear();        
        cout<<"Y:"<<x<< ",X: "<<y<<endl;
        mouseC = cvPoint(x,y);
       
        arr.data.push_back(100);
        arr.data.push_back(x);
        arr.data.push_back(y);
        arr.data.push_back(0);
        //chatter_pub->publish(arr);
        //ros::spinOnce();
        //loop_rate.sleep();
        g_ros_wrapper->publish<std_msgs::Float32MultiArray>(arr);
        
    }
}
