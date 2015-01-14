#include "haptic_teleoperation/VirtualForcePrf.h"
#include <gtest/gtest.h>

// Declare a test
TEST(ForceField, fieldPrfTest)
{
    /*
    Eigen::Vector3d f;
    Eigen::Vector3d robotVelocity(0,0,0);
    double laserRange = 4; // 4 meters
    double laserResolution = 0.005; // 5mm
    int numberOfPixels = int(2*laserRange/laserResolution);
    double obstX,obstY;
    geometry_msgs::Point32 currentPose;

    // Initialize image container with all black
    cv::Mat img(numberOfPixels,numberOfPixels, CV_8UC3, cv::Scalar(0,0,0));
    cv::Mat image = img;
    double maxF = 0;
    double minF = 1000000;
    for(int x=0;x<img.cols;x++)
    {
        for(int y=0;y<img.rows;y++)
        {
            obstX = (x - img.cols/2.0)*laserResolution;
            obstY = (img.rows/2.0 - y)*laserResolution;
            currentPose.x = obstX;
            currentPose.y = obstY;
            currentPose.z = 0;
            f = this->getForcePoint(currentPose,robotVelocity);
            double F = sqrt(f(0)*f(0) + f(1)*f(1) + f(2)*f(2));

            if(F>maxF)
                maxF = F;
            if(F<minF)
                minF = F;
            // This is how you get a pixel
            Vec3b color = image.at<cv::Vec3b>(cv::Point(x,y));
            color.val[0] = uchar(F * 255.0);
            color.val[1] = uchar(F * 255.0);
            color.val[2] =  uchar(F * 255.0);
            image.at<cv::Vec3b>(cv::Point(x,y)) = color;
        }
    }
    std::cout<<"Max f is:"<<maxF<<" min f:"<<minF<<"\n";
    imwrite(testName, img);
    */
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    /*
    ros::init(argc, argv, "prf");
    ros::NodeHandle n;
    double dmin= 1.2 ;
    double amax= 1.0 ;
    double rpz = 0.4 ;
    double tahead = 2 ;

    Eigen::Vector3d robotVel ;
    robotVel(0) = 0 ;
    robotVel(1) = 0 ;
    robotVel(2) = 0 ;
    VirtualForcePrf prf_obj(n);
    prf_obj.setParameters(dmin,amax,rpz,tahead) ;
    */
    return RUN_ALL_TESTS();
}
