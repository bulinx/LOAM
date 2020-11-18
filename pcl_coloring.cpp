/*
  velo2cam_calibration - Automatic calibration algorithm for extrinsic parameters of a stereo camera and a velodyne
  Copyright (C) 2017-2018 Jorge Beltran, Carlos Guindel

  This file is part of velo2cam_calibration.

  velo2cam_calibration is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

  velo2cam_calibration is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with velo2cam_calibration.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
  pcl_coloring: Color the point cloud using the RGB values from the image
*/
#define DEBUG 1
#include <ros/ros.h>
#include "ros/package.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <dynamic_reconfigure/server.h>
#include <pcl_cl/CamRT_Config.h>

using namespace std;
using namespace sensor_msgs;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
ros::Publisher pcl_pub;
string target_frame, source_frame;

double Trans_x,Trans_y,Trans_z,Rotate_Roll,Rotate_Pitch,Rotate_Yaw;
double Trans_x_s,Trans_y_s,Trans_z_s,Rotate_Roll_s,Rotate_Pitch_s,Rotate_Yaw_s;
bool reset_sig;
bool init=1;

void RTcallback(pcl_cl::CamRT_Config &config)
{
        if(init==1){  //init
	Trans_x_s=config.Trans_x;
        Trans_y_s=config.Trans_y;
        Trans_z_s=config.Trans_z;
        Rotate_Roll_s=config.Rotate_Roll;
        Rotate_Pitch_s=config.Rotate_Pitch;
        Rotate_Yaw_s=config.Rotate_Yaw;
        init=0;
        }
        if(config.reset==1){
        config.Trans_x=Trans_x_s;
        config.Trans_y=Trans_y_s;
        config.Trans_z=Trans_z_s;
        config.Rotate_Roll=Rotate_Roll_s;
        config.Rotate_Pitch=Rotate_Pitch_s;
        config.Rotate_Yaw=Rotate_Yaw_s;
        Trans_x=Trans_x_s;
        Trans_y=Trans_y_s;
        Trans_z=Trans_z_s;
        Rotate_Roll=Rotate_Roll_s;
        Rotate_Pitch=Rotate_Pitch_s;
        Rotate_Yaw=Rotate_Yaw_s;
        }
        else{
        Trans_x=config.Trans_x;
        Trans_y=config.Trans_y;
        Trans_z=config.Trans_z;
        Rotate_Roll=config.Rotate_Roll;
        Rotate_Pitch=config.Rotate_Pitch;
        Rotate_Yaw=config.Rotate_Yaw;
        }

}
 
void callback(

const PointCloud2::ConstPtr& pcl_msg,
const CameraInfoConstPtr& cinfo_msg,
const ImageConstPtr& image_msg 

){ 
  if(DEBUG) ROS_INFO("\n\nColouring VELODYNE CLOUD!! %f",Trans_x);;  //test
  // Conversion
  cv_bridge::CvImageConstPtr cv_img_ptr;
  cv::Mat image;
  try{
    image = cv_bridge::toCvShare(image_msg, "bgr8")->image;
  }catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  image_geometry::PinholeCameraModel cam_model_;
  cam_model_.fromCameraInfo(cinfo_msg);

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>); // From ROS Msg
  pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud(new pcl::PointCloud<pcl::PointXYZ>); // After transformation
  PointCloudXYZRGB::Ptr coloured = PointCloudXYZRGB::Ptr(new PointCloudXYZRGB); // For coloring purposes
  PointCloudXYZRGB::Ptr TSDF = PointCloudXYZRGB::Ptr(new PointCloudXYZRGB); //For voxblox
  fromROSMsg(*pcl_msg, *pcl_cloud);

  tf::TransformListener listener;
  tf::StampedTransform transform;
  if(DEBUG) cout << "FRAME ID "<< pcl_cloud->header.frame_id << endl;
  

/*
  try{
    listener.waitForTransform(target_frame.c_str(), source_frame.c_str(), ros::Time(0), ros::Duration(20.0));
    listener.lookupTransform (target_frame.c_str(), source_frame.c_str(), ros::Time(0), transform);
  }catch (tf::TransformException& ex) {
    ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
    return;
  }
*/
    tf::Transform tf_;
    tf::Quaternion q_;
    tf_.setOrigin(tf::Vector3(Trans_x,Trans_y, Trans_z)); //-0.044433, -0.083825, 0.125476
    q_.setRPY(Rotate_Roll,Rotate_Pitch,Rotate_Yaw); //-1.23422, -1.73015, 2.81538 
    tf_.setRotation(q_);
    pcl_ros::transformPointCloud(*pcl_cloud, *trans_cloud, tf_);
  
    tf::Transform tf_inv = tf_.inverse();

  //pcl_ros::transformPointCloud (*pcl_cloud, *trans_cloud, transform);
  trans_cloud->header.frame_id = target_frame;
  coloured->header.frame_id = target_frame;
  //pcl::copyPointCloud(*trans_cloud, *coloured);

 for (pcl::PointCloud<pcl::PointXYZ>::iterator pt = trans_cloud->points.begin(); pt < trans_cloud->points.end(); ++pt)
  {
    cv::Point3d pt_cv((*pt).x, (*pt).y, (*pt).z);
    cv::Point2d uv;
    uv = cam_model_.project3dToPixel(pt_cv);

    pcl::PointXYZRGB p;
        p.x = (*pt).x;
        p.y = (*pt).y;
        p.z = (*pt).z;
        p.r = 0;
        p.g = 0;
        p.b = 0;

    if(uv.x>0 && uv.x < image.cols && uv.y > 0 && uv.y < image.rows && p.z >0){
      // Copy colour to laser pointcloud
      p.b = image.at<cv::Vec3b>(uv)[0];
      p.g = image.at<cv::Vec3b>(uv)[1];
      p.r = image.at<cv::Vec3b>(uv)[2];

     if(p.r!=0 && p.g!=0 && p.b!=0)
      coloured->push_back(p);
    }

  }




  if(DEBUG) ROS_INFO("Publish coloured PC");
  pcl_ros::transformPointCloud(*coloured, *TSDF, tf_inv);
  // Publish coloured PointCloud
  sensor_msgs::PointCloud2 pcl_colour_ros;
  pcl::toROSMsg(*TSDF, pcl_colour_ros);
  pcl_colour_ros.header.stamp = pcl_msg->header.stamp ;
  pcl_pub.publish(pcl_colour_ros);





/*
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (coloured);            //设置输入点云
  pass.setFilterFieldName ("rgb");         //设置过滤时所需要点云类型的Z字段
  pass.setFilterLimits (0, 0, 0);        //设置在过滤字段的范围
  //pass.setFilterLimitsNegative (true);   //设置保留范围内还是过滤掉范围内
  pass.filter (*cloud_full_color);  
*/

}




int main(int argc, char **argv)
{
	ros::init(argc, argv, "pcl_coloring");
        ros::NodeHandle nh_("~");
        nh_.param<std::string>("target_frame", target_frame, "/map"); //nh 读取模式
        nh_.param<std::string>("source_frame", source_frame, "/velodyne");


	dynamic_reconfigure::Server<pcl_cl::CamRT_Config> server;
	dynamic_reconfigure::Server<pcl_cl::CamRT_Config>::CallbackType f;


	f = boost::bind(&RTcallback, _1);
	server.setCallback(f);

        message_filters::Subscriber<PointCloud2> pc_sub(nh_, "pointcloud", 1);
        message_filters::Subscriber<CameraInfo> cinfo_sub(nh_, "camera_info", 1);
        message_filters::Subscriber<Image> image_sub(nh_, "image", 1);

        pcl_pub = nh_.advertise<PointCloud2> ("velodyne_coloured", 1);

        typedef message_filters::sync_policies::ApproximateTime<PointCloud2, CameraInfo, Image> MySyncPolicy;
        // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
        message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pc_sub, cinfo_sub, image_sub);
        sync.registerCallback(boost::bind(&callback, _1, _2, _3 ));
	ros::spin();
	return 0;
}

