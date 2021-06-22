#include <stdio.h>
#include <time.h>
#include <string>
#include <iostream>
#include <iomanip>      // std::setprecision
#include <thread>

#include <ros/ros.h>
#include <ros/package.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Core>

#include <flann/flann.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/common/io.h>
#include <pcl/common/time.h>

#include <pcl/console/print.h>

#include <pcl/io/pcd_io.h>

#include <pcl/ModelCoefficients.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>

#include <pcl/features/don.h>
#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <pcl/surface/mls.h>
#include <pcl/surface/concave_hull.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <vsemi_tof_ros/vsemi_tof_rosConfig.h>

#include "Camera.hpp"
#include "ToFImage.hpp"
#include "settings.h"

using namespace std;

static string package_path = "";

static ros::Publisher cloud_scene_publisher;
static ros::Publisher image_depth_Publisher;
static ros::Publisher image_grayscale_Publisher;
static ros::Publisher image_amplitude_Publisher;

static Settings settings;

static std::string strFrameID = "sensor_frame"; 

uint orientation = 0;

ToFImage* tofImage;
bool running = true;
static bool update_frame_data = false;
static bool process_busy = false;

void updateConfig(vsemi_tof_ros::vsemi_tof_rosConfig &config, uint32_t level)
{
	ROS_INFO("Update config ...");

	settings.mode = static_cast<uint>(config.mode);
	
	settings.image_type = static_cast<uint>(config.image_type);

	settings.hdr = static_cast<uint>(config.hdr);

	settings.automaticIntegrationTime = config.automatic_integration_time;

	settings.integrationTimeATOF0  = static_cast<uint>(config.integration_time_0);
	settings.integrationTimeATOF1  = static_cast<uint>(config.integration_time_1);
	settings.integrationTimeATOF2  = static_cast<uint>(config.integration_time_2);
	settings.integrationTimeATOF3  = static_cast<uint>(config.integration_time_3);
	settings.integrationTimeBTOF4  = static_cast<uint>(config.integration_time_4);
	settings.integrationTimeBTOF5  = static_cast<uint>(config.integration_time_5);

	settings.integrationTimeGray   = static_cast<uint>(config.integration_time_gray);

	settings.minAmplitude0 = static_cast<uint>(config.min_amplitude_0);
	settings.minAmplitude1 = static_cast<uint>(config.min_amplitude_1);
	settings.minAmplitude2 = static_cast<uint>(config.min_amplitude_2);
	settings.minAmplitude3 = static_cast<uint>(config.min_amplitude_3);
	settings.minAmplitude4 = static_cast<uint>(config.min_amplitude_4);
	settings.minAmplitude5 = static_cast<uint>(config.min_amplitude_5);

	settings.dcsFilter = config.dcs_filter;

	settings.roi_leftX   = static_cast<uint>(config.roi_left_x);
	settings.roi_topY    = static_cast<uint>(config.roi_top_y);
	settings.roi_rightX  = static_cast<uint>(config.roi_right_x);
	settings.roi_bottomY = static_cast<uint>(config.roi_bottom_y);

	settings.range   = static_cast<uint>(config.range);

	settings.angle_x = config.angle_x;
	settings.angle_y = config.angle_y;

	settings.pointCloudColor = static_cast<uint>(config.point_cloud_color);

	settings.updateParam = true;
}

void updateCamera(Camera* camera)
{
	ErrorNumber_e status;
	if (settings.hdr == 0)
	{
		status = camera->setHdr(HDR_OFF);
	} else if (settings.hdr == 1)
	{
		status = camera->setHdr(HDR_SPATIAL);
	} else if (settings.hdr == 2)
	{
		status = camera->setHdr(HDR_TEMPORAL);
	}

	if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set HDR failed." << endl;
	std::cout << "\nHDR: " << settings.hdr << std::endl;

	status = camera->setIntegrationTime3d(0, settings.integrationTimeATOF0);
	if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set IntegrationTime3d 0 failed." << endl;
	status = camera->setIntegrationTime3d(1, settings.integrationTimeATOF1);
	if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set IntegrationTime3d 1 failed." << endl;
	status = camera->setIntegrationTime3d(2, settings.integrationTimeATOF2);
	if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set IntegrationTime3d 2 failed." << endl;
	status = camera->setIntegrationTime3d(3, settings.integrationTimeATOF3);
	if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set IntegrationTime3d 3 failed." << endl;
	status = camera->setIntegrationTime3d(4, settings.integrationTimeBTOF4);
	if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set IntegrationTime3d 4 failed." << endl;
	status = camera->setIntegrationTime3d(5, settings.integrationTimeBTOF5);
	if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set IntegrationTime3d 5 failed." << endl;

	status = camera->setIntegrationTimeGrayscale(settings.integrationTimeGray);
	if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set IntegrationTimeGrayscale failed." << endl;

	status = camera->setMinimalAmplitude(0, settings.minAmplitude0);
	if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set MinimalAmplitude 0 failed." << endl;
	status = camera->setMinimalAmplitude(1, settings.minAmplitude1);
	if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set MinimalAmplitude 1 failed." << endl;
	status = camera->setMinimalAmplitude(2, settings.minAmplitude2);
	if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set MinimalAmplitude 2 failed." << endl;
	status = camera->setMinimalAmplitude(3, settings.minAmplitude3);
	if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set MinimalAmplitude 3 failed." << endl;
	status = camera->setMinimalAmplitude(4, settings.minAmplitude4);
	if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set MinimalAmplitude 4 failed." << endl;
	status = camera->setMinimalAmplitude(5, settings.minAmplitude5);
	if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set MinimalAmplitude 5 failed." << endl;

	camera->setRange(0, settings.range);
	std::cout << "\nRange: " << settings.range << std::endl;

	status = camera->setRoi(settings.roi_leftX, settings.roi_topY, settings.roi_rightX, settings.roi_bottomY);
	if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set Roi failed." << endl;

	settings.updateParam = false;
}

void initialise()
{
	ros::NodeHandle nh("~");

	cloud_scene_publisher     = nh.advertise<sensor_msgs::PointCloud2>("cloud_scene", 1);
	image_depth_Publisher     = nh.advertise<sensor_msgs::Image>("image_depth", 1);
	image_grayscale_Publisher = nh.advertise<sensor_msgs::Image>("image_grayscale", 1);
	image_amplitude_Publisher = nh.advertise<sensor_msgs::Image>("image_amplitude", 1);

	//settings.updateParam = false;
}

void publish_cloud(ros::Publisher publisher, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, ros::Time time) {

	pcl::PCLPointCloud2 cloud2;
	pcl::toPCLPointCloud2(*cloud, cloud2);

	sensor_msgs::PointCloud2 ros_msg_pointcloud2;
	pcl_conversions::fromPCL(cloud2, ros_msg_pointcloud2);

	ros_msg_pointcloud2.header.frame_id = strFrameID;
	ros_msg_pointcloud2.header.stamp      = time;

	publisher.publish(ros_msg_pointcloud2);
}

void publish_image(ros::Publisher publisher, cv::Mat image, ros::Time time) {

	sensor_msgs::Image ros_msg;
	ros_msg.header.frame_id = strFrameID;
	ros_msg.height = image.rows;
	ros_msg.width = image.cols;
	ros_msg.encoding = sensor_msgs::image_encodings::BGR8;
	//ros_msg.is_bigendian = (boost::endian::order::native == boost::endian::order::big);
	ros_msg.step = image.cols * image.elemSize();
	size_t size = ros_msg.step * image.rows;
	ros_msg.data.resize(size);

	if (image.isContinuous())
	{
		memcpy((char*)(&ros_msg.data[0]), image.data, size);
	}
	else
	{
		uchar* ros_data_ptr = (uchar*)(&ros_msg.data[0]);
		uchar* cv_data_ptr = image.data;
		for (int i = 0; i < image.rows; ++i)
		{
			memcpy(ros_data_ptr, cv_data_ptr, ros_msg.step);
			ros_data_ptr += ros_msg.step;
			cv_data_ptr += image.step;
		}
	}

	ros_msg.header.stamp   = time;

	publisher.publish(ros_msg);
}

/**
* to process the 3D scene
*/
void process_scene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene, cv::Mat depth_bgr, cv::Mat grayscale, cv::Mat amplitude) {

	if (process_busy) return;

	process_busy = true;

	ros::Time curTime = ros::Time::now();

	// clean up the point cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_clean(new pcl::PointCloud<pcl::PointXYZRGB>);
	std::vector<int> scene_indices;
	pcl::removeNaNFromPointCloud(*scene, *scene_clean, scene_indices);

	// skip empty cloud
	if ((! scene_clean->empty()) && scene_clean->points.size() > 0) {
/*
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
		sor.setInputCloud(scene_clean);
		sor.setMeanK(30);
		sor.setStddevMulThresh(1.0);
		sor.filter(*scene_filtered);
*/
		// scene transform if you know the camera pose
		//Eigen::Matrix3f rotation_matrix3f;
		//rotation_matrix3f = 
		//	  Eigen::AngleAxisf(0.55, Eigen::Vector3f::UnitX())
		//	* Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY())
		//	* Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ());
		//
		//Eigen::Affine3f transform_affine3f = Eigen::Affine3f::Identity();
		//transform_affine3f.translation() << 0, 1.0, -1.0;
		//transform_affine3f.rotate(rotation_matrix3f);
		//pcl::transformPointCloud (*scene_filtered, *scene_filtered, transform_affine3f);

		publish_cloud(cloud_scene_publisher, scene_clean, curTime);

		cvtColor(grayscale, grayscale, cv::COLOR_GRAY2BGR);
		amplitude.convertTo(amplitude, CV_8UC1);
		cvtColor(amplitude, amplitude, cv::COLOR_GRAY2BGR);

		publish_image(image_depth_Publisher, depth_bgr, curTime);
		publish_image(image_grayscale_Publisher, grayscale, curTime);
		publish_image(image_amplitude_Publisher, amplitude, curTime);

	}

	process_busy = false;
} 

/**
* to receive a frame
*/
void tof_image_received()
{
	while (running) {
		if (update_frame_data) {
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_scene(new pcl::PointCloud<pcl::PointXYZRGB>);

			pcl::PointXYZRGB* data_ptr = reinterpret_cast<pcl::PointXYZRGB*>(tofImage->data_3d_xyz_rgb);
			std::vector<pcl::PointXYZRGB> pts(data_ptr, data_ptr + tofImage->n_points);

			cloud_scene->points.insert(cloud_scene->points.end(), pts.begin(), pts.end());

			cv::Mat depth_bgr(tofImage->height, tofImage->width, CV_8UC3, tofImage->data_2d_bgr);
			cv::Mat grayscale(tofImage->height, tofImage->width, CV_8UC1, tofImage->data_grayscale);
			cv::Mat amplitude(tofImage->height, tofImage->width, CV_32F, tofImage->data_amplitude);

			cloud_scene->resize(tofImage->n_points);
			cloud_scene->width = tofImage->n_points;
			cloud_scene->height = 1;
			cloud_scene->is_dense = false;

			if (orientation == 1) {	
				cv::Mat depth_bgr_rotated(tofImage->width, tofImage->height, CV_8UC3, cv::Scalar(0, 0, 0));
				cv::Mat grayscale_rotated(tofImage->width, tofImage->height, CV_8UC1, cv::Scalar(0));
				cv::Mat amplitude_rotated(tofImage->width, tofImage->height, CV_32F, 0.0);

				cv::rotate(depth_bgr, depth_bgr_rotated, cv::ROTATE_90_COUNTERCLOCKWISE);
				cv::rotate(grayscale, grayscale_rotated, cv::ROTATE_90_COUNTERCLOCKWISE);
				cv::rotate(amplitude, amplitude_rotated, cv::ROTATE_90_COUNTERCLOCKWISE);

				Eigen::Matrix3f r;
				r = 
					  Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX())
					* Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY())
					* Eigen::AngleAxisf(-3.14159265 * 0.5, Eigen::Vector3f::UnitZ());

				Eigen::Affine3f t = Eigen::Affine3f::Identity();
				t.translation() << 0, 0, 0;
				t.rotate(r);
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_rotated(new pcl::PointCloud<pcl::PointXYZRGB>);
				pcl::transformPointCloud (*cloud_scene, *scene_rotated, t);

				process_scene(scene_rotated, depth_bgr_rotated.clone(), grayscale_rotated.clone(), amplitude_rotated.clone());
			} else {
				process_scene(cloud_scene, depth_bgr.clone(), grayscale.clone(), amplitude.clone());
			}
			update_frame_data = false;
		} else {
			usleep(1000);
		}
	}
	running = false;
}

void require_tof_image() {

	Camera* camera = Camera::usb_tof_camera_160("/dev/ttyACM0");
	bool success = camera->open();
	if (! success)
	{
		std::cerr << "Failed opening Camera!" << std::endl;
		return;
	}

	ErrorNumber_e status;

	status = camera->setOperationMode(MODE_BEAM_A);
	if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set OperationMode failed." << endl;

	camera->setAcquisitionMode(AUTO_REPEAT);

	tofImage = new ToFImage(camera->getWidth(), camera->getHeight());

	clock_t start, stop;
	int n_frames = 0;
	start = clock();
	while (running) {
		if (settings.updateParam) 
		{
			updateCamera(camera);
			start = clock();
			n_frames = 0;
		} else
		{
			if (settings.image_type == 0) status = camera->getDistance(*tofImage);
			if (settings.image_type == 1) status = camera->getDistanceGrayscale(*tofImage);
			if (settings.image_type == 2) status = camera->getDistanceAmplitude(*tofImage);
			if (status != ERROR_NUMMBER_NO_ERROR)
			{
				std::cerr << "Error: " << status << std::endl;
				break;
			}
			update_frame_data = true;

			n_frames ++;
		}
	}
	stop = clock();
	double interval = (double)(stop - start) / CLOCKS_PER_SEC;
	double frame_rate = ((double) n_frames) / interval;
	std::cout << "Distance frames: " << n_frames << " time spent: " << interval << " frame rate: " << frame_rate << std::endl;

	running = false;

	delete tofImage;
	delete camera;
}

/**
* main
*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "tof_cam_node");

	dynamic_reconfigure::Server<vsemi_tof_ros::vsemi_tof_rosConfig> server;
	dynamic_reconfigure::Server<vsemi_tof_ros::vsemi_tof_rosConfig>::CallbackType f;
	f = boost::bind(&updateConfig, _1, _2);
	server.setCallback(f);

	package_path = ros::package::getPath("vsemi_tof_ros");

	initialise();

	std::thread th_image_received(&tof_image_received);
	th_image_received.detach();

	std::thread th_require_tof_image(&require_tof_image);
	th_require_tof_image.detach();

	ros::spin();

	running = false;
	std::cerr << "Preparing to shutdown ... " << std::endl;
	usleep(2000000);
	std::cerr << "Proceed to shutdown ... " << std::endl;

	ros::shutdown();
}
