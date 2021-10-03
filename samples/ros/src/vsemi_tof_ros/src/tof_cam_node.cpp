#include <iostream>
#include <time.h>
#include <string>
#include <thread>
#include <chrono>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/imgproc.hpp>

#include <pcl/filters/filter.h>

#include <vsemi_tof_ros/vsemi_tof_rosConfig.h>

#include "Camera.hpp"
#include "settings.h"

using namespace std;

static ros::Publisher cloud_scene_publisher;
static ros::Publisher image_depth_Publisher;
static ros::Publisher image_grayscale_Publisher;
static ros::Publisher image_amplitude_Publisher;

static std::string strFrameID = "sensor_frame"; 

static Settings settings;
ToFImage* tofImage;
bool running = true;
static bool update_frame_data = false;
static bool process_busy = false;

void updateConfig(vsemi_tof_ros::vsemi_tof_rosConfig &config, uint32_t level)
{
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
	if (settings.image_type != 1)
	{
		settings.integrationTimeGray   = 0;
	}

	settings.minAmplitude0 = static_cast<uint>(config.min_amplitude_0);
	settings.minAmplitude1 = static_cast<uint>(config.min_amplitude_1);
	settings.minAmplitude2 = static_cast<uint>(config.min_amplitude_2);
	settings.minAmplitude3 = static_cast<uint>(config.min_amplitude_3);
	settings.minAmplitude4 = static_cast<uint>(config.min_amplitude_4);
	settings.minAmplitude5 = static_cast<uint>(config.min_amplitude_5);

	settings.dcsFilter = config.dcs_filter;

	settings.range   = static_cast<uint>(config.range);

	settings.angle_x = config.angle_x;
	settings.angle_y = config.angle_y;

	settings.updateParam = true;
	std::cout << "settings.updateParam: " << settings.updateParam << std::endl;
}

void updateCamera(Camera* camera)
{
	std::cout << "Update camera ... " << std::endl;

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
	std::cout << "HDR: " << settings.hdr << std::endl;
	if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set HDR failed." << endl;

	if (settings.hdr == 2)
	{
		settings.automaticIntegrationTime = false;
	}

	if (settings.automaticIntegrationTime)
	{
		camera->setAutoIntegrationTime3d();
	} else
	{
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

		std::cout << "integrationTimeATOF0: " << settings.integrationTimeATOF0 << std::endl;
		std::cout << "integrationTimeATOF1: " << settings.integrationTimeATOF1 << std::endl;
		std::cout << "integrationTimeATOF2: " << settings.integrationTimeATOF2 << std::endl;
		std::cout << "integrationTimeATOF3: " << settings.integrationTimeATOF3 << std::endl;
		std::cout << "integrationTimeBTOF4: " << settings.integrationTimeBTOF4 << std::endl;
		std::cout << "integrationTimeBTOF5: " << settings.integrationTimeBTOF5 << std::endl;
	}

	status = camera->setIntegrationTimeGrayscale(settings.integrationTimeGray);
	if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set IntegrationTimeGrayscale failed." << endl;
	std::cout << "integrationTimeGray: " << settings.integrationTimeGray << std::endl;

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
	std::cout << "minAmplitude0: " << settings.minAmplitude0 << std::endl;
	std::cout << "minAmplitude1: " << settings.minAmplitude1 << std::endl;
	std::cout << "minAmplitude2: " << settings.minAmplitude2 << std::endl;
	std::cout << "minAmplitude3: " << settings.minAmplitude3 << std::endl;
	std::cout << "minAmplitude4: " << settings.minAmplitude4 << std::endl;
	std::cout << "minAmplitude5: " << settings.minAmplitude5 << std::endl;

	camera->setRange(0, settings.range);
	std::cout << "range: " << settings.range << std::endl;

	camera->setFoV(settings.angle_x, settings.angle_y);
	std::cout << "angle_x: " << settings.angle_x << std::endl;
	std::cout << "angle_y: " << settings.angle_y << std::endl;

	camera->setDcsFilter(settings.dcsFilter);

	settings.updateParam = false;
}

void initialise()
{
	ros::NodeHandle nh("~");

	cloud_scene_publisher     = nh.advertise<sensor_msgs::PointCloud2>("cloud_scene", 1);
	image_depth_Publisher     = nh.advertise<sensor_msgs::Image>("image_depth", 1);
	image_grayscale_Publisher = nh.advertise<sensor_msgs::Image>("image_grayscale", 1);
	image_amplitude_Publisher = nh.advertise<sensor_msgs::Image>("image_amplitude", 1);
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

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_clean(new pcl::PointCloud<pcl::PointXYZRGB>);
	std::vector<int> scene_indices;
	pcl::removeNaNFromPointCloud(*scene, *scene_clean, scene_indices);

	//if ((! scene_clean->empty()) && scene_clean->points.size() > 0) {
		publish_cloud(cloud_scene_publisher, scene_clean, curTime);

		cvtColor(grayscale, grayscale, cv::COLOR_GRAY2BGR);
		amplitude.convertTo(amplitude, CV_8UC1);
		cvtColor(amplitude, amplitude, cv::COLOR_GRAY2BGR);

		publish_image(image_depth_Publisher, depth_bgr, curTime);
		publish_image(image_grayscale_Publisher, grayscale, curTime);
		publish_image(image_amplitude_Publisher, amplitude, curTime);

	//}

	process_busy = false;
} 

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

			process_scene(cloud_scene, depth_bgr.clone(), grayscale.clone(), amplitude.clone());

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
	std::cout << "Camera ID: " << camera->getID() << std::endl;

	ErrorNumber_e status;

	unsigned int device, version;
	status = camera->getIdentification(device, version);
	if (status == ERROR_NUMMBER_NO_ERROR) std::cout << "Identification:\n   device:  " << device << "\n   version: " << version << std::endl;
	else std::cerr << "Error: " << status << std::endl;

	unsigned int major, minor;
	status = camera->getFirmwareRelease(major, minor);
	if (status == ERROR_NUMMBER_NO_ERROR) std::cout << "Firmware:\n   major:   " << major << "\n   minor:   " << minor << std::endl;
	else std::cerr << "Error: " << status << std::endl;

	int16_t temperature;
	status = camera->getTemperature(temperature);
	if (status == ERROR_NUMMBER_NO_ERROR) std::cout << "Temperature         :   " << temperature << std::endl;
	else std::cerr << "Error: " << status << std::endl;

	CameraInfo cameraInfo;
	status = camera->getLensCalibrationData(cameraInfo);
	if (status == ERROR_NUMMBER_NO_ERROR)
	{
		std::cout << "Camera distortion coefficient vector: " << std::endl;
		for(int i = 0; i < 8; i++) {
			std::cout << cameraInfo.D[i] << " ";
		}
		std::cout << std::endl;
		std::cout << "Camera intrinsic matrix: " << std::endl;
		for(int i = 0; i < 9; i++) {
			std::cout << cameraInfo.K[i] << " ";
		}
		std::cout << std::endl;
	} else std::cerr << "Error: " << status << std::endl;

	status = camera->setOperationMode(MODE_BEAM_A);
	if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set Mode failed." << endl;

	status = camera->setModulationFrequency(ModulationFrequency_e::MODULATION_FREQUENCY_20MHZ);
	if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set ModulationFrequency failed." << endl;

	status = camera->setModulationChannel(0, 0);
	if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set tModulationChannel failed." << endl;

	camera->setAcquisitionMode(AUTO_REPEAT);
	if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set AcquisitionMode failed." << endl;

	camera->setOffset(0);
	if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set Offset failed." << endl;

	tofImage = new ToFImage(camera->getWidth(), camera->getHeight());

	std::chrono::steady_clock::time_point st_time;
	std::chrono::steady_clock::time_point en_time;
	double interval, frame_rate;
	int n_frames = 0;
	st_time = std::chrono::steady_clock::now();

	while (running) {
		if (settings.updateParam) 
		{
			if (n_frames > 0)
			{
				en_time = std::chrono::steady_clock::now();
				interval = ((double) std::chrono::duration_cast<std::chrono::microseconds>(en_time - st_time).count()) / 1000000.0;
				frame_rate = ((double) n_frames) / interval;
				std::cout << "Distance frames: " << n_frames << " time spent: " << interval << " frame rate: " << frame_rate << std::endl;
			}

			updateCamera(camera);
			st_time = std::chrono::steady_clock::now();
			n_frames = 0;
		} else
		{
			//std::cerr << "Debug: accuquire a frame ..." << std::endl;
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
	en_time = std::chrono::steady_clock::now();
	interval = ((double) std::chrono::duration_cast<std::chrono::microseconds>(en_time - st_time).count()) / 1000000.0;
	frame_rate = ((double) n_frames) / interval;
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
