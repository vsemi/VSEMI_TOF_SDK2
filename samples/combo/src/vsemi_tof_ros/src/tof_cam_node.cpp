#include <iostream>
#include <string>
#include <thread>
#include <map>
#include <iterator>
#include <chrono>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/imgproc.hpp>

#include <pcl/filters/filter.h>
#include <pcl/common/transforms.h>

#include <vsemi_tof_ros/vsemi_tof_rosConfig.h>

#include "Camera.hpp"
#include "settings.h"

using namespace std;

std::map<uint32_t, ros::Publisher> cloud_publishers;

static ros::Publisher image_depth_Publisher;
static ros::Publisher image_grayscale_Publisher;
static ros::Publisher image_amplitude_Publisher;

static std::string strFrameID = "sensor_frame"; 

static Settings globle_settings;

bool running = true;
static bool process_busy = false;

class Channel
{
private:
	Settings* _settings;
	std::string _port;
	Camera* _camera;

public:
	ToFImage* _tofImage;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud;
	cv::Mat depth_bgr;
	cv::Mat grayscale;
	cv::Mat amplitude;

	bool frame_ready = false;

	Channel(std::string port, Settings* settings)
	{
		_settings = settings;
		_port = port;

		_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	}
	~Channel()
	{
		delete _tofImage;
		delete _camera;
	}
	uint32_t getID()
	{
		uint32_t id = _camera->getID();
		return id;
	}
	
	bool open()
	{
		_camera = Camera::usb_tof_camera_160(_port);
		bool success = _camera->open();
		if (! success)
		{
			return false;
		}

		ErrorNumber_e status;

		status = _camera->setOperationMode(MODE_BEAM_A);
		if (status != ERROR_NUMMBER_NO_ERROR) 
		{
			cerr << "Set Mode failed." << endl;
			return false;
		}

		status = _camera->setModulationFrequency(ModulationFrequency_e::MODULATION_FREQUENCY_20MHZ);
		if (status != ERROR_NUMMBER_NO_ERROR) 
		{
			cerr << "Set ModulationFrequency failed." << endl;
			return false;
		}

		status = _camera->setModulationChannel(0, 0);
		if (status != ERROR_NUMMBER_NO_ERROR) 
		{
			cerr << "Set tModulationChannel failed." << endl;
			return false;
		}

		_camera->setAcquisitionMode(AUTO_REPEAT);

		_camera->setOffset(0);

		_tofImage = new ToFImage(_camera->getWidth(), _camera->getHeight());

		return true;
	}

	void update()
	{
		update(0);
	}

	void update(int modulationChannel)
	{
		std::cout << "Update camera ... " << std::endl;

		ErrorNumber_e status;

		status = _camera->setModulationChannel(0, modulationChannel);
		if (status != ERROR_NUMMBER_NO_ERROR) 
		{
			cerr << "Set ModulationChannel failed." << endl;
		}

		if (_settings->hdr == 0)
		{
			status = _camera->setHdr(HDR_OFF);
		} else if (_settings->hdr == 1)
		{
			status = _camera->setHdr(HDR_SPATIAL);
		} else if (_settings->hdr == 2)
		{
			status = _camera->setHdr(HDR_TEMPORAL);
		}
		std::cout << "HDR: " << _settings->hdr << std::endl;
		if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set HDR failed." << endl;

		if (_settings->hdr == 2)
		{
			_settings->automaticIntegrationTime = false;
		}

		if (_settings->automaticIntegrationTime)
		{
			_camera->setAutoIntegrationTime3d();
		} else
		{
			status = _camera->setIntegrationTime3d(0, _settings->integrationTimeATOF0);
			if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set IntegrationTime3d 0 failed." << endl;
			status = _camera->setIntegrationTime3d(1, _settings->integrationTimeATOF1);
			if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set IntegrationTime3d 1 failed." << endl;
			status = _camera->setIntegrationTime3d(2, _settings->integrationTimeATOF2);
			if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set IntegrationTime3d 2 failed." << endl;
			status = _camera->setIntegrationTime3d(3, _settings->integrationTimeATOF3);
			if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set IntegrationTime3d 3 failed." << endl;
			status = _camera->setIntegrationTime3d(4, _settings->integrationTimeBTOF4);
			if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set IntegrationTime3d 4 failed." << endl;
			status = _camera->setIntegrationTime3d(5, _settings->integrationTimeBTOF5);
			if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set IntegrationTime3d 5 failed." << endl;

			std::cout << "integrationTimeATOF0: " << _settings->integrationTimeATOF0 << std::endl;
			std::cout << "integrationTimeATOF1: " << _settings->integrationTimeATOF1 << std::endl;
			std::cout << "integrationTimeATOF2: " << _settings->integrationTimeATOF2 << std::endl;
			std::cout << "integrationTimeATOF3: " << _settings->integrationTimeATOF3 << std::endl;
			std::cout << "integrationTimeBTOF4: " << _settings->integrationTimeBTOF4 << std::endl;
			std::cout << "integrationTimeBTOF5: " << _settings->integrationTimeBTOF5 << std::endl;
		}

		if (_settings->image_type == 1)
		{
			status = _camera->setIntegrationTimeGrayscale(_settings->integrationTimeGray);
		} else{
			status = _camera->setIntegrationTimeGrayscale(0);
		}
		if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set IntegrationTimeGrayscale failed." << endl;
		std::cout << "integrationTimeGray: " << _settings->integrationTimeGray << std::endl;

		status = _camera->setMinimalAmplitude(0, _settings->minAmplitude0);
		if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set MinimalAmplitude 0 failed." << endl;
		status = _camera->setMinimalAmplitude(1, _settings->minAmplitude1);
		if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set MinimalAmplitude 1 failed." << endl;
		status = _camera->setMinimalAmplitude(2, _settings->minAmplitude2);
		if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set MinimalAmplitude 2 failed." << endl;
		status = _camera->setMinimalAmplitude(3, _settings->minAmplitude3);
		if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set MinimalAmplitude 3 failed." << endl;
		status = _camera->setMinimalAmplitude(4, _settings->minAmplitude4);
		if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set MinimalAmplitude 4 failed." << endl;
		status = _camera->setMinimalAmplitude(5, _settings->minAmplitude5);
		if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set MinimalAmplitude 5 failed." << endl;
		std::cout << "minAmplitude0: " << _settings->minAmplitude0 << std::endl;
		std::cout << "minAmplitude1: " << _settings->minAmplitude1 << std::endl;
		std::cout << "minAmplitude2: " << _settings->minAmplitude2 << std::endl;
		std::cout << "minAmplitude3: " << _settings->minAmplitude3 << std::endl;
		std::cout << "minAmplitude4: " << _settings->minAmplitude4 << std::endl;
		std::cout << "minAmplitude5: " << _settings->minAmplitude5 << std::endl;

		_camera->setRange(0, _settings->range);
		std::cout << "range: " << _settings->range << std::endl;

		_camera->setFoV(_settings->angle_x, _settings->angle_y);
		std::cout << "angle_x: " << _settings->angle_x << std::endl;
		std::cout << "angle_y: " << _settings->angle_y << std::endl;

		_camera->setDcsFilter(_settings->dcsFilter);
	}

	std::thread thNextFrame() {
		return std::thread([=] { nextFrame(); });
	}

	bool nextFrame()
	{
		frame_ready = false;

		ErrorNumber_e status;
		if (_settings->image_type == 0) status = _camera->getDistance(*_tofImage);
		if (_settings->image_type == 1) status = _camera->getDistanceGrayscale(*_tofImage);
		if (_settings->image_type == 2) status = _camera->getDistanceAmplitude(*_tofImage);
		if (status != ERROR_NUMMBER_NO_ERROR)
		{
			std::cerr << "Error: " << status << std::endl;
			return false;
		}

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

		pcl::PointXYZRGB* data_ptr = reinterpret_cast<pcl::PointXYZRGB*>(_tofImage->data_3d_xyz_rgb);
		std::vector<pcl::PointXYZRGB> pts(data_ptr, data_ptr + _tofImage->n_points);

		cloud->points.insert(cloud->points.end(), pts.begin(), pts.end());
		cloud->resize(_tofImage->n_points);
		cloud->width = _tofImage->n_points;
		cloud->height = 1;
		cloud->is_dense = false;

		*_cloud = *cloud;

		depth_bgr = cv::Mat(_tofImage->height, _tofImage->width, CV_8UC3, _tofImage->data_2d_bgr);
		grayscale = cv::Mat(_tofImage->height, _tofImage->width, CV_8UC1, _tofImage->data_grayscale);
		amplitude = cv::Mat(_tofImage->height, _tofImage->width, CV_32F,  _tofImage->data_amplitude);

		frame_ready = true;

		return true;
	}
};

std::map<uint32_t, Channel*> channels;

void updateConfig(vsemi_tof_ros::vsemi_tof_rosConfig &config, uint32_t level)
{
	globle_settings.image_type = static_cast<uint>(config.image_type);

	globle_settings.hdr = 0;//static_cast<uint>(config.hdr);

	globle_settings.automaticIntegrationTime = 0;//config.automatic_integration_time;

	globle_settings.integrationTimeATOF0  = static_cast<uint>(config.integration_time_0);
	globle_settings.integrationTimeATOF1  = 0;//static_cast<uint>(config.integration_time_1);
	globle_settings.integrationTimeATOF2  = 0;//static_cast<uint>(config.integration_time_2);
	globle_settings.integrationTimeATOF3  = 0;//static_cast<uint>(config.integration_time_3);
	globle_settings.integrationTimeBTOF4  = 0;//static_cast<uint>(config.integration_time_4);
	globle_settings.integrationTimeBTOF5  = 0;//static_cast<uint>(config.integration_time_5);

	globle_settings.integrationTimeGray   = 0;//static_cast<uint>(config.integration_time_gray);

	globle_settings.minAmplitude0 = static_cast<uint>(config.min_amplitude_0);
	globle_settings.minAmplitude1 = 0;//static_cast<uint>(config.min_amplitude_1);
	globle_settings.minAmplitude2 = 0;//static_cast<uint>(config.min_amplitude_2);
	globle_settings.minAmplitude3 = 0;//static_cast<uint>(config.min_amplitude_3);
	globle_settings.minAmplitude4 = 0;//static_cast<uint>(config.min_amplitude_4);
	globle_settings.minAmplitude5 = 0;//static_cast<uint>(config.min_amplitude_5);

	globle_settings.dcsFilter = 0;//config.dcs_filter;

	globle_settings.range   = 7500;//static_cast<uint>(config.range);

	globle_settings.angle_x = 55.0;//config.angle_x;
	globle_settings.angle_y = 20.625;//config.angle_y;

	globle_settings.updateParam = true;
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

void start() {

	ros::NodeHandle nh("~");

	bool succeed = false;
	int i = 0;
	while(i < 6 && channels.size() < 3)
	{
		std::string port = "/dev/ttyACM" + std::to_string(i);
		Channel* channel = new Channel(port, &globle_settings);
		succeed = channel->open();
		if (succeed)
		{
			uint32_t id = channel->getID();
			std::cout << "Camera connected at port " << port << ": " << id << std::endl;
			channels[id] = channel;

			cloud_publishers[i] = nh.advertise<sensor_msgs::PointCloud2>("cloud_" + std::to_string(i), 1);
		}
		i ++;
	}
	if (channels.size() == 0)
	{
		std::cout << "No camera found." << std::endl;
		running = false;
		return;
	}

	image_depth_Publisher     = nh.advertise<sensor_msgs::Image>("image_depth", 1);
	image_grayscale_Publisher = nh.advertise<sensor_msgs::Image>("image_grayscale", 1);
	image_amplitude_Publisher = nh.advertise<sensor_msgs::Image>("image_amplitude", 1);

	std::chrono::steady_clock::time_point st_time;
	std::chrono::steady_clock::time_point en_time;

	double interval, frame_rate;

	int n_frames = 0;
	std::vector<std::thread> threads;

	st_time = std::chrono::steady_clock::now();
	while (running) {
		threads.clear();
		if (globle_settings.updateParam) 
		{
			if (n_frames > 0)
			{
				en_time = std::chrono::steady_clock::now();
				interval = ((double) std::chrono::duration_cast<std::chrono::microseconds>(en_time - st_time).count()) / 1000000.0;
				frame_rate = ((double) n_frames) / interval;
				std::cout << "Distance frames: " << n_frames << " time spent: " << interval << " frame rate: " << frame_rate << std::endl;
			}

			int modulationChannel = 0;
			for(std::map<uint32_t,Channel*>::iterator it = channels.begin(); it != channels.end(); ++it)
			{
				Channel* channel = it->second;
				channel->update(modulationChannel);
				modulationChannel ++;
			}

			globle_settings.updateParam = false;
			st_time = std::chrono::steady_clock::now();
			n_frames = 0;
		}
		for(std::map<uint32_t,Channel*>::iterator it = channels.begin(); it != channels.end(); ++it)
		{
			Channel* channel = it->second;
			std::thread th = channel->thNextFrame();
			threads.push_back(std::move(th));
		}
		for (size_t i = 0; i < threads.size(); i ++)
		{
			threads[i].join();
		}
		
		cv::Mat depth_bgr;
		cv::Mat grayscale;
		cv::Mat amplitude;
		int chn = 0;
		int chn_central = 0.5 * channels.size();
		ros::Time curTime = ros::Time::now();
		for(std::map<uint32_t,Channel*>::iterator it = channels.begin(); it != channels.end(); ++it)
		{
			uint32_t id = it->first;
			Channel* channel = it->second;
			if (channel->frame_ready)
			{
				if (chn != chn_central)
				{
					int i = chn_central - chn;

					Eigen::Matrix3f rotation_matrix3f;
					rotation_matrix3f = 
						  Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX())
						* Eigen::AngleAxisf(M_PI * i * globle_settings.angle_x / 180.0, Eigen::Vector3f::UnitY())
						* Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ());
					
					Eigen::Affine3f transform_affine3f = Eigen::Affine3f::Identity();
					transform_affine3f.rotate(rotation_matrix3f);
					pcl::transformPointCloud (*channel->_cloud, *channel->_cloud, transform_affine3f);
				}
				publish_cloud(cloud_publishers[chn], channel->_cloud, curTime);

				if (chn == 0)
				{
					depth_bgr = channel->depth_bgr;
					grayscale = channel->grayscale;
					amplitude = channel->amplitude;
				} else{
					cv::hconcat(depth_bgr, channel->depth_bgr, depth_bgr);
					cv::hconcat(grayscale, channel->grayscale, grayscale);
					cv::hconcat(amplitude, channel->amplitude, amplitude);
				}
			} else{
				break;
			}
			chn ++;
		}
		cvtColor(grayscale, grayscale, cv::COLOR_GRAY2BGR);
		amplitude.convertTo(amplitude, CV_8UC1);
		cvtColor(amplitude, amplitude, cv::COLOR_GRAY2BGR);

		publish_image(image_depth_Publisher, depth_bgr, curTime);
		publish_image(image_grayscale_Publisher, grayscale, curTime);
		publish_image(image_amplitude_Publisher, amplitude, curTime);

		n_frames ++;
	}
	en_time = std::chrono::steady_clock::now();
	interval = ((double) std::chrono::duration_cast<std::chrono::microseconds>(en_time - st_time).count()) / 1000000.0;
	frame_rate = ((double) n_frames) / interval;
	std::cout << "Distance frames: " << n_frames << " time spent: " << interval << " frame rate: " << frame_rate << std::endl;

	for(std::map<uint32_t,Channel*>::iterator it = channels.begin(); it != channels.end(); ++it)
	{
		Channel* channel = it->second;
		delete channel;
	}

	running = false;
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

	std::thread th_start(&start);
	th_start.detach();

	ros::spin();

	running = false;
	std::cerr << "Preparing to shutdown ... " << std::endl;
	usleep(2000000);
	std::cerr << "Proceed to shutdown ... " << std::endl;

	ros::shutdown();
}
