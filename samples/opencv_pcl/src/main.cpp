#include <iostream>
#include <time.h>
#include <thread>
#include <chrono>

#include <opencv2/highgui.hpp>

#include <pcl/visualization/pcl_visualizer.h>

#include <Camera.hpp>

bool update_frame_data = false;
bool is_stopped = false;

ToFImage* tofImage;
cv::Mat *depth_bgr = new cv::Mat(60, 160, CV_8UC3, cv::Scalar(0, 0, 0));
pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

void points_cloud_visualize() {
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Vsemi TOF Camera"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> colorHandler(point_cloud_ptr);
	viewer->addPointCloud<pcl::PointXYZRGB>(point_cloud_ptr, colorHandler, "point_cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "point_cloud");
	viewer->initCameraParameters();

	viewer->setCameraPosition(0, 0, -0.5,    0, 0, 0,   0, 0, 0);

	while (!viewer->wasStopped() && !is_stopped)
	{
		if (update_frame_data) {

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
			*cloud = *point_cloud_ptr;

			viewer->removeAllShapes();
			viewer->removeAllPointClouds();
			pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> colorHandler(point_cloud_ptr);
			viewer->addPointCloud<pcl::PointXYZRGB>(point_cloud_ptr, colorHandler, "point_cloud");

			viewer->spinOnce(1);

			cv::imshow("Depth Map BGR", *depth_bgr);

			if (cv::waitKey(1) == 27)
			{
				break;
			}

			update_frame_data = false;
		} else {
			usleep(10000);
		}
	}
	is_stopped = true;
}

int main() {

	cout << "Starting ..." << endl;

	Camera* camera = Camera::usb_tof_camera_160("/dev/ttyACM0");
	bool success = camera->open();

	if (! success)
	{
		std::cerr << "Failed opening Camera!" << std::endl;
		return 1;
	}

	ErrorNumber_e status;

	status = camera->setOperationMode(MODE_BEAM_A);
	if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set Mode failed." << endl;

	status = camera->setModulationFrequency(ModulationFrequency_e::MODULATION_FREQUENCY_20MHZ);
	if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set ModulationFrequency failed." << endl;

	status = camera->setModulationChannel(0, 1);
	if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set tModulationChannel failed." << endl;

	camera->setAcquisitionMode(AUTO_REPEAT);
	if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set AcquisitionMode failed." << endl;

	camera->setOffset(0);
	if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set Offset failed." << endl;

	unsigned int integrationTime0 = 1000;
	unsigned int integrationTime1 = 50;

	status = camera->setIntegrationTime3d(0, integrationTime0);
	if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set IntegrationTime3d 0 failed." << endl;
	status = camera->setIntegrationTime3d(1, integrationTime1);
	if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set IntegrationTime3d 1 failed." << endl;
	status = camera->setIntegrationTime3d(2, 0);
	if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set IntegrationTime3d 2 failed." << endl;
	status = camera->setIntegrationTime3d(3, 0);
	if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set IntegrationTime3d 3 failed." << endl;
	status = camera->setIntegrationTime3d(4, 0);
	if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set IntegrationTime3d 4 failed." << endl;
	status = camera->setIntegrationTime3d(5, 0);
	if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set IntegrationTime3d 5 failed." << endl;

	unsigned int amplitude0 = 60;
	unsigned int amplitude1 = 60;

	status = camera->setMinimalAmplitude(0, amplitude0);
	if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set MinimalAmplitude 0 failed." << endl;
	status = camera->setMinimalAmplitude(1, amplitude1);
	if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set MinimalAmplitude 1 failed." << endl;
	status = camera->setMinimalAmplitude(2, 0);
	if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set MinimalAmplitude 2 failed." << endl;
	status = camera->setMinimalAmplitude(3, 0);
	if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set MinimalAmplitude 3 failed." << endl;
	status = camera->setMinimalAmplitude(4, 0);
	if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set MinimalAmplitude 4 failed." << endl;
	status = camera->setMinimalAmplitude(5, 0);
	if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set MinimalAmplitude 5 failed." << endl;

	std::cout << "\nTest 3D frame acquisition: " << std::endl;

	std::cout << "integrationTime0: " << integrationTime0 << std::endl;
	std::cout << "integrationTime1: " << integrationTime1 << std::endl;

	std::cout << "amplitude0: " << amplitude0 << std::endl;
	std::cout << "amplitude1: " << amplitude1 << std::endl;

	camera->setIntegrationTimeGrayscale(0);

	camera->setRange(50, 7500);

	HDR_e hdr = HDR_OFF; //HDR_SPATIAL; //HDR_TEMPORAL; // HDR_OFF;//
	status = camera->setHdr(hdr);
	if (status != ERROR_NUMMBER_NO_ERROR) cerr << "Set HDR failed." << endl;
	std::cout << "\nHDR: " << hdr << std::endl;
	std::cout << "=======" << std::endl;

	tofImage = new ToFImage(camera->getWidth(), camera->getHeight());

	std::thread th_points_cloud_visualize(&points_cloud_visualize);
	th_points_cloud_visualize.detach();

	std::chrono::steady_clock::time_point st_time;
	std::chrono::steady_clock::time_point en_time;
	double interval, frame_rate;
	int n_frames = 0;
	st_time = std::chrono::steady_clock::now();

	while (!is_stopped)
	{
		status = camera->getDistance(*tofImage);
		if (status != ERROR_NUMMBER_NO_ERROR)
		{
			is_stopped = true;
			std::cerr << "Error: " << status << std::endl;
			break;
		}

		*depth_bgr = cv::Mat(tofImage->height, tofImage->width, CV_8UC3, tofImage->data_2d_bgr);

		pcl::PointXYZRGB* data_ptr = reinterpret_cast<pcl::PointXYZRGB*>(tofImage->data_3d_xyz_rgb);
		std::vector<pcl::PointXYZRGB> pts(data_ptr, data_ptr + tofImage->n_points);
		point_cloud_ptr->points.clear();
		point_cloud_ptr->points.insert(point_cloud_ptr->points.end(), pts.begin(), pts.end());
		point_cloud_ptr->resize(tofImage->n_points);
		point_cloud_ptr->width = tofImage->n_points;
		point_cloud_ptr->height = 1;
		point_cloud_ptr->is_dense = false;

		update_frame_data = true;

		n_frames ++;
	}
	en_time = std::chrono::steady_clock::now();
	interval = ((double) std::chrono::duration_cast<std::chrono::microseconds>(en_time - st_time).count()) / 1000000.0;
	frame_rate = ((double) n_frames) / interval;
	std::cout << "Distance frames: " << n_frames << " time spent: " << interval << " frame rate: " << frame_rate << std::endl;

	delete tofImage;
	delete camera;

	return 0;
}
