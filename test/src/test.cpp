#include <iostream>
#include <time.h>

#include <Camera.hpp>

void test_general_info(Camera* camera)
{
	ErrorNumber_e status;

	uint16_t chipId, waferId;
	status = camera->getChipInformation(chipId, waferId);
	if (status == ERROR_NUMMBER_NO_ERROR) std::cout << "Chip:\n   wafer:   " << waferId << "\n   ID:      " << chipId << std::endl;
	else std::cerr << "Error: " << status << std::endl;

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
}

void test_distance(Camera* camera, int n_frames)
{
	ErrorNumber_e status;

	ToFImage tofImage(camera->getWidth(), camera->getHeight());
	camera->setIntegrationTimeGrayscale(0);

	clock_t start, stop;
	start = clock();
	for (int i = 0; i < n_frames; i ++)
	{
		status = camera->getDistance(tofImage);
		if (status != ERROR_NUMMBER_NO_ERROR)
		{
			std::cerr << "Error: " << status << std::endl;
			break;
		}
	}
	stop = clock();
	double interval = (double)(stop - start) / CLOCKS_PER_SEC;
	double frame_rate = ((double) n_frames) / interval;
	std::cout << "Distance frames: " << n_frames << " time spent: " << interval << " frame rate: " << frame_rate << std::endl;
}

void test_distance_grayscale(Camera* camera, int n_frames)
{
	ErrorNumber_e status;

	ToFImage tofImage(camera->getWidth(), camera->getHeight());
	camera->setIntegrationTimeGrayscale(6000);

	clock_t start, stop;
	start = clock();
	for (int i = 0; i < n_frames; i ++)
	{
		status = camera->getDistanceGrayscale(tofImage);
		if (status != ERROR_NUMMBER_NO_ERROR)
		{
			std::cerr << "Error: " << status << std::endl;
			break;
		}
	}
	stop = clock();
	double interval = (double)(stop - start) / CLOCKS_PER_SEC;
	double frame_rate = ((double) n_frames) / interval;
	std::cout << "Distance grayscale frames: " << n_frames << " time spent: " << interval << " frame rate: " << frame_rate << std::endl;
}

void test_distance_amplitude(Camera* camera, int n_frames)
{
	ErrorNumber_e status;

	ToFImage tofImage(camera->getWidth(), camera->getHeight());
	camera->setIntegrationTimeGrayscale(0);

	clock_t start, stop;
	start = clock();
	for (int i = 0; i < n_frames; i ++)
	{
		status = camera->getDistanceAmplitude(tofImage);
		if (status != ERROR_NUMMBER_NO_ERROR)
		{
			std::cerr << "Error: " << status << std::endl;
			break;
		}
	}
	stop = clock();
	double interval = (double)(stop - start) / CLOCKS_PER_SEC;
	double frame_rate = ((double) n_frames) / interval;
	std::cout << "Distance amplitude frames: " << n_frames << " time spent: " << interval << " frame rate: " << frame_rate << std::endl;
}

void test()
{
	Camera* camera = Camera::usb_tof_camera_160("/dev/ttyACM0");
	bool success = camera->open();

	if (! success)
	{
		std::cerr << "Failed opening Camera!" << std::endl;
		return;
	}

	test_general_info(camera);

	ErrorNumber_e status;

	status = camera->setOperationMode(MODE_BEAM_A);
	if (status != ERROR_NUMMBER_NO_ERROR) std::cerr << "Set OperationMode failed." << std::endl;

	status = camera->setOffset(0);
	if (status != ERROR_NUMMBER_NO_ERROR) std::cerr << "Set Offset failed." << std::endl;

	unsigned int integrationTime0 = 1000;
	unsigned int integrationTime1 = 50;

	status = camera->setIntegrationTime3d(0, integrationTime0);
	if (status != ERROR_NUMMBER_NO_ERROR) std::cerr << "Set IntegrationTime3d 0 failed." << std::endl;
	status = camera->setIntegrationTime3d(1, integrationTime1);
	if (status != ERROR_NUMMBER_NO_ERROR) std::cerr << "Set IntegrationTime3d 1 failed." << std::endl;
	status = camera->setIntegrationTime3d(2, 0);
	if (status != ERROR_NUMMBER_NO_ERROR) std::cerr << "Set IntegrationTime3d 2 failed." << std::endl;
	status = camera->setIntegrationTime3d(3, 0);
	if (status != ERROR_NUMMBER_NO_ERROR) std::cerr << "Set IntegrationTime3d 3 failed." << std::endl;
	status = camera->setIntegrationTime3d(4, 0);
	if (status != ERROR_NUMMBER_NO_ERROR) std::cerr << "Set IntegrationTime3d 4 failed." << std::endl;
	status = camera->setIntegrationTime3d(5, 0);
	if (status != ERROR_NUMMBER_NO_ERROR) std::cerr << "Set IntegrationTime3d 5 failed." << std::endl;

	unsigned int amplitude0 = 60;
	unsigned int amplitude1 = 60;

	status = camera->setMinimalAmplitude(0, amplitude0);
	if (status != ERROR_NUMMBER_NO_ERROR) std::cerr << "Set MinimalAmplitude 0 failed." << std::endl;
	status = camera->setMinimalAmplitude(1, amplitude1);
	if (status != ERROR_NUMMBER_NO_ERROR) std::cerr << "Set MinimalAmplitude 1 failed." << std::endl;
	status = camera->setMinimalAmplitude(2, 0);
	if (status != ERROR_NUMMBER_NO_ERROR) std::cerr << "Set MinimalAmplitude 2 failed." << std::endl;
	status = camera->setMinimalAmplitude(3, 0);
	if (status != ERROR_NUMMBER_NO_ERROR) std::cerr << "Set MinimalAmplitude 3 failed." << std::endl;
	status = camera->setMinimalAmplitude(4, 0);
	if (status != ERROR_NUMMBER_NO_ERROR) std::cerr << "Set MinimalAmplitude 4 failed." << std::endl;
	status = camera->setMinimalAmplitude(5, 0);
	if (status != ERROR_NUMMBER_NO_ERROR) std::cerr << "Set MinimalAmplitude 5 failed." << std::endl;

	std::cout << "\nTest 3D frame acquisition: " << std::endl;

	std::cout << "integrationTime0: " << integrationTime0 << std::endl;
	std::cout << "integrationTime1: " << integrationTime1 << std::endl;

	std::cout << "amplitude0: " << amplitude0 << std::endl;
	std::cout << "amplitude1: " << amplitude1 << std::endl;

	camera->setRange(50, 7500);

	camera->setAcquisitionMode(AUTO_REPEAT);
	HDR_e hdr;

	hdr = HDR_OFF;
	status = camera->setHdr(hdr);
	if (status != ERROR_NUMMBER_NO_ERROR) std::cerr << "Set HDR failed." << std::endl;
	std::cout << "\nHDR: " << hdr << std::endl;
	std::cout << "=======" << std::endl;
	test_distance(camera, 300);
	test_distance_grayscale(camera, 300);
	test_distance_amplitude(camera, 300);

	hdr = HDR_TEMPORAL;
	status = camera->setHdr(hdr);
	if (status != ERROR_NUMMBER_NO_ERROR) std::cerr << "Set HDR failed." << std::endl;
	std::cout << "\nHDR: " << hdr << std::endl;
	std::cout << "=======" << std::endl;
	test_distance(camera, 300);
	test_distance_grayscale(camera, 300);
	test_distance_amplitude(camera, 300);

	delete camera;
}

int main() {

	std::cout << "Test starting ..." << std::endl;

	test();

	return 0;
}
