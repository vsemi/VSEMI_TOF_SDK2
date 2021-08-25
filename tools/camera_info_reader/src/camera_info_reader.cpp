#include <iostream>
#include <time.h>

#include <Camera.hpp>

void read_camera_info(Camera* camera)
{
	ErrorNumber_e status;

	uint16_t chipId, waferId;
	status = camera->getChipInformation(chipId, waferId);
	if (status == ERROR_NUMMBER_NO_ERROR) std::cout << "\nID:" << waferId << "-" << chipId << "\n   Wafer:   " << waferId << "\n   Chip:    " << chipId << std::endl;
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

void start()
{
	Camera* camera = Camera::usb_tof_camera_160("/dev/ttyACM0");
	bool success = camera->open();

	if (! success)
	{
		std::cerr << "Failed opening Camera!" << std::endl;
		return;
	}

	read_camera_info(camera);

	delete camera;
}

int main() {

	std::cout << "Camera is starting ..." << std::endl;

	start();

	return 0;
}
