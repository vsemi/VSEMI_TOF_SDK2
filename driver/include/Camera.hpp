/**
 * Copyright (C) 2021 Visionary Semiconductor Inc.
 *
 * @defgroup Camera160
 * @brief camera interface
 * @ingroup driver
 *
 * @{
 */
#ifndef TOF_CAMERA_H_
#define TOF_CAMERA_H_

#include <ToFImage.hpp>

#include<string>
#include <memory>

#include "camera_info.h"
#include "error.h"
#include "common.h"

enum OperationMode_e
{
	MODE_BEAM_A = 0,                                        ///<Normal operation with illumination beam A, Wide Field
	MODE_BEAM_B = 1                                         ///<Normal operation with illumination beam B, Narrow Field
};

enum AcquisitionMode_e
{
	SINGLE = 0,
	AUTO_REPEAT = 1,
	STREAM = 2
};

enum HDR_e
{
	HDR_OFF = 0,
	HDR_SPATIAL = 1,
	HDR_TEMPORAL = 2
};

class Camera
{
public:
	static Camera* usb_tof_camera_160(std::string port);

	virtual ~Camera() throw ();

	virtual bool open() = 0;

	virtual void startStream() = 0;
	virtual ErrorNumber_e stopStream() = 0;

	virtual ErrorNumber_e getChipInformation(uint16_t &chipId, uint16_t &waferId) = 0;
	virtual ErrorNumber_e getIdentification(unsigned int &device, unsigned int &version) = 0;
	virtual ErrorNumber_e getFirmwareRelease(unsigned int &major, unsigned int &minor) = 0;

	virtual ErrorNumber_e getTemperature(int16_t &temperature) = 0;

	virtual ErrorNumber_e getDistance(ToFImage &tofImage) = 0;
	virtual ErrorNumber_e getDistanceGrayscale(ToFImage &tofImage) = 0;
	virtual ErrorNumber_e getDistanceAmplitude(ToFImage &tofImage) = 0;

	virtual ErrorNumber_e getLensCalibrationData(CameraInfo &cameraInfo) = 0;

	virtual ErrorNumber_e setOperationMode(OperationMode_e mode) = 0;
	virtual void setAcquisitionMode(AcquisitionMode_e mode) = 0;
	virtual ErrorNumber_e setOffset(const int offset) = 0;
	virtual ErrorNumber_e setIntegrationTime3d(const unsigned int index, const unsigned int integrationTime) = 0;

	virtual ErrorNumber_e setIntegrationTimeGrayscale(const unsigned int integrationTime) = 0;
	virtual ErrorNumber_e setModulationFrequency(const ModulationFrequency_e modulationFrequency) = 0;
	virtual ErrorNumber_e setFilter(const unsigned int threshold, const unsigned int factor) = 0;
	virtual ErrorNumber_e setFilterSpot(const unsigned int threshold, const unsigned int factor) = 0;
	virtual ErrorNumber_e setDcsFilter(const bool enabled) = 0;
	virtual ErrorNumber_e setGaussianFilter(const bool enabled) = 0;
	virtual ErrorNumber_e setCalibrationMode(const bool enabled) = 0;

	virtual ErrorNumber_e setMinimalAmplitude(const unsigned index, const unsigned int amplitude) = 0;
	virtual ErrorNumber_e setBinning(const int binning) = 0;
	virtual ErrorNumber_e setFrameRate(const unsigned int FrameTime) = 0;
	virtual ErrorNumber_e setHdr(HDR_e hdr) = 0;
	virtual ErrorNumber_e setRoi(const unsigned int xMin, const unsigned int yMin, const unsigned int xMax, const unsigned int yMax) = 0;

	virtual ErrorNumber_e setModulationChannel(const bool autoChannelEnabled, const int channel) = 0;

	virtual void setRange(int start, int stop) = 0;

	int getWidth();
	int getHeight();

protected:
	void setWidth(int w, int h);

private:
	int width;
	int height;
};

#endif /* TOF_CAMERA_H_ */

/** @} */
