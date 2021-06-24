/**
 * Copyright (C) 2019 Visionary Semiconductor Inc.
 *
 * @defgroup settings Settings
 * @brief Settings communicating between the driver and ToF sensor
 * @ingroup driver
 *
 * @{
 */
#ifndef VSEMI_TOF_DRIVER_SETTINGS_H
#define VSEMI_TOF_DRIVER_SETTINGS_H

#include <stdint.h>
#include <string>

//! Settings
/*!
 * This struct defines communicating parameters between the driver and ToF sensor.
 * Applications could use this to design control panel UI, for controlling ToF sensor at runtime.
 */
struct Settings{

	bool updateParam;  /*! true to start ToF sensor */

	uint image_type;   /*! 0 for distance only, and 1 for distance and grayscale, 2 for distance and amplitude */

	uint  hdr;         /*! 0 HDR off, 1 for HDR spatial and 2 for HDR temporal */

	bool automaticIntegrationTime; /*! true for auto integration time, false to turn off auto integration time */

	uint integrationTimeATOF0;     /*! 0 - 1000, if automaticIntegrationTime set to false and mode set to 0, use this to set integration time for wide FOV */
	uint integrationTimeATOF1;     /*! 0 - 1000, if automaticIntegrationTime set to false and mode set to 0 and hdr is set to 2, use this to set integration time for HDR range */
	uint integrationTimeATOF2;     /*! 0 - 1000, if automaticIntegrationTime set to false and mode set to 0 and hdr is set to 2, use this to set integration time for HDR range */
	uint integrationTimeATOF3;     /*! 0 - 1000, if automaticIntegrationTime set to false and mode set to 0 and hdr is set to 2, use this to set integration time for HDR range */
	uint integrationTimeBTOF4;     /*! 0 - 1000, if automaticIntegrationTime set to false and mode set to 1, use this to set integration time for narrow beam */
	uint integrationTimeBTOF5;     /*! 0 - 1000, if automaticIntegrationTime set to false and mode set to 1, use this to set integration time for narrow beam */

	uint integrationTimeGray;      /*! 0 - 50000, integration time for grayscale */

	uint minAmplitude0;            /*! 0 - 2047, threshold minAmplitude 0 beam A LSB */
	uint minAmplitude1;            /*! 0 - 2047, threshold minAmplitude 1 beam A LSB */
	uint minAmplitude2;            /*! 0 - 2047, threshold minAmplitude 2 beam A LSB */
	uint minAmplitude3;            /*! 0 - 2047, threshold minAmplitude 3 beam A LSB */
	uint minAmplitude4;            /*! 0 - 2047, threshold minAmplitude 4 beam B LSB */
	uint minAmplitude5;            /*! 0 - 2047, threshold minAmplitude 4 beam B LSB */

	bool dcsFilter;                /*! true to enable DCS spatial filter */

	int range;                  /*! 0 - 9000, the depth range in mm, for coloring the BGR depth map and the 3D points */

	double angle_x      = 55.0f;  /*! horizontal angle of the FOV, modify this to calibrate horizontal angle of the FOV */
	double angle_y      = 20.625f; /*! vertical angle of the FOV, modify this to calibrate vertical angle of the FOV */
};

#endif // VSEMI_TOF_DRIVER_SETTINGS_H

/** @} */
