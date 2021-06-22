/**
 * Copyright (C) 2019 Visionary Semiconductor Inc.
 *
 * @defgroup Error
 * @brief ErrorNumber_e
 * @ingroup driver
 *
 * @{
 */
#ifndef INCLUDE_ERROR_H_
#define INCLUDE_ERROR_H_

enum ErrorNumber_e
{
	ERROR_NUMMBER_NO_ERROR = 0,
	ERROR_NUMBER_TIMEOUT = 32768,
	ERROR_NUMBER_NOT_ACKNOWLEDGE = 32769,
	ERROR_NUMBER_INVALID_PARAMETER = 32770,
	ERROR_NUMBER_SERIAL_PORT_ERROR = 32771,
	ERROR_NUMBER_INVALID_DATA = 32772
};

#endif /* INCLUDE_ERROR_H_ */
