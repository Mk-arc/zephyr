/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file
 * @brief Quectel LC79H Host EPO assistance extensions.
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_GNSS_QUECTEL_LC79H_H_
#define ZEPHYR_INCLUDE_DRIVERS_GNSS_QUECTEL_LC79H_H_

#include <stdint.h>
#include <zephyr/drivers/gnss.h>

#ifdef __cplusplus
extern "C" {
#endif

struct device;

/**
 * @defgroup gnss_quectel_lc79h Quectel LC79H assistance
 * @ingroup gnss_interface
 * @since 4.5
 * @version 0.1.0
 * @{
 *
 * These synchronous extensions require a ready LC79H device and thread context.
 * Do not call them from GNSS receive callbacks. NMEA reception continues while
 * waiting for PAIR replies. Provider access and assistance freshness are the
 * application's responsibility.
 *
 * PAIR replies have no transaction identifier. After a transport error or timeout,
 * restart the receiver and suspend/resume the driver before submitting more data.
 * Board power and reset control are external to this driver.
 */

/** Reference position and circular horizontal uncertainty in WGS84. */
struct gnss_quectel_lc79h_reference_position {
	int32_t latitude_udeg;          /**< Latitude in microdegrees, -90000000 to 90000000. */
	int32_t longitude_udeg;         /**< Longitude in microdegrees, -180000000 to 180000000. */
	int32_t altitude_m;             /**< Ellipsoidal altitude in metres, not mean sea level. */
	uint32_t horizontal_accuracy_m; /**< Horizontal uncertainty radius, 1 to 30000 metres. */
	uint32_t vertical_accuracy_m;   /**< Vertical uncertainty, 1 to 100000 metres. */
};

/**
 * @brief Inject reference UTC using PAIR590.
 * @param dev Ready LC79H device.
 * @param unix_seconds Current UTC seconds since 1970, within years 1981 through 2099.
 *        The application must provide time accurate to within three seconds.
 * @retval 0 Receiver acknowledged the reference time.
 * @retval -EINVAL Time is outside the supported calendar range.
 * @retval -EINVAL Receiver rejected the command parameters.
 * @retval -EHOSTDOWN Driver is suspended.
 * @retval -EPIPE Command channel needs receiver restart after a transport failure.
 * @retval -ETIMEDOUT Command transmission or acknowledgment timed out.
 * @retval -ENOTSUP Receiver does not support the command.
 * @retval -EBUSY Receiver is busy.
 * @retval -EIO Receiver rejected the command.
 * @return A negative transport error from the UART pipe on transmission failure.
 */
int gnss_quectel_lc79h_set_reference_time(const struct device *dev, int64_t unix_seconds);

/**
 * @brief Inject a reference position using PAIR600.
 * @param dev Ready LC79H device.
 * @param position Position and uncertainty; must remain valid until return. NULL is rejected.
 * @retval 0 Receiver acknowledged the position.
 * @retval -EINVAL Position or uncertainty is invalid.
 * @retval -EINVAL Receiver rejected the command parameters.
 * @retval -EHOSTDOWN Driver is suspended.
 * @retval -EPIPE Command channel needs receiver restart after a transport failure.
 * @retval -ETIMEDOUT Command transmission or acknowledgment timed out.
 * @retval -ENOTSUP Receiver does not support the command.
 * @retval -EBUSY Receiver is busy.
 * @retval -EIO Receiver rejected the command.
 * @return A negative transport error from the UART pipe on transmission failure.
 */
int gnss_quectel_lc79h_set_reference_position(
	const struct device *dev, const struct gnss_quectel_lc79h_reference_position *position);

/**
 * @brief Inject one GPS or GLONASS EPO record using PAIR471.
 *
 * Wait for both the successful PAIR001 acknowledgment and matching PAIR471 echo.
 * Receiver firmware may echo the system/satellite pair or the first EPO word;
 * both forms are checked against the submitted record.
 * The application must validate the complete EPO block and its validity interval
 * before injecting records. This call does not validate the opaque EPO checksum.
 *
 * @param dev Ready LC79H device.
 * @param record One 72-byte record with 18 little-endian words. Byte 3 contains
 *        the EPO satellite identifier: 1 to 32 for GPS or 65 to 88 for GLONASS.
 *        The buffer must remain valid until return. NULL is rejected.
 * @retval 0 Receiver acknowledged the record and returned a matching echo.
 * @retval -EINVAL Record is NULL.
 * @retval -EINVAL Receiver rejected the command parameters.
 * @retval -ENOTSUP Record constellation or satellite is unsupported.
 * @retval -ENOTSUP Receiver does not support the command.
 * @retval -EHOSTDOWN Driver is suspended.
 * @retval -EPIPE Command channel needs receiver restart after a transport failure.
 * @retval -ETIMEDOUT Command transmission or either response timed out.
 * @retval -EBUSY Receiver is busy.
 * @retval -EIO Receiver rejected the command.
 * @return A negative transport error from the UART pipe on transmission failure.
 */
int gnss_quectel_lc79h_set_epo_record(const struct device *dev, const uint8_t record[72]);

/** Raw PAIR470 response; week/time-of-week fields use GPS time, not UTC. */
struct gnss_quectel_lc79h_epo_status {
	uint32_t sets;             /**< Receiver-reported number of EPO sets. */
	uint16_t flash_start_week; /**< First flash set GPS week. */
	uint32_t flash_start_tow;  /**< First flash set GPS seconds of week. */
	uint16_t flash_end_week;   /**< Last flash set GPS week. */
	uint32_t flash_end_tow;    /**< Last flash set GPS seconds of week. */
	uint16_t used_start_week;  /**< First currently used set GPS week. */
	uint32_t used_start_tow;   /**< First currently used set GPS seconds of week. */
	uint16_t used_end_week;    /**< Last currently used set GPS week. */
	uint32_t used_end_tow;     /**< Last currently used set GPS seconds of week. */
};

/**
 * @brief Query receiver EPO status using PAIR470.
 *
 * Reported windows do not prove that a Host EPO upload completed. Applications
 * must track acknowledged uploads independently. Zero windows indicate no interval.
 *
 * @param dev Ready LC79H device.
 * @param system Constellation: 0 GPS, 1 GLONASS, 2 Galileo, 3 BeiDou.
 * @param status Output, updated only on success. NULL is rejected.
 * @retval 0 Receiver acknowledged the query and returned matching status.
 * @retval -EINVAL Invalid system or NULL output.
 * @retval -EINVAL Receiver rejected the command parameters.
 * @retval -EHOSTDOWN Driver is suspended.
 * @retval -EPIPE Command channel needs receiver restart after a transport failure.
 * @retval -ETIMEDOUT Command transmission or response timed out.
 * @retval -ENOTSUP Receiver does not support the query.
 * @retval -EBUSY Receiver is busy.
 * @retval -EIO Receiver rejected the query.
 * @retval -ENOSPC Query sentence could not be encoded.
 * @return A negative UART transport error on transmission failure.
 */
int gnss_quectel_lc79h_get_epo_status(const struct device *dev, uint8_t system,
				      struct gnss_quectel_lc79h_epo_status *status);

/**
 * @brief Get the assistance request sequence.
 * @param dev Ready LC79H device.
 * @param epo_systems Systems whose EPO requests should be counted. Use zero for
 *                   only time/position requests. Keep this mask fixed when comparing.
 * @return Counter incremented on resume, time/position requests and EPO requests
 *         for selected systems. Compare for inequality; the counter wraps at 32 bits.
 */
uint32_t gnss_quectel_lc79h_aiding_sequence(const struct device *dev, gnss_systems_t epo_systems);

/** @} */
#ifdef __cplusplus
}
#endif
#endif /* ZEPHYR_INCLUDE_DRIVERS_GNSS_QUECTEL_LC79H_H_ */
