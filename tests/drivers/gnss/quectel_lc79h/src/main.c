/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <string.h>
#include <zephyr/drivers/gnss.h>
#include <zephyr/drivers/gnss/quectel_lc79h.h>
#include <zephyr/drivers/serial/uart_emul.h>
#include <zephyr/pm/device.h>
#include <zephyr/ztest.h>
#include "gnss_quectel_lc79h_protocol.h"

static const struct device *const uart = DEVICE_DT_GET(DT_NODELABEL(test_uart));
static const struct device *const gnss = DEVICE_DT_GET(DT_NODELABEL(gnss));
static char transmitted[LC79H_SENTENCE_SIZE];
static size_t used;
static uint32_t commands;
static int response_mode;
static uint32_t fixes;

static void rx(const char *body)
{
	char line[256];
	uint32_t checksum = 0;

	for (const char *p = body; *p != '\0'; ++p) {
		checksum ^= (unsigned char)*p;
	}
	int n = snprintf(line, sizeof(line), "$%s*%02X\r\n", body, checksum);

	zassert_equal(uart_emul_put_rx_data(uart, (const uint8_t *)line, n), n);
}

static void fix(const struct device *dev, const struct gnss_data *data)
{
	if (data->info.fix_status != GNSS_FIX_STATUS_NO_FIX) {
		++fixes;
	}
}
GNSS_DATA_CALLBACK_DEFINE(gnss, fix);

static void tx_ready(const struct device *dev, size_t size, void *user)
{
	uint8_t byte;

	while (uart_emul_get_tx_data(dev, &byte, 1) > 0) {
		zassert_true(used < sizeof(transmitted) - 1);
		transmitted[used++] = byte;
		if (byte != '\n') {
			continue;
		}
		transmitted[used] = 0;
		++commands;
		uint32_t command;

		int fields = sscanf(transmitted, "$PAIR%u", &command);

		zassert_equal(fields, 1);
		if (response_mode == 1) {
			used = 0;
			continue;
		} /* Silence. */
		if (response_mode == 2) {
			rx("PAIR001,590,3");
			used = 0;
			continue;
		}
		if (response_mode == 3) {
			rx("PAIR001,590,1");
			used = 0;
			continue;
		}
		if (response_mode == 4) {
			rx("PAIR001,600,0");
			used = 0;
			continue;
		}
		if (response_mode == 5) {
			static const char bad[] = "$PAIR001,590,0*00\r\n";

			uart_emul_put_rx_data(dev, (const uint8_t *)bad, sizeof(bad) - 1);
			used = 0;
			continue;
		}
		if (command == 67 || command == 470) {
			char answer[128];
			uint32_t system = 0;

			if (command == 470) {
				fields = sscanf(transmitted, "$PAIR470,%u", &system);
				zassert_equal(fields, 1);
			}
			snprintf(answer, sizeof(answer), "PAIR001,%03u,%u", command,
				 response_mode == 16 ? 3U : 0U);
			if (response_mode == 12 || response_mode == 16) {
				rx(answer);
			}
			if (response_mode != 14 && response_mode != 16) {
				if (command == 67) {
					rx(response_mode == 15 ? "PAIR067,2,1,1,1,1,0"
							       : "PAIR067,1,1,1,1,2,0");
				} else {
					char status[128];

					snprintf(status, sizeof(status),
						 "PAIR470,%u,1,2098,194400,2098,216000,2098,194400,"
						 "2098,216000",
						 response_mode == 13 ? system ^ 1U : system);
					rx(status);
				}
			}
			if (response_mode != 12 && response_mode != 16) {
				rx(answer);
			}
			used = 0;
			continue;
		}
		char body[48];

		if (command == 471 && response_mode >= 8) {
			if (response_mode == 9) {
				rx("PAIR001,471,0");
			}
			/* Exact word-echo format observed on LC79H hardware. */
			if (response_mode != 11) {
				rx(response_mode == 10 ? "PAIR471,56056273" : "PAIR471,56056272");
			}
			if (response_mode == 11) {
				rx("PAIR471,56056272");
				used = 0;
				continue;
			}
		} else if (command == 471 && response_mode != 6) {
			uint32_t system, prn;

			fields = sscanf(transmitted, "$PAIR471,%u,%X", &system, &prn);
			zassert_equal(fields, 2);
			snprintf(body, sizeof(body), "PAIR471,%u,%X", system,
				 response_mode == 7 ? prn + 1 : prn);
			rx(body); /* Echo before ACK is valid too. */
		}
		snprintf(body, sizeof(body), "PAIR001,%u,1", command);
		rx(body);
		/* NMEA remains live while a PAIR command is outstanding. */
		rx("GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
		rx("GPRMC,123519,A,4807.038,N,01131.000,E,0.0,0.0,230394,,,A");
		snprintf(body, sizeof(body), "PAIR001,%u,0", command);
		rx(body);
		used = 0;
	}
}

static void before(void *fixture)
{
	int ret;

	zassert_true(device_is_ready(uart));
	zassert_true(device_is_ready(gnss));
	ret = pm_device_action_run(gnss, PM_DEVICE_ACTION_SUSPEND);
	zassert_true(ret == 0 || ret == -EALREADY);
	response_mode = 0;
	used = commands = fixes = 0;
	uart_emul_flush_rx_data(uart);
	uart_emul_callback_tx_data_ready_set(uart, tx_ready, NULL);
	zassert_ok(pm_device_action_run(gnss, PM_DEVICE_ACTION_RESUME));
}

ZTEST(lc79h, test_time_position_epo_and_nmea)
{
	zassert_ok(gnss_quectel_lc79h_set_reference_time(gnss, 1549789258));
	zassert_true(fixes > 0);
	struct gnss_quectel_lc79h_reference_position position = {24772816, 121022636, 175, 50, 100};

	zassert_ok(gnss_quectel_lc79h_set_reference_position(gnss, &position));
	uint8_t record[72] = {0};

	record[3] = 86;
	zassert_ok(gnss_quectel_lc79h_set_epo_record(gnss, record));
	zassert_equal(commands, 3);
}

ZTEST(lc79h, test_timeout_poison_and_resume)
{
	response_mode = 1;
	zassert_equal(gnss_quectel_lc79h_set_reference_time(gnss, 1549789258), -ETIMEDOUT);
	rx("PAIR001,590,0"); /* Late acknowledgment must not revive the channel. */
	response_mode = 0;
	zassert_equal(gnss_quectel_lc79h_set_reference_time(gnss, 1549789258), -EPIPE);
	zassert_equal(commands, 1);
	zassert_ok(pm_device_action_run(gnss, PM_DEVICE_ACTION_SUSPEND));
	zassert_equal(gnss_quectel_lc79h_set_reference_time(gnss, 1549789258), -EPIPE);
	zassert_ok(pm_device_action_run(gnss, PM_DEVICE_ACTION_RESUME));
	zassert_ok(gnss_quectel_lc79h_set_reference_time(gnss, 1549789258));
}

ZTEST(lc79h, test_nack)
{
	response_mode = 2;
	zassert_equal(gnss_quectel_lc79h_set_reference_time(gnss, 1549789258), -ENOTSUP);
}

ZTEST(lc79h, test_processing_is_not_success)
{
	response_mode = 3;
	zassert_equal(gnss_quectel_lc79h_set_reference_time(gnss, 1549789258), -ETIMEDOUT);
}

ZTEST(lc79h, test_wrong_command)
{
	response_mode = 4;
	zassert_equal(gnss_quectel_lc79h_set_reference_time(gnss, 1549789258), -ETIMEDOUT);
}

ZTEST(lc79h, test_bad_checksum)
{
	response_mode = 5;
	zassert_equal(gnss_quectel_lc79h_set_reference_time(gnss, 1549789258), -ETIMEDOUT);
}

ZTEST(lc79h, test_epo_needs_satellite_echo)
{
	uint8_t record[72] = {0};

	record[3] = 1;
	response_mode = 6;
	zassert_equal(gnss_quectel_lc79h_set_epo_record(gnss, record), -ETIMEDOUT);
}

ZTEST(lc79h, test_epo_wrong_satellite)
{
	uint8_t record[72] = {0};

	record[3] = 1;
	response_mode = 7;
	zassert_equal(gnss_quectel_lc79h_set_epo_record(gnss, record), -ETIMEDOUT);
}

static const uint8_t word_echo_record[72] = {0x72, 0x62, 0x05, 0x56};

ZTEST(lc79h, test_epo_word_echo_before_ack)
{
	response_mode = 8;
	zassert_ok(gnss_quectel_lc79h_set_epo_record(gnss, word_echo_record));
}

ZTEST(lc79h, test_epo_word_echo_after_ack)
{
	response_mode = 9;
	zassert_ok(gnss_quectel_lc79h_set_epo_record(gnss, word_echo_record));
}

ZTEST(lc79h, test_epo_wrong_word_echo)
{
	response_mode = 10;
	zassert_equal(gnss_quectel_lc79h_set_epo_record(gnss, word_echo_record), -ETIMEDOUT);
}

ZTEST(lc79h, test_epo_word_echo_needs_ack)
{
	response_mode = 11;
	zassert_equal(gnss_quectel_lc79h_set_epo_record(gnss, word_echo_record), -ETIMEDOUT);
}

ZTEST(lc79h, test_aiding_request)
{
	uint32_t sequence = gnss_quectel_lc79h_aiding_sequence(gnss, 0);

	rx("PAIR010,1,-1");
	k_sleep(K_MSEC(100));
	zassert_equal(gnss_quectel_lc79h_aiding_sequence(gnss, 0), sequence + 1);
}

ZTEST(lc79h, test_aiding_requests_filter_constellations)
{
	gnss_systems_t systems = GNSS_SYSTEM_GPS | GNSS_SYSTEM_GLONASS;
	uint32_t initial = gnss_quectel_lc79h_aiding_sequence(gnss, systems);
	uint32_t time_only = gnss_quectel_lc79h_aiding_sequence(gnss, 0);
	uint32_t gps_only = gnss_quectel_lc79h_aiding_sequence(gnss, GNSS_SYSTEM_GPS);

	rx("PAIR010,0,2,2442,469670");
	rx("PAIR010,0,3,2442,469670");
	rx("PAIR010,0,4,2442,469670");
	k_sleep(K_MSEC(100));
	zassert_equal(gnss_quectel_lc79h_aiding_sequence(gnss, systems), initial);
	rx("PAIR010,0,1,2442,469670");
	k_sleep(K_MSEC(100));
	zassert_equal(gnss_quectel_lc79h_aiding_sequence(gnss, systems), initial + 1);
	zassert_equal(gnss_quectel_lc79h_aiding_sequence(gnss, GNSS_SYSTEM_GPS), gps_only);
	rx("PAIR010,0,0,2442,469670");
	k_sleep(K_MSEC(100));
	zassert_equal(gnss_quectel_lc79h_aiding_sequence(gnss, systems), initial + 2);
	zassert_equal(gnss_quectel_lc79h_aiding_sequence(gnss, GNSS_SYSTEM_GPS), gps_only + 1);
	zassert_equal(gnss_quectel_lc79h_aiding_sequence(gnss, 0), time_only);
	rx("PAIR010,2,-1");
	k_sleep(K_MSEC(100));
	zassert_equal(gnss_quectel_lc79h_aiding_sequence(gnss, systems), initial + 3);
	zassert_equal(gnss_quectel_lc79h_aiding_sequence(gnss, 0), time_only + 1);
}

ZTEST(lc79h, test_queries_both_reply_orders)
{
	gnss_systems_t systems = 0;
	struct gnss_quectel_lc79h_epo_status status;

	for (int mode = 0; mode <= 12; mode += 12) {
		response_mode = mode;
		zassert_ok(gnss_get_enabled_systems(gnss, &systems));
		zassert_equal(systems, GNSS_SYSTEM_GPS | GNSS_SYSTEM_GLONASS | GNSS_SYSTEM_GALILEO |
					       GNSS_SYSTEM_BEIDOU | GNSS_SYSTEM_QZSS);
		zassert_ok(gnss_quectel_lc79h_get_epo_status(gnss, 1, &status));
		zassert_equal(status.sets, 1);
		zassert_equal(status.flash_start_week, 2098);
		zassert_equal(status.flash_start_tow, 194400);
		zassert_equal(status.used_end_week, 2098);
		zassert_equal(status.used_end_tow, 216000);
	}
}

ZTEST(lc79h, test_query_needs_data_and_ack)
{
	gnss_systems_t systems = GNSS_SYSTEM_GPS;

	response_mode = 14;
	zassert_equal(gnss_get_enabled_systems(gnss, &systems), -ETIMEDOUT);
	zassert_equal(systems, GNSS_SYSTEM_GPS);
}

ZTEST(lc79h, test_epo_query_wrong_system)
{
	struct gnss_quectel_lc79h_epo_status status = {.sets = 99};

	response_mode = 13;
	zassert_equal(gnss_quectel_lc79h_get_epo_status(gnss, 0, &status), -ETIMEDOUT);
	zassert_equal(status.sets, 99);
}

ZTEST(lc79h, test_search_mode_rejects_invalid_flags)
{
	gnss_systems_t systems = 0;

	response_mode = 15;
	zassert_equal(gnss_get_enabled_systems(gnss, &systems), -ETIMEDOUT);
}

ZTEST(lc79h, test_query_nack_and_arguments)
{
	gnss_systems_t systems = 0;
	struct gnss_quectel_lc79h_epo_status status;

	response_mode = 16;
	zassert_equal(gnss_get_enabled_systems(gnss, &systems), -ENOTSUP);
	zassert_equal(gnss_quectel_lc79h_get_epo_status(gnss, 4, &status), -EINVAL);
	zassert_equal(gnss_quectel_lc79h_get_epo_status(gnss, 0, NULL), -EINVAL);
}

ZTEST_SUITE(lc79h, NULL, NULL, before, NULL, NULL);
