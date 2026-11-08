/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */
#include "gnss_quectel_lc79h_protocol.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>

static uint32_t word_le(const uint8_t *data)
{
	return (uint32_t)data[0] | (uint32_t)data[1] << 8 | (uint32_t)data[2] << 16 |
	       (uint32_t)data[3] << 24;
}

static int finish(char *out, size_t capacity, int length)
{
	if (length < 0 || (size_t)length + 6 > capacity) {
		return -ENOSPC;
	}
	uint8_t checksum = 0;

	for (int i = 1; i < length; ++i) {
		checksum ^= (uint8_t)out[i];
	}
	snprintf(out + length, capacity - (size_t)length, "*%02X\r\n", checksum);
	return length + 5;
}

int lc79h_encode_time(char *out, size_t capacity, int64_t seconds)
{
	/* Bound calendar conversion and reject pre-GNSS dates. */
	if (out == NULL || seconds < 347155200 || seconds >= 4102444800LL) {
		return -EINVAL;
	}
	int64_t days = seconds / 86400;
	int remaining = (int)(seconds % 86400);
	int64_t z = days + 719468;
	int era = (int)(z / 146097);
	uint32_t doe = (uint32_t)(z - (int64_t)era * 146097);
	uint32_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
	int year = (int)yoe + era * 400;
	uint32_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
	uint32_t mp = (5 * doy + 2) / 153;
	uint32_t day = doy - (153 * mp + 2) / 5 + 1;
	int month = (int)mp + (mp < 10 ? 3 : -9);

	year += month <= 2;
	return finish(out, capacity,
		      snprintf(out, capacity, "$PAIR590,%d,%d,%u,%d,%d,%d", year, month, day,
			       remaining / 3600, remaining / 60 % 60, remaining % 60));
}

int lc79h_encode_position(char *out, size_t capacity,
			  const struct gnss_quectel_lc79h_reference_position *p)
{
	if (out == NULL || p == NULL || p->latitude_udeg < -90000000 ||
	    p->latitude_udeg > 90000000 || p->longitude_udeg < -180000000 ||
	    p->longitude_udeg > 180000000 || p->horizontal_accuracy_m == 0 ||
	    p->horizontal_accuracy_m > 30000 || p->vertical_accuracy_m == 0 ||
	    p->vertical_accuracy_m > 100000) {
		return -EINVAL;
	}
	uint32_t lat = (uint32_t)(p->latitude_udeg < 0 ? -p->latitude_udeg : p->latitude_udeg);
	uint32_t lon = (uint32_t)(p->longitude_udeg < 0 ? -p->longitude_udeg : p->longitude_udeg);

	return finish(out, capacity,
		      snprintf(out, capacity, "$PAIR600,%s%u.%06u,%s%u.%06u,%d,%u,%u,0,%u",
			       p->latitude_udeg < 0 ? "-" : "", lat / 1000000, lat % 1000000,
			       p->longitude_udeg < 0 ? "-" : "", lon / 1000000, lon % 1000000,
			       p->altitude_m, p->horizontal_accuracy_m, p->horizontal_accuracy_m,
			       p->vertical_accuracy_m));
}

static int satellite(uint8_t id, uint32_t *system, uint32_t *prn)
{
	if (id >= 1 && id <= 32) {
		*system = 0;
		*prn = id;
	} else if (id >= 65 && id <= 88) {
		*system = 1;
		*prn = id - 64;
	} else {
		return -ENOTSUP;
	}
	return 0;
}

int lc79h_encode_epo(char *out, size_t capacity, const uint8_t record[LC79H_EPO_RECORD_SIZE])
{
	uint32_t system, prn;

	if (out == NULL || record == NULL) {
		return -EINVAL;
	}
	int ret = satellite(record[3], &system, &prn);

	if (ret < 0) {
		return ret;
	}
	int used = snprintf(out, capacity, "$PAIR471,%u,%X", system, prn);

	for (uint32_t i = 0; i < 18; ++i) {
		if (used < 0 || (size_t)used >= capacity) {
			return -ENOSPC;
		}
		int n = snprintf(out + used, capacity - (size_t)used, ",%X",
				 word_le(record + i * 4));
		if (n < 0 || (size_t)n >= capacity - (size_t)used) {
			return -ENOSPC;
		}
		used += n;
	}
	return finish(out, capacity, used);
}

static int digit(char c)
{
	if (c >= '0' && c <= '9') {
		return c - '0';
	}
	if (c >= 'A' && c <= 'F') {
		return c - 'A' + 10;
	}
	if (c >= 'a' && c <= 'f') {
		return c - 'a' + 10;
	}
	return -1;
}

static bool number(const char **cursor, uint32_t base, uint32_t *value)
{
	const char *p = *cursor;
	uint32_t n = 0, count = 0;

	while (*p != '\0' && *p != ',' && *p != '*') {
		int d = digit(*p++);

		if (d < 0 || (uint32_t)d >= base || ++count > (base == 16U ? 8U : 6U)) {
			return false;
		}
		n = n * base + (uint32_t)d;
	}
	if (count == 0U) {
		return false;
	}
	*cursor = p;
	*value = n;
	return true;
}

bool lc79h_parse_reply(const char *line, struct lc79h_reply *reply)
{
	if (line == NULL || reply == NULL || strncmp(line, "$PAIR", 5) != 0) {
		return false;
	}
	const char *star = strchr(line, '*');

	if (star == NULL || strlen(star) != 3 || digit(star[1]) < 0 || digit(star[2]) < 0) {
		return false;
	}
	uint8_t checksum = 0;

	for (const char *p = line + 1; p < star; ++p) {
		checksum ^= (uint8_t)*p;
	}
	if (checksum != (digit(star[1]) << 4 | digit(star[2]))) {
		return false;
	}
	const char *p = line + 5;
	struct lc79h_reply parsed = {0};
	uint32_t second;

	if (!number(&p, 10, &parsed.command) || *p++ != ',') {
		return false;
	}
	if (parsed.command == 67 || parsed.command == 470) {
		uint32_t count = parsed.command == 67 ? 6U : 10U;

		for (uint32_t i = 0; i < count; ++i) {
			if (!number(&p, 10, &parsed.values[i])) {
				return false;
			}
			if (i + 1U < count && *p++ != ',') {
				return false;
			}
		}
		if (p != star) {
			return false;
		}
		if (parsed.command == 67) {
			if (parsed.values[0] > 1U || parsed.values[1] > 1U ||
			    parsed.values[2] > 1U || parsed.values[3] > 1U ||
			    parsed.values[5] != 0U) {
				return false;
			}
		} else {
			if (parsed.values[0] > 3U) {
				return false;
			}
			for (uint32_t i = 2; i < count; i += 2) {
				if (parsed.values[i] > UINT16_MAX ||
				    parsed.values[i + 1] >= 604800U) {
					return false;
				}
			}
		}
		*reply = parsed;
		return true;
	}
	if (parsed.command == 471) {
		if (!number(&p, 16, &parsed.first)) {
			return false;
		}
		if (p == star) {
			parsed.epo_word_echo = true;
			*reply = parsed;
			return true;
		}
		/* The alternate response is decimal system ID and hexadecimal PRN. */
		if (parsed.first > 3U) {
			return false;
		}
	} else if (!number(&p, 10, &parsed.first)) {
		return false;
	}
	if (*p++ != ',') {
		return false;
	}
	bool negative = *p == '-';

	if (negative) {
		++p;
	}
	if (!number(&p, parsed.command == 471 ? 16 : 10, &second) || second > INT32_MAX) {
		return false;
	}
	parsed.second = negative ? -(int)second : (int)second;
	if (parsed.command == 10 && parsed.first == 0) {
		uint32_t ignored;

		if (*p++ != ',' || !number(&p, 10, &ignored) || *p++ != ',' ||
		    !number(&p, 10, &ignored)) {
			return false;
		}
	}
	if (p != star) {
		return false;
	}
	*reply = parsed;
	return true;
}
