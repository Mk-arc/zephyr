/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_DRIVERS_GNSS_QUECTEL_LC79H_PROTOCOL_H_
#define ZEPHYR_DRIVERS_GNSS_QUECTEL_LC79H_PROTOCOL_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/drivers/gnss/quectel_lc79h.h>

#define LC79H_SENTENCE_SIZE   192
#define LC79H_EPO_RECORD_SIZE 72

int lc79h_encode_time(char *out, size_t capacity, int64_t seconds);
int lc79h_encode_position(char *out, size_t capacity,
			  const struct gnss_quectel_lc79h_reference_position *position);
int lc79h_encode_epo(char *out, size_t capacity, const uint8_t record[LC79H_EPO_RECORD_SIZE]);

struct lc79h_reply {
	uint32_t command;
	uint32_t values[10]; /* PAIR067 search flags or PAIR470 EPO status. */
	uint32_t first;
	int32_t second;
	bool epo_word_echo; /* PAIR471 echoes W[0] on some receiver firmware. */
};
bool lc79h_parse_reply(const char *line, struct lc79h_reply *reply);

#endif /* ZEPHYR_DRIVERS_GNSS_QUECTEL_LC79H_PROTOCOL_H_ */
