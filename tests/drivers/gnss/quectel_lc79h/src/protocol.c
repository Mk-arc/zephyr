/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */
#include "gnss_quectel_lc79h_protocol.h"
#include <zephyr/ztest.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

static void sentence(char *out, const char *body)
{
	uint32_t checksum = 0;

	for (const char *p = body; *p != '\0'; ++p) {
		checksum ^= (unsigned char)*p;
	}
	sprintf(out, "$%s*%02X", body, checksum);
}

ZTEST(lc79h, test_protocol_encoding)
{
	char line[LC79H_SENTENCE_SIZE];

	zassert_true(lc79h_encode_time(line, sizeof(line), 1549789258) > 0);
	zassert_true(strcmp(line, "$PAIR590,2019,2,10,9,0,58*0B\r\n") == 0);
	zassert_true(lc79h_encode_time(line, 10, 1549789258) == -ENOSPC);
	zassert_true(lc79h_encode_time(line, sizeof(line), 0) == -EINVAL);
	zassert_true(lc79h_encode_time(line, sizeof(line), 1709251199) > 0);
	zassert_true(strncmp(line, "$PAIR590,2024,2,29,23,59,59*", 27) == 0);
	struct gnss_quectel_lc79h_reference_position position = {-1, -12345678, -12, 30000, 10000};

	zassert_true(lc79h_encode_position(line, sizeof(line), &position) > 0);
	zassert_true(strstr(line, "-0.000001,-12.345678,-12,30000,30000,0,10000*") != NULL);
	position.horizontal_accuracy_m = 30001;
	zassert_true(lc79h_encode_position(line, sizeof(line), &position) == -EINVAL);
	position.horizontal_accuracy_m = 0;
	zassert_true(lc79h_encode_position(line, sizeof(line), &position) == -EINVAL);

	struct lc79h_reply reply;

	zassert_true(lc79h_parse_reply("$PAIR001,590,0*37", &reply));
	zassert_true(reply.command == 1 && reply.first == 590 && reply.second == 0);
	zassert_true(!lc79h_parse_reply("$PAIR001,590,0*38", &reply));
	zassert_true(!lc79h_parse_reply("$PAIR001,590,0*37junk", &reply));
	zassert_true(!lc79h_parse_reply("$PAIR001,590,0*7", &reply));
	zassert_true(lc79h_parse_reply("$PAIR471,1,16*0E", &reply));
	zassert_true(reply.first == 1 && reply.second == 22);
	zassert_false(reply.epo_word_echo);
	zassert_true(lc79h_parse_reply("$PAIR471,56056272*13", &reply));
	zassert_true(reply.epo_word_echo);
	zassert_equal(reply.first, 0x56056272U);
	sentence(line, "PAIR471,FFFFFFFF");
	zassert_true(lc79h_parse_reply(line, &reply));
	zassert_equal(reply.first, UINT32_MAX);
	sentence(line, "PAIR471,156056272");
	zassert_false(lc79h_parse_reply(line, &reply));
	sentence(line, "PAIR471,5605627G");
	zassert_false(lc79h_parse_reply(line, &reply));
	sentence(line, "PAIR471,56056272,0");
	zassert_false(lc79h_parse_reply(line, &reply));
	zassert_false(lc79h_parse_reply("$PAIR471,56056272*12", &reply));
	sentence(line, "PAIR471,1,-80000000");
	zassert_false(lc79h_parse_reply(line, &reply));
	zassert_true(lc79h_parse_reply("$PAIR010,1,-1*16", &reply));
	zassert_true(lc79h_parse_reply("$PAIR010,0,0,2044,369413*33", &reply));
	sentence(line, "PAIR001,590,0,ignored");
	zassert_true(!lc79h_parse_reply(line, &reply));
	sentence(line, "PAIR001,590,+0");
	zassert_true(!lc79h_parse_reply(line, &reply));

	sentence(line, "PAIR067,1,1,1,1,1,0");
	zassert_true(lc79h_parse_reply(line, &reply));
	zassert_equal(reply.values[4], 1);
	sentence(line, "PAIR067,1,1,1,1,0");
	zassert_false(lc79h_parse_reply(line, &reply));
	sentence(line, "PAIR067,1,1,1,1,0,1");
	zassert_false(lc79h_parse_reply(line, &reply));
	sentence(line, "PAIR470,0,0,0,0,0,0,0,0,0,0");
	zassert_true(lc79h_parse_reply(line, &reply));
	zassert_equal(reply.values[1], 0);
	sentence(line, "PAIR470,0,1,2098,604800,2098,216000,2098,194400,2098,216000");
	zassert_false(lc79h_parse_reply(line, &reply));

	uint8_t record[72];

	memset(record, 0xff, sizeof(record));
	record[3] = 86; /* GLONASS PRN 22, represented as hexadecimal 16. */
	zassert_true(lc79h_encode_epo(line, sizeof(line), record) > 0);
	zassert_true(strncmp(line, "$PAIR471,1,16,56FFFFFF,FFFFFFFF", 30) == 0);
	zassert_true(strlen(line) < sizeof(line));
	zassert_true(lc79h_encode_epo(line, 20, record) == -ENOSPC);
	record[3] = 255;
	zassert_true(lc79h_encode_epo(line, sizeof(line), record) == -ENOTSUP);
}
