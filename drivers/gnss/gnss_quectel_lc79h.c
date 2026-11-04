/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT quectel_lc79h

#include "gnss_nmea0183_match.h"
#include "gnss_quectel_lc79h_protocol.h"
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/drivers/gnss.h>
#include <zephyr/drivers/gnss/quectel_lc79h.h>
#include <zephyr/modem/backend/uart.h>
#include <zephyr/modem/chat.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>

struct lc79h_config {
	const struct device *uart;
};

struct lc79h_data {
	/* Must remain first: Zephyr's NMEA match callbacks use this prefix. */
	struct gnss_nmea0183_match_data nmea;
#if CONFIG_GNSS_SATELLITES
	struct gnss_satellite satellites[64];
#endif
	struct modem_backend_uart backend;
	struct modem_pipe *pipe;
	uint8_t rx[1024];
	uint8_t tx[LC79H_SENTENCE_SIZE];
	struct modem_chat chat;
	uint8_t chat_rx[256];
	uint8_t *argv[32];
	struct k_mutex command_lock;
	struct k_spinlock reply_lock;
	struct k_sem reply_ready;
	atomic_t aiding_sequence;
	atomic_t epo_requests[5];
	bool active;
	bool poisoned;
	bool pending;
	bool ack;
	bool echo;
	uint32_t command;
	uint32_t system;
	uint32_t prn;
	uint32_t epo_word;
	int result;
	struct lc79h_reply response;
};

static void pair_received(struct modem_chat *chat, char **argv, uint16_t argc, void *user)
{
	struct lc79h_data *data = user;
	struct lc79h_reply reply;
	char line[LC79H_SENTENCE_SIZE];

	ARG_UNUSED(chat);
	if (argc != 2) {
		return;
	}
	int n = snprintf(line, sizeof(line), "%s%s", argv[0], argv[1]);

	if (n < 0 || n >= sizeof(line) || !lc79h_parse_reply(line, &reply)) {
		return;
	}
	if (reply.command == 10 && reply.first <= 2) {
		if (reply.first == 0) {
			if (reply.second >= 0 && reply.second < ARRAY_SIZE(data->epo_requests)) {
				atomic_inc(&data->epo_requests[reply.second]);
			}
		} else {
			atomic_inc(&data->aiding_sequence);
		}
		return;
	}
	k_spinlock_key_t key = k_spin_lock(&data->reply_lock);

	if (data->pending) {
		if (reply.command == 1 && reply.first == data->command) {
			switch (reply.second) {
			case 0:
				data->ack = true;
				break;
			case 1:
				break; /* Processing, not a final acknowledgment. */
			case 2:
				data->result = -EIO;
				break;
			case 3:
				data->result = -ENOTSUP;
				break;
			case 4:
				data->result = -EINVAL;
				break;
			case 5:
				data->result = -EBUSY;
				break;
			default:
				break;
			}
		} else if (reply.command == 471 && data->command == 471 &&
			   (reply.epo_word_echo
				    ? reply.first == data->epo_word
				    : (reply.first == data->system && reply.second == data->prn))) {
			data->echo = true;
		}
		if (reply.command == data->command &&
		    (reply.command == 67 ||
		     (reply.command == 470 && reply.values[0] == data->system))) {
			data->response = reply;
			data->echo = true;
		}
		if (data->result < 0 ||
		    (data->ack &&
		     ((data->command != 471 && data->command != 67 && data->command != 470) ||
		      data->echo))) {
			data->pending = false;
			k_sem_give(&data->reply_ready);
		}
	}
	k_spin_unlock(&data->reply_lock, key);
}

MODEM_CHAT_MATCHES_DEFINE(
	lc79h_matches, MODEM_CHAT_MATCH_WILDCARD("$??GGA,", ",*", gnss_nmea0183_match_gga_callback),
	MODEM_CHAT_MATCH_WILDCARD("$??RMC,", ",*", gnss_nmea0183_match_rmc_callback),
#if CONFIG_GNSS_SATELLITES
	MODEM_CHAT_MATCH_WILDCARD("$??GSV,", ",*", gnss_nmea0183_match_gsv_callback),
#endif
	MODEM_CHAT_MATCH("$PAIR", "", pair_received));

static int send_command(const struct device *dev, const char *sentence, size_t length,
			uint32_t command, uint32_t system, uint32_t prn, uint32_t epo_word,
			struct lc79h_reply *response)
{
	struct lc79h_data *data = dev->data;
	int result = 0;

	k_mutex_lock(&data->command_lock, K_FOREVER);
	if (!data->active || data->poisoned) {
		result = data->poisoned ? -EPIPE : -EHOSTDOWN;
		goto unlock;
	}
	k_sem_reset(&data->reply_ready);
	k_spinlock_key_t key = k_spin_lock(&data->reply_lock);

	data->command = command;
	data->system = system;
	data->prn = prn;
	data->epo_word = epo_word;
	data->ack = false;
	data->echo = false;
	data->result = 0;
	data->pending = true;
	k_spin_unlock(&data->reply_lock, key);

	int64_t deadline = k_uptime_get() + 3000;
	size_t sent = 0;

	while (sent < length && k_uptime_get() < deadline) {
		int n = modem_pipe_transmit(data->pipe, (const uint8_t *)sentence + sent,
					    length - sent);
		if (n < 0) {
			result = n;
			break;
		}
		sent += (size_t)n;
		if (sent < length) {
			k_sleep(K_MSEC(1));
		}
	}
	int64_t left = deadline - k_uptime_get();

	if (result == 0 &&
	    (sent != length || left <= 0 || k_sem_take(&data->reply_ready, K_MSEC(left)) != 0)) {
		result = -ETIMEDOUT;
	}
	key = k_spin_lock(&data->reply_lock);
	if (result == 0) {
		result = data->result;
		if (result == 0 && response != NULL) {
			*response = data->response;
		}
	} else {
		/* No transaction IDs: never let a late reply acknowledge a retry. */
		data->poisoned = true;
	}
	data->pending = false;
	k_spin_unlock(&data->reply_lock, key);
unlock:
	k_mutex_unlock(&data->command_lock);
	return result;
}

int gnss_quectel_lc79h_set_reference_time(const struct device *dev, int64_t seconds)
{
	char sentence[LC79H_SENTENCE_SIZE];
	int n = lc79h_encode_time(sentence, sizeof(sentence), seconds);

	return n < 0 ? n : send_command(dev, sentence, (size_t)n, 590, 0, 0, 0, NULL);
}

int gnss_quectel_lc79h_set_reference_position(
	const struct device *dev, const struct gnss_quectel_lc79h_reference_position *position)
{
	char sentence[LC79H_SENTENCE_SIZE];
	int n = lc79h_encode_position(sentence, sizeof(sentence), position);

	return n < 0 ? n : send_command(dev, sentence, (size_t)n, 600, 0, 0, 0, NULL);
}

int gnss_quectel_lc79h_set_epo_record(const struct device *dev, const uint8_t record[72])
{
	char sentence[LC79H_SENTENCE_SIZE];
	int n = lc79h_encode_epo(sentence, sizeof(sentence), record);

	if (n < 0) {
		return n;
	}
	uint32_t system = record[3] > 32 ? 1 : 0;
	uint32_t prn = record[3] - (system != 0U ? 64U : 0U);

	return send_command(dev, sentence, (size_t)n, 471, system, prn, sys_get_le32(record), NULL);
}

static int lc79h_get_enabled_systems(const struct device *dev, gnss_systems_t *systems)
{
	static const char sentence[] = "$PAIR067*3B\r\n";
	static const gnss_systems_t mapping[] = {
		GNSS_SYSTEM_GPS,    GNSS_SYSTEM_GLONASS, GNSS_SYSTEM_GALILEO,
		GNSS_SYSTEM_BEIDOU, GNSS_SYSTEM_QZSS,
	};
	struct lc79h_reply reply;
	gnss_systems_t enabled = 0;
	int ret;

	if (systems == NULL) {
		return -EINVAL;
	}
	ret = send_command(dev, sentence, sizeof(sentence) - 1, 67, 0, 0, 0, &reply);
	if (ret < 0) {
		return ret;
	}
	for (size_t i = 0; i < ARRAY_SIZE(mapping); ++i) {
		if (reply.values[i] != 0U) {
			enabled |= mapping[i];
		}
	}
	*systems = enabled;
	return 0;
}

int gnss_quectel_lc79h_get_epo_status(const struct device *dev, uint8_t system,
				      struct gnss_quectel_lc79h_epo_status *status)
{
	char sentence[24];
	struct lc79h_reply reply;
	int ret;

	if (system > 3U || status == NULL) {
		return -EINVAL;
	}
	/* XOR checksum of PAIR470,0 is 0x25; only the last digit changes. */
	ret = snprintf(sentence, sizeof(sentence), "$PAIR470,%u*%02X\r\n", system, 0x25U ^ system);
	if (ret < 0 || ret >= sizeof(sentence)) {
		return -ENOSPC;
	}
	ret = send_command(dev, sentence, (size_t)ret, 470, system, 0, 0, &reply);
	if (ret < 0) {
		return ret;
	}
	status->sets = reply.values[1];
	status->flash_start_week = reply.values[2];
	status->flash_start_tow = reply.values[3];
	status->flash_end_week = reply.values[4];
	status->flash_end_tow = reply.values[5];
	status->used_start_week = reply.values[6];
	status->used_start_tow = reply.values[7];
	status->used_end_week = reply.values[8];
	status->used_end_tow = reply.values[9];
	return 0;
}

uint32_t gnss_quectel_lc79h_aiding_sequence(const struct device *dev, gnss_systems_t epo_systems)
{
	struct lc79h_data *data = dev->data;
	static const gnss_systems_t mapping[] = {
		GNSS_SYSTEM_GPS,    GNSS_SYSTEM_GLONASS, GNSS_SYSTEM_GALILEO,
		GNSS_SYSTEM_BEIDOU, GNSS_SYSTEM_QZSS,
	};
	uint32_t sequence = (uint32_t)atomic_get(&data->aiding_sequence);

	for (size_t i = 0; i < ARRAY_SIZE(mapping); ++i) {
		if ((epo_systems & mapping[i]) != 0U) {
			sequence += (uint32_t)atomic_get(&data->epo_requests[i]);
		}
	}
	return sequence;
}

static int lc79h_resume(const struct device *dev)
{
	struct lc79h_data *data = dev->data;
	int ret = modem_pipe_open(data->pipe, K_SECONDS(1));

	if (ret < 0) {
		return ret;
	}
	ret = modem_chat_attach(&data->chat, data->pipe);
	if (ret < 0) {
		modem_pipe_close(data->pipe, K_SECONDS(1));
		return ret;
	}
	data->active = true;
	data->poisoned = false;
	atomic_inc(&data->aiding_sequence);
	return 0;
}

#ifdef CONFIG_PM_DEVICE
static int lc79h_pm_action(const struct device *dev, enum pm_device_action action)
{
	struct lc79h_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->command_lock, K_FOREVER);
	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		ret = lc79h_resume(dev);
		break;
	case PM_DEVICE_ACTION_SUSPEND:
		data->active = false;
		modem_chat_release(&data->chat);
		ret = modem_pipe_close(data->pipe, K_SECONDS(1));
		data->nmea.gga_utc = 0;
		data->nmea.rmc_utc = 0;
		break;
	default:
		ret = -ENOTSUP;
		break;
	}
	k_mutex_unlock(&data->command_lock);
	return ret;
}
#endif

static int lc79h_init(const struct device *dev)
{
	const struct lc79h_config *config = dev->config;
	struct lc79h_data *data = dev->data;
	static const uint8_t delimiter[] = {'\r', '\n'};

	k_mutex_init(&data->command_lock);
	k_sem_init(&data->reply_ready, 0, 1);
	const struct gnss_nmea0183_match_config nmea = {
		.gnss = dev,
#if CONFIG_GNSS_SATELLITES
		.satellites = data->satellites,
		.satellites_size = ARRAY_SIZE(data->satellites),
#endif
	};
	int ret = gnss_nmea0183_match_init(&data->nmea, &nmea);

	if (ret < 0) {
		return ret;
	}
	const struct modem_backend_uart_config uart = {
		.uart = config->uart,
		.receive_buf = data->rx,
		.receive_buf_size = sizeof(data->rx),
		.transmit_buf = data->tx,
		.transmit_buf_size = sizeof(data->tx),
	};
	data->pipe = modem_backend_uart_init(&data->backend, &uart);
	if (data->pipe == NULL) {
		return -ENODEV;
	}
	const struct modem_chat_config chat = {
		.user_data = data,
		.receive_buf = data->chat_rx,
		.receive_buf_size = sizeof(data->chat_rx),
		.delimiter = delimiter,
		.delimiter_size = sizeof(delimiter),
		.argv = data->argv,
		.argv_size = ARRAY_SIZE(data->argv),
		.unsol_matches = lc79h_matches,
		.unsol_matches_size = ARRAY_SIZE(lc79h_matches),
	};
	ret = modem_chat_init(&data->chat, &chat);
	if (ret < 0) {
		return ret;
	}
#ifdef CONFIG_PM_DEVICE
	pm_device_init_suspended(dev);
	return 0;
#else
	return lc79h_resume(dev);
#endif
}

static DEVICE_API(gnss, lc79h_api) = {
	.get_enabled_systems = lc79h_get_enabled_systems,
};

#define LC79H_DEFINE(inst)                                                                         \
	static struct lc79h_data lc79h_data_##inst;                                                \
	static const struct lc79h_config lc79h_config_##inst = {                                   \
		.uart = DEVICE_DT_GET(DT_INST_BUS(inst)),                                          \
	};                                                                                         \
	PM_DEVICE_DT_INST_DEFINE(inst, lc79h_pm_action);                                           \
	DEVICE_DT_INST_DEFINE(inst, lc79h_init, PM_DEVICE_DT_INST_GET(inst), &lc79h_data_##inst,   \
			      &lc79h_config_##inst, POST_KERNEL, CONFIG_GNSS_INIT_PRIORITY,        \
			      &lc79h_api);

DT_INST_FOREACH_STATUS_OKAY(LC79H_DEFINE)
