/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "le_audio.h"
#include "uart_async_adapter.h"

#include <zephyr/zbus/zbus.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <bluetooth/services/nus.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/bluetooth/audio/bap_lc3_preset.h>
/* TODO: Remove when a get_info function is implemented in host */
#include <../subsys/bluetooth/audio/bap_endpoint.h>
#include <../subsys/bluetooth/audio/bap_iso.h>
#include <zephyr/bluetooth/audio/csip.h>
#include "ble_hci_vsc.h"

#include <stdio.h>

#include "macros_common.h"
#include "nrf5340_audio_common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bis_gateway, CONFIG_BLE_LOG_LEVEL);

BUILD_ASSERT(CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT <= 2,
	     "A maximum of two audio streams are currently supported");

ZBUS_CHAN_DEFINE(le_audio_chan, struct le_audio_msg, NULL, NULL, ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(0));

#define STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE
#define PRIORITY 7
#define UART_BUF_SIZE CONFIG_BT_NUS_UART_BUFFER_SIZE
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_WAIT_FOR_RX CONFIG_BT_NUS_UART_RX_WAIT_TIME

#define HCI_ISO_BUF_ALLOC_PER_CHAN 2
#define STANDARD_QUALITY_16KHZ 16000
#define STANDARD_QUALITY_24KHZ 24000
#define HIGH_QUALITY_48KHZ 48000

/* For being able to dynamically define iso_tx_pools */
#define NET_BUF_POOL_ITERATE(i, _)                                                                 \
	NET_BUF_POOL_FIXED_DEFINE(iso_tx_pool_##i, HCI_ISO_BUF_ALLOC_PER_CHAN,                     \
				  BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU), 8, NULL);
#define NET_BUF_POOL_PTR_ITERATE(i, ...) IDENTITY(&iso_tx_pool_##i)
LISTIFY(CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT, NET_BUF_POOL_ITERATE, (;))

/* clang-format off */
static struct net_buf_pool *iso_tx_pools[] = { LISTIFY(CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT,
						       NET_BUF_POOL_PTR_ITERATE, (,)) };
/* clang-format on */

static struct bt_bap_broadcast_source *broadcast_source;

static struct bt_bap_stream audio_streams[CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT];

static struct bt_bap_lc3_preset lc3_preset = BT_BAP_LC3_BROADCAST_PRESET_NRF5340_AUDIO;

static atomic_t iso_tx_pool_alloc[CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT];
static bool delete_broadcast_src;
static uint32_t seq_num[CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT];

static struct bt_le_ext_adv *adv;
static struct bt_le_ext_adv *adv_conn;
static struct bt_conn *default_conn;
static struct k_work adv_work;

static le_audio_timestamp_cb timestamp_cb;

static K_SEM_DEFINE(ble_init_ok, 0, 1);

static const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));
static struct k_work_delayable uart_work;

struct uart_data_t {
	void *fifo_reserved;
	uint8_t data[UART_BUF_SIZE];
	uint16_t len;
};

static K_FIFO_DEFINE(fifo_uart_tx_data);
static K_FIFO_DEFINE(fifo_uart_rx_data);

/* Bonded address queue */
K_MSGQ_DEFINE(bonds_queue, sizeof(bt_addr_le_t), CONFIG_BT_MAX_PAIRED, 4);

static const struct bt_data ad_peer[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL)

	};

static void le_audio_event_publish(enum le_audio_evt_type event)
{
	int ret;
	struct le_audio_msg msg;

	msg.event = event;

	ret = zbus_chan_pub(&le_audio_chan, &msg, K_NO_WAIT);
	ERR_CHK(ret);
}

static bool is_iso_buffer_full(uint8_t idx)
{
	/* net_buf_alloc allocates buffers for APP->NET transfer over HCI RPMsg,
	 * but when these buffers are released it is not guaranteed that the
	 * data has actually been sent. The data might be qued on the NET core,
	 * and this can cause delays in the audio.
	 * When stream_sent_cb() is called the data has been sent.
	 * Data will be discarded if allocation becomes too high, to avoid audio delays.
	 * If the NET and APP core operates in clock sync, discarding should not occur.
	 */

	if (atomic_get(&iso_tx_pool_alloc[idx]) >= HCI_ISO_BUF_ALLOC_PER_CHAN) {
		return true;
	}

	return false;
}

static int get_stream_index(struct bt_bap_stream *stream, uint8_t *index)
{
	for (int i = 0; i < ARRAY_SIZE(audio_streams); i++) {
		if (&audio_streams[i] == stream) {
			*index = i;
			return 0;
		}
	}

	LOG_WRN("Stream %p not found", (void *)stream);

	return -EINVAL;
}

static void stream_sent_cb(struct bt_bap_stream *stream)
{
	static uint32_t sent_cnt[ARRAY_SIZE(audio_streams)];
	uint8_t index = 0;

	get_stream_index(stream, &index);

	if (atomic_get(&iso_tx_pool_alloc[index])) {
		atomic_dec(&iso_tx_pool_alloc[index]);
	} else {
		LOG_WRN("Decreasing atomic variable for stream %d failed", index);
	}

	sent_cnt[index]++;

	if ((sent_cnt[index] % 1000U) == 0U) {
		LOG_DBG("Sent %d total ISO packets on stream %d", sent_cnt[index], index);
	}
}

static void stream_started_cb(struct bt_bap_stream *stream)
{
	uint8_t index = 0;

	get_stream_index(stream, &index);
	seq_num[index] = 0;

	le_audio_event_publish(LE_AUDIO_EVT_STREAMING);

	LOG_INF("Broadcast source %p started", (void *)stream);
}

static void stream_stopped_cb(struct bt_bap_stream *stream, uint8_t reason)
{
	int ret;

	le_audio_event_publish(LE_AUDIO_EVT_NOT_STREAMING);

	LOG_INF("Broadcast source %p stopped. Reason: %d", (void *)stream, reason);

	if (delete_broadcast_src && broadcast_source != NULL) {
		ret = bt_bap_broadcast_source_delete(broadcast_source);
		if (ret) {
			LOG_ERR("Unable to delete broadcast source %p", (void *)stream);
			delete_broadcast_src = false;
			return;
		}

		broadcast_source = NULL;

		LOG_INF("Broadcast source %p deleted", (void *)stream);

		delete_broadcast_src = false;
	}
}

static struct bt_bap_stream_ops stream_ops = {
	.started = stream_started_cb,
	.stopped = stream_stopped_cb,
#if (CONFIG_BT_AUDIO_TX)
	.sent = stream_sent_cb,
#endif /* (CONFIG_BT_AUDIO_TX) */
};

#if (CONFIG_AURACAST)
static void public_broadcast_features_set(uint8_t *features)
{
	int freq = bt_codec_cfg_get_freq(&lc3_preset.codec);

	if (features == NULL) {
		LOG_ERR("No pointer to features");
		return;
	}

	if (IS_ENABLED(CONFIG_BT_AUDIO_BROADCAST_ENCRYPTED)) {
		*features |= 0x01;
	}

	if (freq == STANDARD_QUALITY_16KHZ || freq == STANDARD_QUALITY_24KHZ) {
		*features |= 0x02;
	} else if (freq == HIGH_QUALITY_48KHZ) {
		*features |= 0x04;
	} else {
		LOG_WRN("%dkHz is not compatible with Auracast, choose 16kHz, 24kHz or 48kHz",
			freq);
	}
}
#endif /* (CONFIG_AURACAST) */

static void advertising_process(struct k_work *work)
{
	int ret;
	struct bt_le_adv_param adv_param;

#if CONFIG_BT_BONDABLE
	bt_addr_le_t addr;

	if (!k_msgq_get(&bonds_queue, &addr, K_NO_WAIT)) {
		char addr_buf[BT_ADDR_LE_STR_LEN];

		adv_param = *BT_LE_ADV_CONN_DIR_LOW_DUTY(&addr);
		adv_param.id = BT_ID_DEFAULT;
		adv_param.options |= BT_LE_ADV_OPT_DIR_ADDR_RPA;

		/* Clear ADV data set before update to direct advertising */
		ret = bt_le_ext_adv_set_data(adv_conn, NULL, 0, NULL, 0);
		if (ret) {
			LOG_ERR("Failed to clear advertising data. Err: %d", ret);
			return;
		}

		ret = bt_le_ext_adv_update_param(adv_conn, &adv_param);
		if (ret) {
			LOG_ERR("Failed to update ext_adv to direct advertising. Err = %d", ret);
			return;
		}

		bt_addr_le_to_str(&addr, addr_buf, BT_ADDR_LE_STR_LEN);
		LOG_INF("Set direct advertising to %s", addr_buf);
	} else
#endif /* CONFIG_BT_BONDABLE */
	{
/*		ret = bt_csip_set_member_generate_rsi(csip, csip_rsi);
		if (ret) {
			LOG_ERR("Failed to generate RSI (ret %d)", ret);
			return;
		}*/

		ret = bt_le_ext_adv_set_data(adv_conn, ad_peer, ARRAY_SIZE(ad_peer), NULL, 0);
		if (ret) {
			LOG_ERR("Failed to set advertising data. Err: %d", ret);
			return;
		}
	}

	ret = bt_le_ext_adv_start(adv_conn, BT_LE_EXT_ADV_START_DEFAULT);
	if (ret) {
		LOG_ERR("Failed to start advertising set. Err: %d", ret);
		return;
	}

	LOG_INF("Advertising successfully started");
}

#if CONFIG_BT_BONDABLE
static void bond_find(const struct bt_bond_info *info, void *user_data)
{
	int ret;

	/* Filter already connected peers. */
	if (default_conn) {
		const bt_addr_le_t *dst = bt_conn_get_dst(default_conn);

		if (!bt_addr_le_cmp(&info->addr, dst)) {
			LOG_DBG("Already connected");
			return;
		}
	}

	ret = k_msgq_put(&bonds_queue, (void *)&info->addr, K_NO_WAIT);
	if (ret) {
		LOG_WRN("No space in the queue for the bond");
	}
}
#endif /* CONFIG_BT_BONDABLE */

static void advertising_start(void)
{
#if CONFIG_BT_BONDABLE
	k_msgq_purge(&bonds_queue);
	bt_foreach_bond(BT_ID_DEFAULT, bond_find, NULL);
#endif /* CONFIG_BT_BONDABLE */

	k_work_submit(&adv_work);
}

static void connected_cb(struct bt_conn *conn, uint8_t err)
{
	int ret;
	char addr[BT_ADDR_LE_STR_LEN];
	uint16_t conn_handle;
	enum ble_hci_vs_tx_power conn_tx_pwr;

	struct bt_le_conn_param param = {
		.interval_min = 64,
		.interval_max = 64,
		.latency = 0,
		.timeout = 3200,
	};
	LOG_INF("Request conn param min=%d max=%d", param.interval_min, param.interval_max);
	bt_conn_le_param_update(conn, &param);  
	if (err) {
		default_conn = NULL;
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected: %s", addr);

	ret = bt_hci_get_conn_handle(conn, &conn_handle);
	if (ret) {
		LOG_ERR("Unable to get conn handle");
	} else {
#if (CONFIG_NRF_21540_ACTIVE)
		conn_tx_pwr = CONFIG_NRF_21540_MAIN_DBM;
#else
		conn_tx_pwr = CONFIG_BLE_CONN_TX_POWER_DBM;
#endif /* (CONFIG_NRF_21540_ACTIVE) */
		ret = ble_hci_vsc_conn_tx_pwr_set(conn_handle, conn_tx_pwr);
		if (ret) {
			LOG_ERR("Failed to set TX power for conn");
		} else {
			LOG_DBG("TX power set to %d dBm for connection %p", conn_tx_pwr,
				(void *)conn);
		}
	}

	default_conn = bt_conn_ref(conn);
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (conn != default_conn) {
		LOG_WRN("Disconnected on wrong conn");
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s (reason 0x%02x)", addr, reason);

	bt_conn_unref(default_conn);
	default_conn = NULL;

/*	ret = ble_mcp_conn_disconnected(conn);
	if (ret) {
		LOG_ERR("ble_msc_conn_disconnected failed with %d", ret);
	}*/

	advertising_start();
}
/*
static void security_changed_cb(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
	int ret;

	if (err) {
		LOG_ERR("Security failed: level %d err %d", level, err);
		ret = bt_conn_disconnect(conn, err);
		if (ret) {
			LOG_ERR("Failed to disconnect %d", ret);
		}
	} else {
		LOG_DBG("Security changed: level %d", level);
	}
}
*/
static struct bt_conn_cb conn_callbacks = {
	.connected = connected_cb,
	.disconnected = disconnected_cb,
//	.security_changed = security_changed_cb,
};

static int adv_create(void)
{
	int ret;

	/* Broadcast Audio Streaming Endpoint advertising data */
	NET_BUF_SIMPLE_DEFINE(ad_buf, BT_UUID_SIZE_16 + BT_AUDIO_BROADCAST_ID_SIZE);
	/* Buffer for Public Broadcast Announcement */
	NET_BUF_SIMPLE_DEFINE(base_buf, 128);

#if (CONFIG_AURACAST)
	NET_BUF_SIMPLE_DEFINE(pba_buf, BT_UUID_SIZE_16 + 2);
	struct bt_data ext_ad[4];
	uint8_t pba_features = 0;
#else
	struct bt_data ext_ad[3];
#endif /* (CONFIG_AURACAST) */
	struct bt_data per_ad;

	uint32_t broadcast_id = 0;

	/* Create a non-connectable non-scannable advertising set */
	ret = bt_le_ext_adv_create(LE_AUDIO_EXTENDED_ADV_NAME, NULL, &adv);
	if (ret) {
		LOG_ERR("Unable to create extended advertising set: %d", ret);
		return ret;
	}
	/* Create a connectable scannable advertising set */
	ret = bt_le_ext_adv_create(LE_AUDIO_EXTENDED_ADV_CONN_NAME, NULL, &adv_conn);
	if (ret) {
		LOG_ERR("Unable to create connectable extended advertising set: %d", ret);
		return ret;
	}

	/* Set periodic advertising parameters */
	ret = bt_le_per_adv_set_param(adv, LE_AUDIO_PERIODIC_ADV);
	if (ret) {
		LOG_ERR("Failed to set periodic advertising parameters (ret %d)", ret);
		return ret;
	}

	if (IS_ENABLED(CONFIG_BT_AUDIO_USE_BROADCAST_ID_RANDOM)) {
		ret = bt_bap_broadcast_source_get_id(broadcast_source, &broadcast_id);
		if (ret) {
			LOG_ERR("Unable to get broadcast ID: %d", ret);
			return ret;
		}
	} else {
		broadcast_id = CONFIG_BT_AUDIO_BROADCAST_ID_FIXED;
	}

	ext_ad[0] = (struct bt_data)BT_DATA_BYTES(BT_DATA_BROADCAST_NAME,
						  CONFIG_BT_AUDIO_BROADCAST_NAME);

	/* Setup extended advertising data */
	net_buf_simple_add_le16(&ad_buf, BT_UUID_BROADCAST_AUDIO_VAL);
	net_buf_simple_add_le24(&ad_buf, broadcast_id);

	ext_ad[1] = (struct bt_data)BT_DATA(BT_DATA_SVC_DATA16, ad_buf.data, ad_buf.len);

	ext_ad[2] = (struct bt_data)BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE,
						  (CONFIG_BT_DEVICE_APPEARANCE >> 0) & 0xff,
						  (CONFIG_BT_DEVICE_APPEARANCE >> 8) & 0xff);

#if (CONFIG_AURACAST)
	public_broadcast_features_set(&pba_features);

	net_buf_simple_add_le16(&pba_buf, 0x1856);
	net_buf_simple_add_u8(&pba_buf, pba_features);
	/* No metadata, set length to 0 */
	net_buf_simple_add_u8(&pba_buf, 0x00);

	ext_ad[3] = (struct bt_data)BT_DATA(BT_DATA_SVC_DATA16, pba_buf.data, pba_buf.len);
#endif /* (CONFIG_AURACAST) */

	ret = bt_le_ext_adv_set_data(adv, ext_ad, ARRAY_SIZE(ext_ad), NULL, 0);
	if (ret) {
		LOG_ERR("Failed to set extended advertising data: %d", ret);
		return ret;
	}

	/* Setup periodic advertising data */
	ret = bt_bap_broadcast_source_get_base(broadcast_source, &base_buf);
	if (ret) {
		LOG_ERR("Failed to get encoded BASE: %d", ret);
		return ret;
	}

	per_ad.type = BT_DATA_SVC_DATA16;
	per_ad.data_len = base_buf.len;
	per_ad.data = base_buf.data;

	ret = bt_le_per_adv_set_data(adv, &per_ad, 1);
	if (ret) {
		LOG_ERR("Failed to set periodic advertising data: %d", ret);
		return ret;
	}

	return 0;
}

static int initialize(le_audio_timestamp_cb timestmp_cb)
{
	int ret;
	static bool initialized;
	struct bt_codec_data bis_codec_data =
		BT_CODEC_DATA(BT_CODEC_CONFIG_LC3_FREQ, BT_AUDIO_CODEC_CONFIG_FREQ);
	struct bt_bap_broadcast_source_stream_param stream_params[ARRAY_SIZE(audio_streams)];
	struct bt_bap_broadcast_source_subgroup_param
		subgroup_params[CONFIG_BT_BAP_BROADCAST_SRC_SUBGROUP_COUNT];
	struct bt_bap_broadcast_source_create_param create_param;

	if (initialized) {
		LOG_WRN("Already initialized");
		return -EALREADY;
	}

	if (timestmp_cb == NULL) {
		LOG_ERR("Timestamp callback is NULL");
		return -EINVAL;
	}

	bt_conn_cb_register(&conn_callbacks);

	timestamp_cb = timestmp_cb;

	(void)memset(audio_streams, 0, sizeof(audio_streams));

	for (size_t i = 0; i < ARRAY_SIZE(stream_params); i++) {
		stream_params[i].stream = &audio_streams[i];
		bt_bap_stream_cb_register(stream_params[i].stream, &stream_ops);
		stream_params[i].data_count = 1U;
		stream_params[i].data = &bis_codec_data;
	}

	for (size_t i = 0U; i < ARRAY_SIZE(subgroup_params); i++) {
		subgroup_params[i].params_count = ARRAY_SIZE(stream_params);
		subgroup_params[i].params = &stream_params[i];
		subgroup_params[i].codec = &lc3_preset.codec;
#if (CONFIG_BT_AUDIO_BROADCAST_IMMEDIATE_FLAG)
		/* Immediate rendering flag */
		subgroup_params[i].codec->meta[0].data.type = 0x09;
		subgroup_params[i].codec->meta[0].data.data_len = 0;
		subgroup_params[i].codec->meta_count = 1;
#endif /* (CONFIG_BT_AUDIO_BROADCAST_IMMEDIATE_FLAG) */
	}

	create_param.params_count = ARRAY_SIZE(subgroup_params);
	create_param.params = subgroup_params;
	create_param.qos = &lc3_preset.qos;

	if (IS_ENABLED(CONFIG_BT_AUDIO_PACKING_INTERLEAVED)) {
		create_param.packing = BT_ISO_PACKING_INTERLEAVED;
	} else {
		create_param.packing = BT_ISO_PACKING_SEQUENTIAL;
	}

	if (IS_ENABLED(CONFIG_BT_AUDIO_BROADCAST_ENCRYPTED)) {
		create_param.encryption = true;
		memset(create_param.broadcast_code, 0, sizeof(create_param.broadcast_code));
		memcpy(create_param.broadcast_code, CONFIG_BT_AUDIO_BROADCAST_ENCRYPTION_KEY,
		       MIN(sizeof(CONFIG_BT_AUDIO_BROADCAST_ENCRYPTION_KEY),
			   sizeof(create_param.broadcast_code)));
	} else {
		create_param.encryption = false;
	}

	LOG_DBG("Creating broadcast source");

	ret = bt_bap_broadcast_source_create(&create_param, &broadcast_source);

	if (ret) {
		LOG_ERR("Failed to create broadcast source, ret: %d", ret);
		return ret;
	}

	/* Create advertising set */
	ret = adv_create();

	if (ret) {
		LOG_ERR("Failed to create advertising set");
		return ret;
	}
	k_work_init(&adv_work, advertising_process);

	initialized = true;

	return 0;
}

int le_audio_user_defined_button_press(enum le_audio_user_defined_action action)
{
	return 0;
}

int le_audio_config_get(uint32_t *bitrate, uint32_t *sampling_rate, uint32_t *pres_delay)
{
	LOG_WRN("Getting config from gateway is not yet supported");
	return -ENOTSUP;
}

int le_audio_volume_up(void)
{
	LOG_WRN("Not possible to increase volume on/from broadcast source");
	return -ENXIO;
}

int le_audio_volume_down(void)
{
	LOG_WRN("Not possible to decrease volume on/from broadcast source");
	return -ENXIO;
}

int le_audio_volume_mute(void)
{
	LOG_WRN("Not possible to mute volume on/from broadcast source");
	return -ENXIO;
}

int le_audio_play_pause(void)
{
	int ret;

	/* All streams in a broadcast source is in the same state,
	 * so we can just check the first stream
	 */
	if (audio_streams[0].ep == NULL) {
		LOG_ERR("stream->ep is NULL");
		return -ECANCELED;
	}

	if (audio_streams[0].ep->status.state == BT_BAP_EP_STATE_STREAMING) {
		ret = bt_bap_broadcast_source_stop(broadcast_source);
		if (ret) {
			LOG_WRN("Failed to stop broadcast, ret: %d", ret);
		}
	} else {
		ret = bt_bap_broadcast_source_start(broadcast_source, adv);
		if (ret) {
			LOG_WRN("Failed to start broadcast, ret: %d", ret);
		}
	}
	advertising_start();

	return ret;
}

int le_audio_send(struct encoded_audio enc_audio)
{
	int ret;
	static bool wrn_printed[CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT];
	struct net_buf *buf;
	size_t num_streams = ARRAY_SIZE(audio_streams);
	size_t data_size_pr_stream;

	if ((enc_audio.num_ch == 1) || (enc_audio.num_ch == num_streams)) {
		data_size_pr_stream = enc_audio.size / enc_audio.num_ch;
	} else {
		LOG_ERR("Num enc channels is %d Must be 1 or num streams", enc_audio.num_ch);
		return -EINVAL;
	}

	if (data_size_pr_stream != LE_AUDIO_SDU_SIZE_OCTETS(CONFIG_LC3_BITRATE)) {
		LOG_ERR("The encoded data size does not match the SDU size");
		return -ECANCELED;
	}

	for (int i = 0; i < num_streams; i++) {
		if (audio_streams[i].ep->status.state != BT_BAP_EP_STATE_STREAMING) {
			LOG_DBG("Stream %d not in streaming state", i);
			continue;
		}

		if (is_iso_buffer_full(i)) {
			if (!wrn_printed[i]) {
				LOG_WRN("HCI ISO TX overrun on ch %d - Single print", i);
				wrn_printed[i] = true;
			}

			return -ENOMEM;
		}

		wrn_printed[i] = false;

		buf = net_buf_alloc(iso_tx_pools[i], K_NO_WAIT);
		if (buf == NULL) {
			/* This should never occur because of the is_iso_buffer_full() check */
			LOG_WRN("Out of TX buffers");
			return -ENOMEM;
		}

		net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);
		if (enc_audio.num_ch == 1) {
			net_buf_add_mem(buf, &enc_audio.data[0], data_size_pr_stream);
		} else {
			net_buf_add_mem(buf, &enc_audio.data[i * data_size_pr_stream],
					data_size_pr_stream);
		}

		atomic_inc(&iso_tx_pool_alloc[i]);

		ret = bt_bap_stream_send(&audio_streams[i], buf, seq_num[i]++,
					 BT_ISO_TIMESTAMP_NONE);
		if (ret < 0) {
			LOG_WRN("Failed to send audio data: %d", ret);
			net_buf_unref(buf);
			atomic_dec(&iso_tx_pool_alloc[i]);
			return ret;
		}
	}

	struct bt_iso_tx_info tx_info = { 0 };

	ret = bt_iso_chan_get_tx_sync(&audio_streams[0].ep->iso->chan, &tx_info);

	if (ret) {
		LOG_DBG("Error getting ISO TX anchor point: %d", ret);
	} else {
		timestamp_cb(tx_info.ts, false);
	}

	return 0;
}

#if CONFIG_BT_NUS_UART_ASYNC_ADAPTER
UART_ASYNC_ADAPTER_INST_DEFINE(async_adapter);
#else
static const struct device *const async_adapter;
#endif

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	ARG_UNUSED(dev);

	static size_t aborted_len;
	struct uart_data_t *buf;
	static uint8_t *aborted_buf;
	static bool disable_req;

	switch (evt->type) {
	case UART_TX_DONE:
		LOG_DBG("UART_TX_DONE");
		if ((evt->data.tx.len == 0) ||
		    (!evt->data.tx.buf)) {
			return;
		}

		if (aborted_buf) {
			buf = CONTAINER_OF(aborted_buf, struct uart_data_t,
					   data);
			aborted_buf = NULL;
			aborted_len = 0;
		} else {
			buf = CONTAINER_OF(evt->data.tx.buf, struct uart_data_t,
					   data);
		}

		k_free(buf);

		buf = k_fifo_get(&fifo_uart_tx_data, K_NO_WAIT);
		if (!buf) {
			return;
		}

		if (uart_tx(uart, buf->data, buf->len, SYS_FOREVER_MS)) {
			LOG_WRN("Failed to send data over UART");
		}

		break;

	case UART_RX_RDY:
		LOG_DBG("UART_RX_RDY");
		buf = CONTAINER_OF(evt->data.rx.buf, struct uart_data_t, data);
		buf->len += evt->data.rx.len;

		if (disable_req) {
			return;
		}

		if ((evt->data.rx.buf[buf->len - 1] == '\n') ||
		    (evt->data.rx.buf[buf->len - 1] == '\r')) {
			disable_req = true;
			uart_rx_disable(uart);
		}

		break;

	case UART_RX_DISABLED:
		LOG_DBG("UART_RX_DISABLED");
		disable_req = false;

		buf = k_malloc(sizeof(*buf));
		if (buf) {
			buf->len = 0;
		} else {
			LOG_WRN("Not able to allocate UART receive buffer");
			k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
			return;
		}

		uart_rx_enable(uart, buf->data, sizeof(buf->data),
			       UART_WAIT_FOR_RX);

		break;

	case UART_RX_BUF_REQUEST:
		LOG_DBG("UART_RX_BUF_REQUEST");
		buf = k_malloc(sizeof(*buf));
		if (buf) {
			buf->len = 0;
			uart_rx_buf_rsp(uart, buf->data, sizeof(buf->data));
		} else {
			LOG_WRN("Not able to allocate UART receive buffer");
		}

		break;

	case UART_RX_BUF_RELEASED:
		LOG_DBG("UART_RX_BUF_RELEASED");
		buf = CONTAINER_OF(evt->data.rx_buf.buf, struct uart_data_t,
				   data);

		if (buf->len > 0) {
			k_fifo_put(&fifo_uart_rx_data, buf);
		} else {
			k_free(buf);
		}

		break;

	case UART_TX_ABORTED:
		LOG_DBG("UART_TX_ABORTED");
		if (!aborted_buf) {
			aborted_buf = (uint8_t *)evt->data.tx.buf;
		}

		aborted_len += evt->data.tx.len;
		buf = CONTAINER_OF(aborted_buf, struct uart_data_t,
				   data);

		uart_tx(uart, &buf->data[aborted_len],
			buf->len - aborted_len, SYS_FOREVER_MS);

		break;

	default:
		break;
	}
}

static void uart_work_handler(struct k_work *item)
{
	struct uart_data_t *buf;

	buf = k_malloc(sizeof(*buf));
	if (buf) {
		buf->len = 0;
	} else {
		LOG_WRN("Not able to allocate UART receive buffer");
		k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
		return;
	}

	uart_rx_enable(uart, buf->data, sizeof(buf->data), UART_WAIT_FOR_RX);
}

static bool uart_test_async_api(const struct device *dev)
{
	const struct uart_driver_api *api =
			(const struct uart_driver_api *)dev->api;

	return (api->callback_set != NULL);
}

static int uart_init(void)
{
	int err;
	int pos;
	struct uart_data_t *rx;
	struct uart_data_t *tx;

	if (!device_is_ready(uart)) {
		return -ENODEV;
	}

	rx = k_malloc(sizeof(*rx));
	if (rx) {
		rx->len = 0;
	} else {
		return -ENOMEM;
	}

	k_work_init_delayable(&uart_work, uart_work_handler);


	if (IS_ENABLED(CONFIG_BT_NUS_UART_ASYNC_ADAPTER) && !uart_test_async_api(uart)) {
		/* Implement API adapter */
		uart_async_adapter_init(async_adapter, uart);
		uart = async_adapter;
	}

	err = uart_callback_set(uart, uart_cb, NULL);
	if (err) {
		k_free(rx);
		LOG_ERR("Cannot initialize UART callback");
		return err;
	}

	if (IS_ENABLED(CONFIG_UART_LINE_CTRL)) {
		LOG_INF("Wait for DTR");
		while (true) {
			uint32_t dtr = 0;

			uart_line_ctrl_get(uart, UART_LINE_CTRL_DTR, &dtr);
			if (dtr) {
				break;
			}
			/* Give CPU resources to low priority threads. */
			k_sleep(K_MSEC(100));
		}
		LOG_INF("DTR set");
		err = uart_line_ctrl_set(uart, UART_LINE_CTRL_DCD, 1);
		if (err) {
			LOG_WRN("Failed to set DCD, ret code %d", err);
		}
		err = uart_line_ctrl_set(uart, UART_LINE_CTRL_DSR, 1);
		if (err) {
			LOG_WRN("Failed to set DSR, ret code %d", err);
		}
	}

	tx = k_malloc(sizeof(*tx));

	if (tx) {
		pos = snprintf(tx->data, sizeof(tx->data),
			       "Starting Nordic UART service example\r\n");

		if ((pos < 0) || (pos >= sizeof(tx->data))) {
			k_free(rx);
			k_free(tx);
			LOG_ERR("snprintf returned %d", pos);
			return -ENOMEM;
		}

		tx->len = pos;
	} else {
		k_free(rx);
		return -ENOMEM;
	}

	err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
	if (err) {
		k_free(rx);
		k_free(tx);
		LOG_ERR("Cannot display welcome message (err: %d)", err);
		return err;
	}

	err = uart_rx_enable(uart, rx->data, sizeof(rx->data), 50);
	if (err) {
		LOG_ERR("Cannot enable uart reception (err: %d)", err);
		/* Free the rx buffer only because the tx buffer will be handled in the callback */
		k_free(rx);
	}

	return err;
}

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err) {
		LOG_INF("Security changed: %s level %u", addr, level);
	} else {
		LOG_WRN("Security failed: %s level %u err %d", addr,
			level, err);
	}
}
#endif

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
			  uint16_t len)
{
	int err;
	char addr[BT_ADDR_LE_STR_LEN] = {0};

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));

	LOG_INF("Received data from: %s", addr);

	for (uint16_t pos = 0; pos != len;) {
		struct uart_data_t *tx = k_malloc(sizeof(*tx));

		if (!tx) {
			LOG_WRN("Not able to allocate UART send data buffer");
			return;
		}

		/* Keep the last byte of TX buffer for potential LF char. */
		size_t tx_data_size = sizeof(tx->data) - 1;

		if ((len - pos) > tx_data_size) {
			tx->len = tx_data_size;
		} else {
			tx->len = (len - pos);
		}

		memcpy(tx->data, &data[pos], tx->len);

		pos += tx->len;

		/* Append the LF character when the CR character triggered
		 * transmission from the peer.
		 */
		if ((pos == len) && (data[len - 1] == '\r')) {
			tx->data[tx->len] = '\n';
			tx->len++;
		}

		err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
		if (err) {
			k_fifo_put(&fifo_uart_tx_data, tx);
		}
	}
}

static struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
};

int service_init(void)
{
	int err = 0;
	err = uart_init();

	if (err) {
		LOG_ERR("Failed to initialize UART driver (err: %d)", err);
		return 0;
	}

	LOG_INF("Bluetooth initialized");

	k_sem_give(&ble_init_ok);

	/*if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}*/

	err = bt_nus_init(&nus_cb);
	if (err) {
		LOG_ERR("Failed to initialize UART service (err: %d)", err);
		return 0;
	}
	return 0;

}

int le_audio_enable(le_audio_receive_cb recv_cb, le_audio_timestamp_cb timestmp_cb)
{
	int ret;

	ARG_UNUSED(recv_cb);

	LOG_INF("Starting broadcast gateway %s", CONFIG_BT_AUDIO_BROADCAST_NAME);

	ret = initialize(timestmp_cb);
	if (ret) {
		LOG_ERR("Failed to initialize");
		return ret;
	}
	service_init();
	advertising_start();

	/* Start extended advertising */
	ret = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
	if (ret) {
		LOG_ERR("Failed to start extended advertising: %d", ret);
		return ret;
	}

	/* Enable Periodic Advertising */
	ret = bt_le_per_adv_start(adv);
	if (ret) {
		LOG_ERR("Failed to enable periodic advertising: %d", ret);
		return ret;
	}
/*	ret = bt_le_ext_adv_start(adv_conn, BT_LE_EXT_ADV_START_DEFAULT);
	if (ret) {
		LOG_ERR("Failed to start connectable extended advertising: %d", ret);
		return ret;
	}
*/
	LOG_DBG("Starting broadcast source");

	ret = bt_bap_broadcast_source_start(broadcast_source, adv);
	if (ret) {
		return ret;
	}

	LOG_DBG("LE Audio enabled");

	return 0;
}

int le_audio_disable(void)
{
	int ret;

	if (audio_streams[0].ep->status.state == BT_BAP_EP_STATE_STREAMING) {
		/* Deleting broadcast source in stream_stopped_cb() */
		delete_broadcast_src = true;

		ret = bt_bap_broadcast_source_stop(broadcast_source);
		if (ret) {
			return ret;
		}
	} else if (broadcast_source != NULL) {
		ret = bt_bap_broadcast_source_delete(broadcast_source);
		if (ret) {
			return ret;
		}

		broadcast_source = NULL;
	}

	LOG_DBG("LE Audio disabled");

	return 0;
}

void ble_write_thread(void)
{
	/* Don't go any further until BLE is initialized */
	k_sem_take(&ble_init_ok, K_FOREVER);

	for (;;) {
		/* Wait indefinitely for data to be sent over bluetooth */
		struct uart_data_t *buf = k_fifo_get(&fifo_uart_rx_data,
						     K_FOREVER);

		if (bt_nus_send(NULL, buf->data, buf->len)) {
			LOG_WRN("Failed to send data over BLE connection");
		}

		k_free(buf);
	}
}

K_THREAD_DEFINE(ble_write_thread_id, STACKSIZE, ble_write_thread, NULL, NULL,
		NULL, PRIORITY, 0, 0);