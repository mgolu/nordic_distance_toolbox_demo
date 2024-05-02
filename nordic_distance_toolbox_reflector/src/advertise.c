#include "advertise.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <bluetooth/gatt_dm.h>
#include <bluetooth/scan.h>

#include <zephyr/random/rand32.h>

#include <dm.h>

LOG_MODULE_REGISTER(advertise, LOG_LEVEL_DBG);

#define DEVICE_NAME             "Thingy" 
#define DEVICE_NAME_LEN         (sizeof(DEVICE_NAME) - 1)

#define COMPANY_CODE 0x0059
#define SUPPORT_MCPD_CODE 0x0D17A9CE
#define SUPPORT_RTT_CODE 0x1D17A9CE

static uint64_t bt_addr_to_int(const bt_addr_le_t *addr) {
    uint64_t addr_int = 0;
    for (int i = 0; i < BT_ADDR_SIZE; i++) {
        addr_int = addr_int << 8;
        addr_int += addr->a.val[i];
    }
    return addr_int;
}

static struct bt_le_ext_adv *adv;

struct adv_mfg_data {
	uint16_t company_code;	    /* Company Identifier Code. */
	uint32_t support_dm_code;   /* To identify the device that supports distance measurement. */
	uint32_t rng_seed;          /* Random seed used for generating hopping patterns. */
} __packed;

static struct adv_mfg_data mfg_data;

struct bt_le_adv_param adv_param_noconn =
	BT_LE_ADV_PARAM_INIT(BT_LE_ADV_OPT_USE_IDENTITY |
				BT_LE_ADV_OPT_SCANNABLE |
				BT_LE_ADV_OPT_NOTIFY_SCAN_REQ |
				BT_LE_ADV_OPT_CONNECTABLE,
				BT_GAP_ADV_FAST_INT_MIN_2,
				BT_GAP_ADV_FAST_INT_MAX_2,
				NULL);


struct bt_le_adv_param *adv_param = &adv_param_noconn;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, (unsigned char *)&mfg_data, sizeof(mfg_data)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

#define BT_UUID_NULL BT_UUID_128_ENCODE(0, 0, 0, 0, 0)

static struct bt_data sd[] = {
	BT_DATA(BT_DATA_MANUFACTURER_DATA, (unsigned char *)&mfg_data, sizeof(mfg_data)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NULL),
};


#define PEER_MAX                8   /* Maximum number of tracked peer devices. */
struct peer_entry {
	sys_snode_t node;
	bt_addr_le_t bt_addr;
	struct dm_result result;
};

static K_HEAP_DEFINE(peer_heap, PEER_MAX * sizeof(struct peer_entry));
static sys_slist_t peer_list = SYS_SLIST_STATIC_INIT(&peer_list);

static K_MUTEX_DEFINE(list_mtx);

static void list_lock(void)
{
	k_mutex_lock(&list_mtx, K_FOREVER);
}

static void list_unlock(void)
{
	k_mutex_unlock(&list_mtx);
}

bool peer_supported_test(const bt_addr_le_t *peer)
{
	sys_snode_t *node, *tmp;
	struct peer_entry *item;

	SYS_SLIST_FOR_EACH_NODE_SAFE(&peer_list, node, tmp) {
		item = CONTAINER_OF(node, struct peer_entry, node);
		if (bt_addr_le_cmp(&item->bt_addr, peer) == 0) {
			return true;
		}
	}

	return false;
}

int peer_supported_add(const bt_addr_le_t *peer)
{
	struct peer_entry *item;

	if (peer_supported_test(peer)) {
		return 0;
	}

	item = k_heap_alloc(&peer_heap, sizeof(struct peer_entry), K_NO_WAIT);
	if (!item) {
		return -ENOMEM;
	}

	bt_addr_le_copy(&item->bt_addr, peer);
	list_lock();
	sys_slist_append(&peer_list, &item->node);
	list_unlock();

	return 0;
}

static uint32_t timestamp = 0;

static void adv_scanned_cb(struct bt_le_ext_adv *adv, struct bt_le_ext_adv_scanned_info *info) {
    struct dm_request req;

	if (peer_supported_test(info->addr)) {
		uint32_t current_time = k_uptime_get();
    	if (timestamp + 1000 < current_time) {
        	timestamp = current_time;
    	}
    	else {
        	return;
    	}
		req.role = DM_ROLE_REFLECTOR;
		#ifdef CONFIG_MCPD_DISTANCE
		req.ranging_mode = DM_RANGING_MODE_MCPD;
		#endif

		#ifdef CONFIG_RTT_DISTANCE
		req.ranging_mode = DM_RANGING_MODE_RTT;
		#endif

		/* We need to make sure that we only initiate a ranging to a single peer.
			* A scan response from this device can be received by multiple peers which can
			* all start a ranging at the same time as a consequence. To prevent this,
			* we need to make sure that we set a per-peer random as the random seed.
			* This helps the ranging library to avoid interference from other devices
			* trying to range at the same time.
			*
			* This means that the initiator and the reflector need to set the same value
			* for the random seed.
			*/
		bt_addr_le_copy(&req.bt_addr, info->addr);
		req.rng_seed = mfg_data.rng_seed;
		req.start_delay_us = 0;
		req.extra_window_time_us = 0;

		LOG_INF("Going to start ranging");

		dm_request_add(&req);
	}
}

static void connected(struct bt_conn *conn, uint8_t err) {
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		LOG_ERR("Connection failed (err %u)\n", err);
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected %s\n", addr);
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
	LOG_INF("Disconnected (reason %u)", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected, };
const static struct bt_le_ext_adv_cb adv_cb = {
	.scanned = adv_scanned_cb
};


int advertise_init(void) {
    int err;

	int uuid[16] = {0};
	sys_csrand_get(uuid, 16);

	sd[1].data = uuid;
	sd[1].data_len = 16;
	sd[1].type = BT_DATA_UUID128_ALL;

    mfg_data.company_code = COMPANY_CODE;
	#ifdef CONFIG_MCPD_DISTANCE
    mfg_data.support_dm_code = SUPPORT_MCPD_CODE;
	#endif

	#ifdef CONFIG_RTT_DISTANCE
	mfg_data.support_dm_code = SUPPORT_RTT_CODE;
	#endif

	sys_csrand_get(&mfg_data.rng_seed, sizeof(mfg_data.rng_seed));

    struct bt_le_ext_adv_start_param ext_adv_start_param = {0};

    if (adv) {
		err = bt_le_ext_adv_stop(adv);
		if (err) {
			LOG_ERR("Failed to stop extended advertising  (err %d)\n", err);
			return err;
		}
			err = bt_le_ext_adv_delete(adv);
		if (err) {
			LOG_ERR("Failed to delete advertising set  (err %d)\n", err);
			return err;
		}
	}

	err = bt_le_ext_adv_create(adv_param, &adv_cb, &adv);
	if (err) {
		LOG_ERR("Failed to create advertising set (err %d)\n", err);
		return err;
	}

	err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Failed setting adv data (err %d)\n", err);
		return err;
	}

	err = bt_le_filter_accept_list_clear();
	if (err) {
		LOG_ERR("Failed to clear accept list (err %d)\n", err);
	}

	err = bt_le_ext_adv_start(adv, &ext_adv_start_param);
	if (err) {
		LOG_ERR("Failed to start extended advertising  (err %d)\n", err);
		return err;
	}

	return err;
}

static struct bt_scan_manufacturer_data scan_mfg_data = {
	.data = (unsigned char *)&mfg_data,
	.data_len = sizeof(mfg_data.company_code) + sizeof(mfg_data.support_dm_code),
};

static struct bt_le_scan_param scan_param = {
	.type     = BT_LE_SCAN_TYPE_ACTIVE,
	.interval = BT_GAP_SCAN_FAST_INTERVAL,
	.window   = BT_GAP_SCAN_FAST_WINDOW,
	.options  = BT_LE_SCAN_OPT_NONE,
	.timeout  = 0,
};

static struct bt_scan_init_param scan_init = {
	.connect_if_match = 0,
	.scan_param = &scan_param,
	.conn_param = NULL
};

static void scan_filter_match(struct bt_scan_device_info *device_info,
			      struct bt_scan_filter_match *filter_match,
			      bool connectable)
{
	bt_addr_le_t addr;

	bt_addr_le_copy(&addr, device_info->recv_info->addr);
	peer_supported_add(device_info->recv_info->addr);
}

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, NULL, NULL, NULL);

int scan_start(void)
{
	int err;

	bt_scan_init(&scan_init);
	bt_scan_cb_register(&scan_cb);

	err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_MANUFACTURER_DATA, &scan_mfg_data);
	if (err) {
		printk("Scanning filters cannot be set (err %d)\n", err);
		return err;
	}

	err = bt_scan_filter_enable(BT_SCAN_MANUFACTURER_DATA_FILTER, false);
	if (err) {
		printk("Filters cannot be turned on (err %d)\n", err);
		return err;
	}

	err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
	if (err) {
		printk("Scanning failed to start (err %d)\n", err);
		return err;
	}

	return err;
}
