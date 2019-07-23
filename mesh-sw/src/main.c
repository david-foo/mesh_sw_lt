/* main.c - Application main entry point */

/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <misc/printk.h>
#include <settings/settings.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/mesh.h>

#include <board.h>
#include <gpio.h>


// GPIO for the buttons
#define PIN_A SW0_GPIO_PIN
#define PIN_B SW1_GPIO_PIN
#define PORT SW0_GPIO_NAME
#define EDGE (GPIO_INT_EDGE | GPIO_INT_ACTIVE_LOW | GPIO_INT_DEBOUNCE)

static struct gpio_callback gpio_btnA_cb;
static struct gpio_callback gpio_btnB_cb;

// for use with k_work_submit which we use to handle button presses in a background thread to avoid holding onto an IRQ for too long
static struct k_work buttonA_work;
static struct k_work buttonB_work;

static struct bt_mesh_cfg_srv cfg_srv = {
	.relay = BT_MESH_RELAY_DISABLED,
	.beacon = BT_MESH_BEACON_ENABLED,
#if defined(CONFIG_BT_MESH_FRIEND)
	.frnd = BT_MESH_FRIEND_ENABLED,
#else
	.frnd = BT_MESH_FRIEND_NOT_SUPPORTED,
#endif
#if defined(CONFIG_BT_MESH_GATT_PROXY)
	.gatt_proxy = BT_MESH_GATT_PROXY_ENABLED,
#else
	.gatt_proxy = BT_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
	.default_ttl = 7,

	/* 3 transmissions with 20ms interval */
	.net_transmit = BT_MESH_TRANSMIT(2, 20),
	.relay_retransmit = BT_MESH_TRANSMIT(2, 20),
};

static struct bt_mesh_health_srv health_srv = {
};

BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0);

u8_t onoff[] = {
	0,
	1};

// handler functions for this model's RX messages

static void gen_onoff_status(struct bt_mesh_model *model,
														struct bt_mesh_msg_ctx *ctx,
														struct net_buf_simple *buf)
{
	u8_t	state;

	state = net_buf_simple_pull_u8(buf);

	printk("Node 0x%04x OnOff status from 0x%04x with state 0x%02x\n",
	       bt_mesh_model_elem(model)->addr, ctx->addr, state);
}

#define BT_MESH_MODEL_OP_GENERIC_ONOFF_GET	      BT_MESH_MODEL_OP_2(0x82, 0x01)
#define BT_MESH_MODEL_OP_GENERIC_ONOFF_SET	      BT_MESH_MODEL_OP_2(0x82, 0x02)
#define BT_MESH_MODEL_OP_GENERIC_ONOFF_SET_UNACK	BT_MESH_MODEL_OP_2(0x82, 0x03)
#define BT_MESH_MODEL_OP_GENERIC_ONOFF_STATUS   	BT_MESH_MODEL_OP_2(0x82, 0x04)

static const struct bt_mesh_model_op gen_onoff_cli_op[] = {
		{BT_MESH_MODEL_OP_GENERIC_ONOFF_STATUS, 1, gen_onoff_status},
		BT_MESH_MODEL_OP_END,
};

BT_MESH_MODEL_PUB_DEFINE(gen_onoff_pub_cli, NULL, 2 + 2);

static struct bt_mesh_model root_models[] = {
	BT_MESH_MODEL_CFG_SRV(&cfg_srv),
	//BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
	BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_CLI, gen_onoff_cli_op,
																		&gen_onoff_pub_cli, &onoff[0]),
};

static struct bt_mesh_elem elements[] = {
	BT_MESH_ELEM(0, root_models, BT_MESH_MODEL_NONE),
};

static const struct bt_mesh_comp comp = {
	.cid = BT_COMP_ID_LF,
	.elem = elements,
	.elem_count = ARRAY_SIZE(elements),
};

static int output_number(bt_mesh_output_action_t action, uint32_t number)
{
	printk("OOB Number: %u\n", number);

	return 0;
}

static u16_t primary_addr;
static u16_t primary_net_idx;



static void prov_complete(u16_t net_idx, u16_t addr)
{
	printk("provisioning complete for net_idx 0x%04x addr 0x%04x\n",
		net_idx, addr);
	primary_addr = addr;
	primary_net_idx = net_idx;
}

static void prov_reset(void)
{
	bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);
}

static const uint8_t dev_uuid[16] = { 0xdd, 0x04 };

static const struct bt_mesh_prov prov = {
	.uuid = dev_uuid,
	.output_size = 4,
	.output_actions = BT_MESH_DISPLAY_NUMBER,
	.output_number = output_number,
	.complete = prov_complete,
	.reset = prov_reset,
};

static void bt_ready(int err)
{
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	err = bt_mesh_init(&prov, &comp);
	if (err) {
		printk("Initializing mesh failed (err %d)\n", err);
		return;
	}

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	/* This will be a no-op if settings_load() loaded provisioning info */
	bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);

	printk("Mesh initialized\n");
}

void genericOnOffGet() 
{
  // 2 bytes for the opcode
	// 0 bytes parameters: 
	// 4 additional bytes for the TransMIC
	int err;
	struct bt_mesh_model *mod_cli;
	struct bt_mesh_model_pub *pub_cli;

	mod_cli = &root_models[1];
	pub_cli = mod_cli->pub;

	bt_mesh_model_msg_init(pub_cli->msg,
			       BT_MESH_MODEL_OP_GENERIC_ONOFF_GET);

	err = bt_mesh_model_publish(mod_cli);
	if (err) {
		printk("bt_mesh_model_publish err %d\n", err);
	}
}

static u8_t tid = 0;

int sendGenOnOffSet(u8_t on_or_off, u16_t message_type) 
{
  // 2 bytes for the opcode
	// 2 bytes parameters: first is the onoff value, second is for the TID
	// 4 additional bytes for the TransMIC
	int err;
	struct bt_mesh_model *mod_cli;
	struct bt_mesh_model_pub *pub_cli;

	mod_cli = &root_models[1];
	pub_cli = mod_cli->pub;

	bt_mesh_model_msg_init(pub_cli->msg,
			       message_type);
	net_buf_simple_add_u8(pub_cli->msg, on_or_off);
	net_buf_simple_add_u8(pub_cli->msg, tid++);
	err = bt_mesh_model_publish(mod_cli);
	if (err) {
		printk("bt_mesh_model_publish err %d\n", err);
	}
}

void genericOnOffSetUnAck(u8_t on_or_off) 
{
	if (sendGenOnOffSet(on_or_off, BT_MESH_MODEL_OP_GENERIC_ONOFF_SET_UNACK))
	{
		printk("Unable to send generic onoff set unack message\n");
	}
	printk("onoff set unack message %d sent\n",on_or_off);
}


void genericOnOffSet(u8_t on_or_off) 
{
	if (sendGenOnOffSet(on_or_off, BT_MESH_MODEL_OP_GENERIC_ONOFF_SET))
	{
		printk("Unable to send generic onoff set unack message\n");
	}
	printk("onoff set unack message %d sent\n",on_or_off);
}

void buttonA_work_handler(struct k_work *work)
{
	printk("Button A work handler\n");
  genericOnOffSetUnAck(onoff[1]);
	// genericOnOffSet(onoff[1]);
}

void buttonB_work_handler(struct k_work *work)
{
	printk("Button B work handler\n");
  //genericOnOffSetUnAck(onoff[0]);
	// genericOnOffSet(onoff[0]);
 	 genericOnOffGet();
}

void button_A_pressed(struct device *gpiob, struct gpio_callback *cb,
											u32_t pins)
{
	printk("Button A pressed at %d\n", k_cycle_get_32());
	k_work_submit(&buttonA_work);
}

void button_B_pressed(struct device *gpiob, struct gpio_callback *cb,
											u32_t pins)
{
	printk("Button B pressed at %d\n", k_cycle_get_32());
	k_work_submit(&buttonB_work);
}


void configureButtons(void)
{
	struct device *gpiob;

	printk("Press button A or button B\n");
	gpiob = device_get_binding(PORT);
	if (!gpiob)
	{
		printk("error\n");
		return;
	}

	// Button A
	k_work_init(&buttonA_work, buttonA_work_handler);
	gpio_pin_configure(gpiob, PIN_A, GPIO_DIR_IN | GPIO_INT | EDGE);
	gpio_init_callback(&gpio_btnA_cb, button_A_pressed, BIT(PIN_A));
	gpio_add_callback(gpiob, &gpio_btnA_cb);
	gpio_pin_enable_callback(gpiob, PIN_A);

	// Button B
	k_work_init(&buttonB_work, buttonB_work_handler);
	gpio_pin_configure(gpiob, PIN_B, GPIO_DIR_IN | GPIO_INT | EDGE);
	gpio_init_callback(&gpio_btnB_cb, button_B_pressed, BIT(PIN_B));
	gpio_add_callback(gpiob, &gpio_btnB_cb);
	gpio_pin_enable_callback(gpiob, PIN_B);
}

void main(void)
{
	int err;

	printk("switch\n");
	printk("Initializing...\n");
	configureButtons();

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(bt_ready);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
	}
}
