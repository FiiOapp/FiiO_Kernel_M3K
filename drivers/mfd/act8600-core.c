/*
 * linux/drivers/mfd/act8600-core.c - mfd core driver PMU ACT8600
 *
 * Copyright (C) 2012 Ingenic Semiconductor Co., Ltd.
 * Written by Large Dipper <ykli@ingenic.com>.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/mfd/core.h>
#include <linux/mfd/act8600-private.h>

int act8600_read_reg(struct i2c_client *client,
		     unsigned char reg, unsigned char *val)
{
	i2c_master_send(client, &reg, 1);

	return i2c_master_recv(client, val, 1);
}
EXPORT_SYMBOL_GPL(act8600_read_reg);

int act8600_write_reg(struct i2c_client *client,
		      unsigned char reg, unsigned char val)
{
	unsigned char msg[2];

	memcpy(&msg[0], &reg, 1);
	memcpy(&msg[1], &val, 1);

	return i2c_master_send(client, msg, 2);
}
EXPORT_SYMBOL_GPL(act8600_write_reg);

static struct mfd_cell act8600_cells[] = {
	{
		.id = 0,
		.name = "act8600-regulator",
	},
	{
		.id = 1,
		.name = "act8600-charger",
	},
};

static int act8600_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	struct act8600 *act8600;
	int ret = 0;

	act8600 = kzalloc(sizeof(struct act8600), GFP_KERNEL);
	if (act8600 == NULL) {
		dev_err(&client->dev, "device alloc error\n");
		return -ENOMEM;
	}

	act8600->client = client;
	act8600->dev = &client->dev;
	i2c_set_clientdata(client, act8600);

	ret = mfd_add_devices(act8600->dev, -1,
			      act8600_cells, ARRAY_SIZE(act8600_cells),
			      NULL, 0, NULL);

	return 0;
}

static int act8600_remove(struct i2c_client *client)
{
	struct act8600 *act8600 = i2c_get_clientdata(client);

	mfd_remove_devices(act8600->dev);
	kfree(act8600);

	return 0;
}

static const struct i2c_device_id act8600_id[] = {
	{"act8600", 0 },
	{}
};

static struct i2c_driver act8600_pmu_driver = {
	.probe		= act8600_probe,
	.remove		= act8600_remove,
	.id_table	= act8600_id,
	.driver = {
		.name	= "act8600",
		.owner	= THIS_MODULE,
	},
};

static int act8600_pmu_init(void)
{
	return i2c_add_driver(&act8600_pmu_driver);
}

static void __exit act8600_pmu_exit(void)
{
	i2c_del_driver(&act8600_pmu_driver);
}

subsys_initcall_sync(act8600_pmu_init);
module_exit(act8600_pmu_exit);

MODULE_DESCRIPTION("act8600 PMU mfd Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Large Dipper <ykli@ingenic.cn>");
