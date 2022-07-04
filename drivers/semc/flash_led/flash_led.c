/*
 * Copyright 2021 Sony Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kobject.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/log2.h>
#include <linux/sysfs.h>
#include <linux/leds.h>
#include <linux/printk.h>

static struct led_classdev led_cdev;

DEFINE_LED_TRIGGER(ledtrig);

static ssize_t flash_led_oneshot_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long state;
	unsigned long on = 0;
	unsigned long off = 0;

	ret = kstrtoul(buf, 10, &state);
	if (ret)
		return ret;
	on = state;
	led_trigger_blink_oneshot(ledtrig, &on, &off, 0);

	return count;
}

static DEVICE_ATTR(oneshot, (S_IRUGO | S_IWUSR | S_IWGRP),
	NULL, flash_led_oneshot_store);

static struct attribute *flash_led_attrs[] = {
	&dev_attr_oneshot.attr,
	NULL
};

static struct attribute_group flash_led_attr_group = {
	.attrs = flash_led_attrs,
};

static int flash_led_init(void)
{
	int ret;

	led_trigger_register_simple("switch0_trigger", &ledtrig);
	led_cdev.name = "flash_led";
	ret = led_classdev_register(NULL, &led_cdev);
	if (ret <0) {
		pr_err("Unalbe to register led node\n");
		return ret;
	}
	ret = sysfs_create_group(&led_cdev.dev->kobj, &flash_led_attr_group);
	if (ret)
		led_classdev_unregister(&led_cdev);

	return ret;
}

static void flash_led_exit(void)
{
	led_classdev_unregister(&led_cdev);
}

module_init(flash_led_init);
module_exit(flash_led_exit);

MODULE_DESCRIPTION("flash led driver");
MODULE_LICENSE("GPL v2");
