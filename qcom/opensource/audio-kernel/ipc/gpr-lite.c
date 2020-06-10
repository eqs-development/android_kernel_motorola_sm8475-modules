/* Copyright (c) 2011-2017, 2019-2020 The Linux Foundation. All rights reserved.
 * Copyright (c) 2018, Linaro Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/idr.h>
#include <linux/slab.h>
#include <linux/of_device.h>
#include <ipc/gpr-lite.h>
#include <linux/rpmsg.h>
#include <linux/of.h>

struct gpr {
	struct rpmsg_endpoint *ch;
	struct device *dev;
	spinlock_t svcs_lock;
	struct idr svcs_idr;
	int dest_domain_id;
};

/**
 * gpr_send_pkt() - Send a gpr message from gpr device
 *
 * @adev: Pointer to previously registered gpr device.
 * @pkt: Pointer to gpr packet to send
 *
 * Return: Will be an negative on packet size on success.
 */
int gpr_send_pkt(struct gpr_device *adev, struct gpr_pkt *pkt)
{
	struct gpr *gpr = dev_get_drvdata(adev->dev.parent);
	struct gpr_hdr *hdr;
	unsigned long flags;
	uint32_t pkt_size;
	int ret;

	spin_lock_irqsave(&adev->lock, flags);

	hdr = &pkt->hdr;
	pkt_size = GPR_PKT_GET_PACKET_BYTE_SIZE(hdr->header);

	dev_dbg(gpr->dev, "SVC_ID %d %s packet size %d\n", adev->svc_id, __func__, pkt_size);
	ret = rpmsg_trysend(gpr->ch, pkt, pkt_size);
	spin_unlock_irqrestore(&adev->lock, flags);

	return ret ? ret : pkt_size;
}
EXPORT_SYMBOL_GPL(gpr_send_pkt);

static void gpr_dev_release(struct device *dev)
{
	struct gpr_device *adev = to_gpr_device(dev);

	kfree(adev);
}

static int gpr_callback(struct rpmsg_device *rpdev, void *buf,
				  int len, void *priv, u32 addr)
{
	struct gpr *gpr = dev_get_drvdata(&rpdev->dev);
	uint16_t hdr_size, pkt_size, svc_id;
	//uint16_t ver;
	struct gpr_device *svc = NULL;
	struct gpr_driver *adrv = NULL;
	struct gpr_hdr *hdr;
	unsigned long flags;
	//uint32_t opcode_type;

	if (len <= GPR_HDR_SIZE) {
		dev_err(gpr->dev, "GPR: Improper gpr pkt received:%p %d\n",
			buf, len);
		return -EINVAL;
	}

	hdr = buf;

	hdr_size = GPR_PKT_GET_HEADER_BYTE_SIZE(hdr->header);
	if (hdr_size < GPR_HDR_SIZE) {
		dev_err(gpr->dev, "GPR: Wrong hdr size:%d\n", hdr_size);
		return -EINVAL;
	}

	pkt_size = GPR_PKT_GET_PACKET_BYTE_SIZE(hdr->header);
	dev_dbg(gpr->dev,"Header %x", hdr->header);

	if (pkt_size < GPR_HDR_SIZE || pkt_size != len) {
		dev_err(gpr->dev, "GPR: Wrong packet size\n");
		return -EINVAL;
	}

	dev_dbg(gpr->dev, "%s: dst_port %x hdr_size %d pkt_size %d\n",__func__ , hdr->dst_port, hdr_size, pkt_size);

	svc_id = hdr->dst_port;
	spin_lock_irqsave(&gpr->svcs_lock, flags);
	svc = idr_find(&gpr->svcs_idr, svc_id);
	if (svc && svc->dev.driver) {
		adrv = to_gpr_driver(svc->dev.driver);
	} else {
		/*Does not match any SVC ID hence would be routed to audio passthrough*/
		svc = idr_find(&gpr->svcs_idr, GPR_SVC_MAX);
		if (svc && svc->dev.driver)
			adrv = to_gpr_driver(svc->dev.driver);
	}
	spin_unlock_irqrestore(&gpr->svcs_lock, flags);

	if (!adrv) {
		dev_err(gpr->dev, "GPR: service is not registered\n");
		return -EINVAL;
	}

	/*
	 * NOTE: hdr_size is not same as GPR_HDR_SIZE as remote can include
	 * optional headers in to gpr_hdr which should be ignored
	 */

	adrv->callback(svc, buf);

	return 0;
}

static int gpr_device_match(struct device *dev, struct device_driver *drv)
{
	struct gpr_device *adev = to_gpr_device(dev);
	struct gpr_driver *adrv = to_gpr_driver(drv);
	const struct gpr_device_id *id = adrv->id_table;

	/* Attempt an OF style match first */
	if (of_driver_match_device(dev, drv))
		return 1;

	if (!id)
		return 0;

	while (id->domain_id != 0 || id->svc_id != 0) {
		if (id->domain_id == adev->domain_id &&
		    id->svc_id == adev->svc_id)
			return 1;
		id++;
	}

	return 0;
}

static int gpr_device_probe(struct device *dev)
{
	struct gpr_device *adev = to_gpr_device(dev);
	struct gpr_driver *adrv = to_gpr_driver(dev->driver);

	return adrv->probe(adev);
}

static int gpr_device_remove(struct device *dev)
{
	struct gpr_device *adev = to_gpr_device(dev);
	struct gpr_driver *adrv;
	struct gpr *gpr = dev_get_drvdata(adev->dev.parent);

	if (dev->driver) {
		adrv = to_gpr_driver(dev->driver);
		if (adrv->remove)
			adrv->remove(adev);
		spin_lock(&gpr->svcs_lock);
		idr_remove(&gpr->svcs_idr, adev->svc_id);
		spin_unlock(&gpr->svcs_lock);
	}

	return 0;
}

static int gpr_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	struct gpr_device *adev = to_gpr_device(dev);
	int ret;

	ret = of_device_uevent_modalias(dev, env);
	if (ret != -ENODEV)
		return ret;

	return add_uevent_var(env, "MODALIAS=gpr:%s", adev->name);
}

struct bus_type gprbus = {
	.name		= "gprbus",
	.match		= gpr_device_match,
	.probe		= gpr_device_probe,
	.uevent		= gpr_uevent,
	.remove		= gpr_device_remove,
};
EXPORT_SYMBOL_GPL(gprbus);

static int gpr_add_device(struct device *dev, struct device_node *np,
			  const struct gpr_device_id *id)
{
	struct gpr *gpr = dev_get_drvdata(dev);
	struct gpr_device *adev = NULL;
	int ret;

	adev = kzalloc(sizeof(*adev), GFP_KERNEL);
	if (!adev)
		return -ENOMEM;

	spin_lock_init(&adev->lock);

	adev->svc_id = id->svc_id;
	adev->domain_id = id->domain_id;
	adev->version = id->svc_version;
	if (np)
		strscpy(adev->name, np->name, GPR_NAME_SIZE);
	else
		strscpy(adev->name, id->name, GPR_NAME_SIZE);

	dev_set_name(&adev->dev, "gprsvc:%s:%x:%x", adev->name,
		     id->domain_id, id->svc_id);

	adev->dev.bus = &gprbus;
	adev->dev.parent = dev;
	adev->dev.of_node = np;
	adev->dev.release = gpr_dev_release;
	adev->dev.driver = NULL;

	spin_lock(&gpr->svcs_lock);
	idr_alloc(&gpr->svcs_idr, adev, id->svc_id,
		  id->svc_id + 1, GFP_ATOMIC);
	spin_unlock(&gpr->svcs_lock);

	dev_info(dev, "Adding GPR dev: %s\n", dev_name(&adev->dev));

	ret = device_register(&adev->dev);
	if (ret) {
		dev_err(dev, "device_register failed: %d\n", ret);
		put_device(&adev->dev);
	}

	return ret;
}

static void of_register_gpr_devices(struct device *dev)
{
	struct gpr *gpr = dev_get_drvdata(dev);
	struct device_node *node;

	for_each_child_of_node(dev->of_node, node) {
		struct gpr_device_id id = { {0} };

		if (of_property_read_u32(node, "reg", &id.svc_id))
			continue;

		id.domain_id = gpr->dest_domain_id;

		if (gpr_add_device(dev, node, &id))
			dev_err(dev, "Failed to add gpr %d svc\n", id.svc_id);
	}
}

static int gpr_probe(struct rpmsg_device *rpdev)
{
	struct device *dev = &rpdev->dev;
	struct gpr *gpr;
	int ret;

	gpr = devm_kzalloc(dev, sizeof(*gpr), GFP_KERNEL);
	if (!gpr)
		return -ENOMEM;

	ret = of_property_read_u32(dev->of_node, "reg", &gpr->dest_domain_id);
	if (ret) {
		dev_err(dev, "GPR Domain ID not specified in DT\n");
		return ret;
	}

	dev_set_drvdata(dev, gpr);
	gpr->ch = rpdev->ept;
	gpr->dev = dev;
	spin_lock_init(&gpr->svcs_lock);
	idr_init(&gpr->svcs_idr);
	of_register_gpr_devices(dev);

	return 0;
}

static int gpr_remove_device(struct device *dev, void *null)
{
	struct gpr_device *adev = to_gpr_device(dev);

	device_unregister(&adev->dev);

	return 0;
}

static void gpr_remove(struct rpmsg_device *rpdev)
{
	device_for_each_child(&rpdev->dev, NULL, gpr_remove_device);
}

/*
 * __gpr_driver_register() - Client driver registration with gprbus
 *
 * @drv:Client driver to be associated with client-device.
 * @owner: owning module/driver
 *
 * This API will register the client driver with the gprbus
 * It is called from the driver's module-init function.
 */
int __gpr_driver_register(struct gpr_driver *drv, struct module *owner)
{
	drv->driver.bus = &gprbus;
	drv->driver.owner = owner;

	return driver_register(&drv->driver);
}
EXPORT_SYMBOL_GPL(__gpr_driver_register);

/*
 * gpr_driver_unregister() - Undo effect of gpr_driver_register
 *
 * @drv: Client driver to be unregistered
 */
void gpr_driver_unregister(struct gpr_driver *drv)
{
	driver_unregister(&drv->driver);
}
EXPORT_SYMBOL_GPL(gpr_driver_unregister);

static const struct of_device_id gpr_of_match[] = {
	{ .compatible = "qcom,gpr"},
	{}
};
MODULE_DEVICE_TABLE(of, gpr_of_match);

static struct rpmsg_driver gpr_driver = {
	.probe = gpr_probe,
	.remove = gpr_remove,
	.callback = gpr_callback,
	.drv = {
		.name = "qcom,gpr",
		.of_match_table = gpr_of_match,
	},
};

static int __init gpr_init(void)
{
	int ret;

	ret = bus_register(&gprbus);
	if (!ret)
		ret = register_rpmsg_driver(&gpr_driver);
	else
		bus_unregister(&gprbus);

	return ret;
}

static void __exit gpr_exit(void)
{
	bus_unregister(&gprbus);
	unregister_rpmsg_driver(&gpr_driver);
}

subsys_initcall(gpr_init);
module_exit(gpr_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("QTI GPR Bus");
