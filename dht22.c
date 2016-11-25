#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>

static int __init dht22_init(void) {
	pr_info("DHT22 module loaded\n");

	return 0;
}

static void __exit dht22_exit(void) {
	pr_info("DHT22 module unloaded\n");
}

module_init(dht22_init);
module_exit(dht22_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Filip Kolev");
MODULE_DESCRIPTION("A test module for the DHT22 sensor.");
MODULE_VERSION("0.1");
