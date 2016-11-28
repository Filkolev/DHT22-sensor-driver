#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/time64.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/slab.h>

/* 
 * TODO: Implement the following:
 * - timer/s to check whether the expected interrupt took too long;
 * - queue work when autoupdate is enabled;
 */

#define GPIO_DEFAULT 24
#define AUTOUPDATE_DEFAULT false
#define AUTOUPDATE_TIMEOUT_MIN 2000 /* 2 sec minimum between readings */
#define DATA_SIZE 5 /* Number of bytes the DHT22 sensor sends */
#define BITS_PER_BYTE 8

#define LOW 0
#define HIGH 1

static int irq_count = 0; // TODO: remove

static void trigger_sensor(struct work_struct *work);

/* TODO: Move all sm-related stuff in separate header and source file? */
static void change_sm_state(void);
static irqreturn_t dht22_irq_handler(int irq, void *data);

enum dht22_state get_next_state_idle(void);
enum dht22_state get_next_state_triggered(void);
enum dht22_state get_next_state_responding_low(void);
enum dht22_state get_next_state_responding_high(void);
enum dht22_state get_next_state_sending_data_prepare(void);
enum dht22_state get_next_state_sending_data_value(void);
enum dht22_state get_next_state_error(void);

enum dht22_state {
	IDLE = 0,
	TRIGGERED,
	RESPONDING_LOW,
	RESPONDING_HIGH,
	SENDING_DATA_PREPARE,
	SENDING_DATA_VALUE,
	ERROR,
	COUNT_STATES
};

struct dht22_sm {
	enum dht22_state state;
	enum dht22_state (*get_next_state)(void);
};

static struct dht22_sm *sm;
static enum dht22_state (*state_functions[COUNT_STATES])(void) = {
	get_next_state_idle,
	get_next_state_triggered,
	get_next_state_responding_low,
	get_next_state_responding_high,
	get_next_state_sending_data_prepare,
	get_next_state_sending_data_value,
	get_next_state_error
};

/* Container for the data received by the DHT22 */
static int sensor_data[DATA_SIZE];

/* Keeping track of the previous state of the gpio for each interrupt */
static int prev_gpio_state;
static struct timespec64 ts_prev_gpio_switch, ts_prev_reading;

static int gpio = GPIO_DEFAULT;
module_param(gpio, int, S_IRUGO);
MODULE_PARM_DESC(gpio, "GPIO number of the DHT22's data pin (default = 24)");

static int irq_number;

/* TODO: module params: autoupdate, autoupdate_timeout */

static int __init dht22_init(void)
{
	int ret;

	pr_info("DHT22 module loading...\n");

	ret = 0;
	if (!gpio_is_valid(gpio)) {
		pr_err("Failed validation of GPIO %d\n", gpio);
		ret = -EINVAL;
		goto out;
	}  
	
	pr_info("Validation succeeded for GPIO %d\n", gpio);

	ret = gpio_request(gpio, "sysfs");
	if (ret < 0) {
		pr_err("GPIO request failed. Exiting.\n");
		goto out;
	}

	gpio_direction_input(gpio);
	pr_info("gpio initial value: %d\n", gpio_get_value(gpio));
//	gpio_export(gpio, true);

	irq_number = gpio_to_irq(gpio);
	if (irq_number < 0) {
		pr_err("Failed to retrieve IRQ number for GPIO. Exiting.\n");
		ret = irq_number;
		goto irq_err;
	}
	
	pr_info("Assigned IRQ number %d\n", irq_number);

	ret = request_irq(irq_number,
			dht22_irq_handler,
			(IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING),
			"dht22_gpio_handler",
			NULL);
	if (ret < 0) {
		pr_err("request_irq() failed. Exiting.\n");
		goto irq_req_err;
	}

	prev_gpio_state = HIGH;
	getnstimeofday64(&ts_prev_gpio_switch);	
	trigger_sensor(NULL);

	pr_info("DHT22 module finished loading.\n");

irq_req_err:
	free_irq(irq_number, NULL);
irq_err:
//	gpio_unexport(gpio);
	gpio_free(gpio);

out:
	return ret;
}

static void __exit dht22_exit(void) {
	pr_info("DHT22 module unloaded\n");
}

static void trigger_sensor(struct work_struct *work) {
	/* 
	 * prepare: 250 ms HIGH
	 * send start signal: 20 ms LOW
	 * send end start signal: 40 us HIGH
	 * delay: 10 us in library (maybe unnecessary)
	 */
	pr_info("Triggering DHT22 sensor.\n");

	mdelay(250);

	gpio_direction_output(gpio, LOW);
	mdelay(20);

	gpio_direction_input(gpio);
	udelay(50);
}

static void change_sm_state(void) {
	sm->state = sm->get_next_state();
	sm->get_next_state = state_functions[sm->state];
}

static irqreturn_t dht22_irq_handler(int irq, void *data) {
	int current_gpio_state;
	struct timespec64 ts_current_gpio_switch, ts_gpio_switch_diff;

	irq_count++; // TODO: remove
	current_gpio_state = !prev_gpio_state;
	getnstimeofday64(&ts_current_gpio_switch);
	ts_gpio_switch_diff = timespec64_sub(ts_current_gpio_switch,
						ts_prev_gpio_switch);

	/* TODO: schedule processing in bottom half */
	pr_info("irq #%02d; gpio state: %d; time delta: %ld usec (%ld msec)",
		irq_count,
		current_gpio_state,
		ts_gpio_switch_diff.tv_nsec / NSEC_PER_USEC,
		ts_gpio_switch_diff.tv_nsec / NSEC_PER_MSEC);

	prev_gpio_state = current_gpio_state;
	ts_prev_gpio_switch = ts_current_gpio_switch;

	return IRQ_HANDLED;
}

module_init(dht22_init);
module_exit(dht22_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Filip Kolev");
MODULE_DESCRIPTION("A test module for the DHT22 sensor.");
MODULE_VERSION("0.1");
