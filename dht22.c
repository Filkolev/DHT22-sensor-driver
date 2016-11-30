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
 * - export to sysfs
 */

#define GPIO_DEFAULT 6
#define AUTOUPDATE_DEFAULT false
#define AUTOUPDATE_TIMEOUT_MIN 2000 /* 2 sec minimum between readings */
#define DATA_SIZE 5 /* Number of bytes the DHT22 sensor sends */
#define BITS_PER_BYTE 8
#define EXPECTED_IRQ_COUNT 86
#define TRIGGER_IRQ_COUNT 3
#define INIT_RESPONSE_IRQ_COUNT 2

#define LOW 0
#define HIGH 1

/* signal length in ms */
#define TRIGGER_DELAY 250
#define TRIGGER_SIGNAL_LEN 20

/* signal lengths in us */
#define TRIGGER_POST_DELAY 40
#define INIT_RESPONSE_LEN 80
#define PREP_SIGNAL_LEN 50
#define ZERO_BIT 28
#define ONE_BIT 75
#define TOLERANCE 15 /* Each irq delta should deviate +/- 15us at most */

static void trigger_sensor(void);
static void reset_data(void);

static bool finished = false;
static bool triggered = false;
static irqreturn_t dht22_irq_handler(int irq, void *data);
static void process_results(struct work_struct *work);
static void process_data(void);

enum dht22_state get_next_state_idle(void);
enum dht22_state get_next_state_responding(void);
enum dht22_state get_next_state_finished(void);
enum dht22_state get_next_state_error(void);

enum dht22_state {
	IDLE = 0,
	RESPONDING,
	FINISHED,
	ERROR,
	COUNT_STATES
};

static enum dht22_state (*state_functions[COUNT_STATES])(void) = {
	get_next_state_idle,
	get_next_state_responding,
	get_next_state_finished,
	get_next_state_error
};

static enum dht22_state sm_state = IDLE;
static struct timespec64 ts_prev_gpio_switch, ts_prev_reading;
static int irq_number;
static int processed_irq_count = 0;

static int irq_deltas[EXPECTED_IRQ_COUNT];
static int sensor_data[DATA_SIZE];

static DECLARE_WORK(work, process_results);

static int gpio = GPIO_DEFAULT;
module_param(gpio, int, S_IRUGO);
MODULE_PARM_DESC(gpio, "GPIO number of the DHT22's data pin (default = 24)");

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

	getnstimeofday64(&ts_prev_gpio_switch);	
	gpio_direction_input(gpio);
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
	
	reset_data();
	trigger_sensor();

	pr_info("DHT22 module finished loading.\n");
	goto out;

irq_req_err:
	free_irq(irq_number, NULL);
irq_err:
//	gpio_unexport(gpio);
	gpio_free(gpio);
out:
	return ret;
}

static void __exit dht22_exit(void)
{
	free_irq(irq_number, NULL);
//	gpio_unexport(gpio);
	gpio_free(gpio);

	pr_info("DHT22 module unloaded\n");
}

static void trigger_sensor(void)
{
	/* 
	 * prepare: 250 ms HIGH
	 * send start signal: 20 ms LOW
	 * send end start signal: 40 us HIGH
	 * delay: 10 us in library (maybe unnecessary)
	 */
	pr_info("Triggering DHT22 sensor.\n");

	triggered = true;
	sm_state = (*state_functions[sm_state])();

	mdelay(TRIGGER_DELAY);
	gpio_direction_output(gpio, LOW);
	mdelay(TRIGGER_SIGNAL_LEN);

	gpio_direction_input(gpio);
	udelay(TRIGGER_POST_DELAY);
}

static void reset_data(void)
{
	int i;

	for (i = 0; i < DATA_SIZE; i++)
		sensor_data[i] = 0;

	for (i = 0; i < EXPECTED_IRQ_COUNT; i++)
		irq_deltas[i] = 0;

	processed_irq_count = 0;	
	finished = false;
	triggered = false;
}

static irqreturn_t dht22_irq_handler(int irq, void *data)
{
	struct timespec64 ts_current_gpio_switch, ts_gpio_switch_diff;

	if (processed_irq_count >= EXPECTED_IRQ_COUNT) {
		sm_state = ERROR;
		reset_data(); /* TODO: move in separate work, and into the err state func */
		return IRQ_HANDLED;
	}

	getnstimeofday64(&ts_current_gpio_switch);
	ts_gpio_switch_diff = timespec64_sub(ts_current_gpio_switch,
						ts_prev_gpio_switch);

	irq_deltas[processed_irq_count++] = (int)(ts_gpio_switch_diff.tv_nsec / NSEC_PER_USEC); 
	ts_prev_gpio_switch = ts_current_gpio_switch;
	
	if (processed_irq_count == EXPECTED_IRQ_COUNT) {
		sm_state = FINISHED;
		schedule_work(&work);
	}

	sm_state = (*state_functions[sm_state])();
	return IRQ_HANDLED;
}

enum dht22_state get_next_state_idle(void)
{
	if (triggered)
		return RESPONDING;

	return IDLE;
}

enum dht22_state get_next_state_responding(void)
{
	if (finished)
		return FINISHED;

	return RESPONDING;
}

enum dht22_state get_next_state_finished(void)
{
	return IDLE;
}

enum dht22_state get_next_state_error(void)
{
	return IDLE;
}

static void process_data(void)
{
	int i, bit_value, current_byte, current_bit;

	for (i = 6; i < 6 + 80; i += 2) {
		bit_value = irq_deltas[i] < PREP_SIGNAL_LEN ? 0 : 1;
		current_byte = (i - 6) / (2 * BITS_PER_BYTE);
		current_bit = 7 - (((i - 6) % (2 * BITS_PER_BYTE)) / 2);
		sensor_data[current_byte] |= bit_value << current_bit;
	}
}

static void process_results(struct work_struct *work)
{
	int hash, temperature, humidity;

	process_data();

	hash = (sensor_data[0] + sensor_data[1] + sensor_data[2] + sensor_data[3]) & 0xFF;
	if (hash != sensor_data[4]) {
		pr_err("Hash mismatch. Stopping. (%d, %d, %d, %d, %d)\n",
				sensor_data[0], sensor_data[1], sensor_data[2], sensor_data[3], sensor_data[4]);
		sm_state = ERROR;
		return;
	}

	humidity = ((sensor_data[0] << BITS_PER_BYTE) | sensor_data[1]);
	temperature = ((sensor_data[2] << BITS_PER_BYTE) | sensor_data[3]);
	if (sensor_data[2] & 0x80)
		temperature *= -1;

	pr_info("Temperature: %d.%d C; Humidity: %d.%d%%\n",
		temperature / 10,
		temperature % 10,	
		humidity / 10,
		humidity % 10);

	reset_data();
}

module_init(dht22_init);
module_exit(dht22_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Filip Kolev");
MODULE_DESCRIPTION("A test module for the DHT22 sensor.");
MODULE_VERSION("0.1");
