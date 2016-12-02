#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/time64.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/ktime.h>
#include <linux/hrtimer.h>

/*
 * TODO: Implement the following:
 * - export to sysfs
 */

#define GPIO_DEFAULT 23
#define AUTOUPDATE_DEFAULT false

/* 2s minimum between readings, 5s empirically */
#define AUTOUPDATE_TIMEOUT_MIN 2000
#define DATA_SIZE 5 /* Number of bytes the DHT22 sensor sends */
#define BITS_PER_BYTE 8
#define EXPECTED_IRQ_COUNT 86 /* The total number of interrupts to process */
#define TRIGGER_IRQ_COUNT 3
#define INIT_RESPONSE_IRQ_COUNT 2
#define DATA_IRQ_COUNT 80

#define LOW 0
#define HIGH 1

/* signal lengths in ms */
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
static enum hrtimer_restart timer_func(struct hrtimer *);

static irqreturn_t dht22_irq_handler(int irq, void *data);
static void process_results(struct work_struct *work);
static void process_data(void);

static int setup_dht22_gpio(int gpio);
static int setup_dht22_irq(int gpio);
static struct dht22_sm * create_sm(void);
static void destroy_sm(struct dht22_sm *sm);
static void change_dht22_sm_state(struct dht22_sm *sm);
static void handle_dht22_state(struct dht22_sm *sm);
static void sm_work_func(struct work_struct *work);
static void cleanup_work_func(struct work_struct *work);

static void reset_dht22_sm(struct dht22_sm *sm);

static enum dht22_state get_next_state_idle(struct dht22_sm *sm);
static enum dht22_state get_next_state_responding(struct dht22_sm *sm);
static enum dht22_state get_next_state_finished(struct dht22_sm *sm);
static enum dht22_state get_next_state_error(struct dht22_sm *sm);

static void handle_idle(struct dht22_sm *sm);
static void handle_finished(struct dht22_sm *sm);
static void noop(struct dht22_sm *sm) { }

enum dht22_state {
	IDLE = 0,
	RESPONDING,
	FINISHED,
	ERROR,
	COUNT_STATES
};

static enum dht22_state
(*state_functions[COUNT_STATES])(struct dht22_sm *sm) = {
	get_next_state_idle,
	get_next_state_responding,
	get_next_state_finished,
	get_next_state_error
};

static void (*handler_functions[COUNT_STATES])(struct dht22_sm *sm) = {
	handle_idle,
	noop,
	handle_finished,
	noop
};

struct dht22_sm {
	enum dht22_state state;
	void (*change_state)(struct dht22_sm *sm);
	void (*handle_state)(struct dht22_sm *sm);
	void (*reset)(struct dht22_sm *sm);
	bool finished;
	bool triggered;
	bool error;
	bool dirty;
	struct mutex lock;
};

static struct dht22_sm *sm;
static struct timespec64 ts_prev_gpio_switch, ts_prev_reading;
static int irq_number;
static int processed_irq_count = 0;
static ktime_t kt_interval;
static struct hrtimer timer;

static int irq_deltas[EXPECTED_IRQ_COUNT];
static int sensor_data[DATA_SIZE];

static DECLARE_WORK(work, process_results);
static DECLARE_WORK(cleanup_work, cleanup_work_func);
static DECLARE_WORK(sm_work_0, sm_work_func);
static DECLARE_WORK(sm_work_1, sm_work_func);
static DECLARE_WORK(sm_work_2, sm_work_func);
static DECLARE_WORK(sm_work_3, sm_work_func);
static DECLARE_WORK(sm_work_4, sm_work_func);

static struct work_struct *workers[] = {
	&sm_work_0,
	&sm_work_1,
	&sm_work_2,
	&sm_work_3,
	&sm_work_4
};

static int gpio = GPIO_DEFAULT;
module_param(gpio, int, S_IRUGO);
MODULE_PARM_DESC(gpio, "GPIO number of the DHT22's data pin (default = 23)");

static bool autoupdate = false;
module_param(autoupdate, bool, S_IRUGO);
MODULE_PARM_DESC(autoupdate,
	"Re-trigger sensor automatically? (default = false)");

static int autoupdate_timeout = AUTOUPDATE_TIMEOUT_MIN;
module_param(autoupdate_timeout, int, S_IRUGO);
MODULE_PARM_DESC(autoupdate_timeout,
	"Interval between trigger events for the sensor (default = 2s)");

static int __init dht22_init(void)
{
	int ret;

	pr_info("DHT22 module loading...\n");
	ret = 0;

	sm = create_sm();
	if (IS_ERR(sm)) {
		ret = PTR_ERR(sm);
		goto out;
	}

	sm->reset(sm);

	ret = setup_dht22_gpio(gpio);
	if (ret)
		goto gpio_err;

	getnstimeofday64(&ts_prev_gpio_switch);
	ret = setup_dht22_irq(gpio);
	if (ret)
		goto irq_err;

	reset_data();

	if (autoupdate_timeout < AUTOUPDATE_TIMEOUT_MIN)
		autoupdate_timeout = AUTOUPDATE_TIMEOUT_MIN;

	kt_interval = ktime_set(autoupdate_timeout / MSEC_PER_SEC,
			(autoupdate_timeout % MSEC_PER_SEC) * NSEC_PER_USEC);

	hrtimer_init(&timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	timer.function = timer_func;
	hrtimer_start(&timer, kt_interval, HRTIMER_MODE_REL);

	pr_info("DHT22 module finished loading.\n");
	goto out;

irq_err:
//	gpio_unexport(gpio);
	gpio_free(gpio);
gpio_err:
	destroy_sm(sm);
	sm = NULL;
out:
	return ret;
}

static void __exit dht22_exit(void)
{
	hrtimer_cancel(&timer);
	free_irq(irq_number, NULL);
//	gpio_unexport(gpio);
	gpio_free(gpio);
	destroy_sm(sm);
	sm = NULL;	

	pr_info("DHT22 module unloaded\n");
}

static void trigger_sensor(void)
{
	/*
	 * According to datasheet the triggering signal is as follows:
	 * - prepare (wait some time while line is HIGH): 250 ms
	 * - send start signal (pull line LOW): 20 ms LOW
	 * - end start signal (stop pulling LOW): 40 us HIGH
	 */
	pr_info("Sensor triggered\n");
	sm->triggered = true;
	sm->state = RESPONDING;
	getnstimeofday64(&ts_prev_reading);

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
}

static enum hrtimer_restart timer_func(struct hrtimer *hrtimer)
{
	/*
	 * If the count of processed IRQs is not 0, this means the previous
	 * reading is still ongoig (either the sensor was slow to respond or
	 * we missed an interrupt and never reached the finish state).
	 * Reset the state to allow the sensor to continue.
	 * In trials this shows effective insofar as the sensor manages to
	 * recover and starts the next reading. However, sequences of errors
	 * are sometimes observed in quick succession. Need to figure out a
	 * way to recover in a more graceful way.
	 */
	ktime_t delay;

	delay = ktime_set(0, 0);
	if (processed_irq_count) {
		pr_info("Resetting. Processed IRQs: %d\n", processed_irq_count);
		reset_data();
		sm->reset(sm);
		/*
		 * Delay the next trigger event to prevent multple successive
		 * errors. Doesn't seem to have a tangible positive effect...
		 */
		delay = ktime_set(1, 0);
	}

	trigger_sensor();

	hrtimer_forward_now(hrtimer, ktime_add(kt_interval, delay));
	return (autoupdate ? HRTIMER_RESTART : HRTIMER_NORESTART);
}

static irqreturn_t dht22_irq_handler(int irq, void *data)
{
	struct timespec64 ts_current_irq, ts_diff;

	if (!sm->triggered || processed_irq_count >= EXPECTED_IRQ_COUNT) {
		sm->error = true;
		goto irq_handle;
	}

	getnstimeofday64(&ts_current_irq);
	ts_diff = timespec64_sub(ts_current_irq, ts_prev_gpio_switch);

	irq_deltas[processed_irq_count] = (int)(ts_diff.tv_nsec / NSEC_PER_USEC);
	processed_irq_count++;
	ts_prev_gpio_switch = ts_current_irq;

	if (processed_irq_count == EXPECTED_IRQ_COUNT) {
		sm->finished = true;
	}

irq_handle:
	schedule_work(workers[processed_irq_count % ARRAY_SIZE(workers)]);

	return IRQ_HANDLED;
}

static int setup_dht22_gpio(int gpio)
{
	int ret;

	ret = 0;
	if (!gpio_is_valid(gpio)) {
		pr_err("Failed validation of GPIO %d\n", gpio);
		return -EINVAL;
	}

	pr_info("Validation succeeded for GPIO %d\n", gpio);

	ret = gpio_request(gpio, "sysfs");
	if (ret < 0) {
		pr_err("GPIO request failed. Exiting.\n");
		return ret;
	}

	gpio_direction_input(gpio);
//	gpio_export(gpio, true);

	return ret;
}

static int setup_dht22_irq(int gpio)
{
	int ret;

	ret = 0;

	irq_number = gpio_to_irq(gpio);
	if (irq_number < 0) {
		pr_err("Failed to retrieve IRQ number for GPIO. Exiting.\n");
		return irq_number;
	}

	pr_info("Assigned IRQ number %d\n", irq_number);

	ret = request_irq(irq_number,
			dht22_irq_handler,
			(IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING),
			"dht22_gpio_handler",
			NULL);
	if (ret < 0) {
		pr_err("request_irq() failed. Exiting.\n");
	}
	
	return ret;
}

static struct dht22_sm *create_sm(void)
{
	struct dht22_sm *sm;
	struct mutex lock;

	sm = kmalloc(sizeof(struct dht22_sm), GFP_KERNEL);
	if (!sm) {
		pr_err("Could not create state machine. Exiting...\n");
		return ERR_PTR(-ENOMEM);
	}

	mutex_init(&lock);
	sm->lock = lock;
	
	sm->change_state = change_dht22_sm_state;
	sm->handle_state = handle_dht22_state;
	sm->reset = reset_dht22_sm;

	return sm;
}

static void destroy_sm(struct dht22_sm *sm)
{
	mutex_destroy(&sm->lock);
	kfree(sm);	
}

static enum dht22_state get_next_state_idle(struct dht22_sm *sm)
{
	if (sm->error)
		return ERROR;

	if (sm->triggered)
		return RESPONDING;

	return IDLE;
}

static enum dht22_state get_next_state_responding(struct dht22_sm *sm)
{
	if (sm->error)
		return ERROR;

	if (sm->finished)
		return FINISHED;

	return RESPONDING;
}

static enum dht22_state get_next_state_finished(struct dht22_sm *sm)
{
	if (sm->error)
		return ERROR;

	return IDLE;
}

static enum dht22_state get_next_state_error(struct dht22_sm *sm)
{
	return IDLE;
}

static void change_dht22_sm_state(struct dht22_sm *sm)
{
	sm->state = (*state_functions[sm->state])(sm);
}

static inline void handle_dht22_state(struct dht22_sm *sm)
{
	(*handler_functions[sm->state])(sm);
}

static void handle_idle(struct dht22_sm *sm)
{
	if (sm->dirty)
		schedule_work(&cleanup_work);
}

static void handle_finished(struct dht22_sm *sm)
{
	schedule_work(&work);
}

static void sm_work_func(struct work_struct *work)
{
	sm->change_state(sm);
	sm->handle_state(sm);
}

static void cleanup_work_func(struct work_struct *work)
{
	reset_data();
	sm->reset(sm);
}

static void reset_dht22_sm(struct dht22_sm *sm)
{
	sm->state = IDLE;
	sm->finished = false;
	sm->triggered = false;	
	sm->error = false;
	sm->dirty = false;
}

static void process_data(void)
{
	int i, bit_value, current_byte, current_bit, start_idx;

	/*
	 * Skip the triggering and initial response irq deltas and process
	 * the data irq deltas (2 for each bit, a start signal and the value).
	 * Most significant bits arrive first.
	 */
	start_idx = TRIGGER_IRQ_COUNT + INIT_RESPONSE_IRQ_COUNT;
	for (i = start_idx; i < start_idx + DATA_IRQ_COUNT ; i += 2) {
		bit_value = irq_deltas[i + 1] < PREP_SIGNAL_LEN ? 0 : 1;
		current_byte = (i - start_idx) / (BITS_PER_BYTE << 1);
		current_bit = 7 - (((i - start_idx) % (BITS_PER_BYTE << 1)) >> 1);
		sensor_data[current_byte] |= bit_value << current_bit;
	}
}

static void process_results(struct work_struct *work)
{
	int hash, temperature, humidity;

	process_data();

	hash = sensor_data[0] + sensor_data[1] + sensor_data[2] + sensor_data[3];
	hash &= 0xFF;
	if (hash != sensor_data[4]) {
		pr_err("Hash mismatch. Stopping. (%d, %d, %d, %d, %d)\n",
				sensor_data[0],
				sensor_data[1],
				sensor_data[2],
				sensor_data[3],
				sensor_data[4]);
		schedule_work(&cleanup_work);
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

	schedule_work(&cleanup_work);
}

module_init(dht22_init);
module_exit(dht22_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Filip Kolev");
MODULE_DESCRIPTION("A test module for the DHT22 sensor.");
MODULE_VERSION("0.1");
