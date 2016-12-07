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
#include <linux/kobject.h>

#include "dht22.h"
#include "dht22_sm.h"

static struct dht22_sm *sm;
static struct timespec64 ts_prev_gpio_switch, ts_prev_reading;
static int irq_number;
static int processed_irq_count = 0;
static ktime_t kt_interval;
static struct hrtimer timer;
static struct kobject *dht22_kobj;

static int irq_deltas[EXPECTED_IRQ_COUNT];
static int sensor_data[DATA_SIZE];

static int raw_temperature = 0;
static int raw_humidity = 0;

static struct workqueue_struct *queue;

static DECLARE_WORK(trigger_work, trigger_sensor);
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
MODULE_PARM_DESC(gpio, "GPIO number of the DHT22's data pin (default = 6)");

static bool autoupdate = false;
module_param(autoupdate, bool, S_IRUGO);
MODULE_PARM_DESC(autoupdate,
	"Re-trigger sensor automatically? (default = false)");

static int autoupdate_timeout = AUTOUPDATE_TIMEOUT_MIN;
module_param(autoupdate_timeout, int, S_IRUGO);
MODULE_PARM_DESC(autoupdate_timeout,
	"Interval between trigger events (default: 2s, min: 2s, max: 10 min)");

static struct kobj_attribute gpio_attr = __ATTR_RO(gpio_number);
static struct kobj_attribute autoupdate_attr = __ATTR_RW(autoupdate);
static struct kobj_attribute autoupdate_timeout_attr =
		__ATTR_RW(autoupdate_timeout_ms);
static struct kobj_attribute temperature_attr = __ATTR_RO(temperature);
static struct kobj_attribute humidity_attr = __ATTR_RO(humidity);

static struct attribute *dht22_attrs[] = {
	&gpio_attr.attr,
	&autoupdate_attr.attr,
	&autoupdate_timeout_attr.attr,
	&temperature_attr.attr,
	&humidity_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.name = "dht22",
	.attrs = dht22_attrs,
};

static int __init dht22_init(void)
{
	int ret;

	pr_info("DHT22 module loading...\n");
	ret = 0;

	/*
	 * Create a workqueue with high priority which can only run one
	 * work item at a time. The strict execution ordering is mandated
	 * by the need to ensure the state machine processes its state
	 * properly.
	 * In case we fail to allocate the workqueue, we fall back to the
	 * system high-prio workqueue.
	 */
	queue = alloc_workqueue("dht22_queue", WQ_HIGHPRI, 1);
	if (!queue)
		queue = system_highpri_wq;

	sm = create_sm();
	if (IS_ERR(sm)) {
		ret = PTR_ERR(sm);
		goto out;
	}

	sm->reset(sm);
	sm->work = &work;
	sm->cleanup_work = &cleanup_work;
	sm->queue = queue;

	ret = setup_dht22_gpio(gpio);
	if (ret)
		goto gpio_err;

	getnstimeofday64(&ts_prev_gpio_switch);
	ret = setup_dht22_irq(gpio);

	if (ret)
		goto irq_err;

	dht22_kobj = kobject_create_and_add("dht22", kernel_kobj);
	if (!dht22_kobj) {
		pr_err("Failed to create kobject mapping.\n");
		ret = -ENOMEM;
		goto irq_err;
	}

	ret = sysfs_create_group(dht22_kobj, &attr_group);
	if (ret) {
		pr_err("Failed to create sysfs group.\n");
		goto sysfs_err;
	}

	verify_timeout();
	reset_data();

	kt_interval = ktime_set(autoupdate_timeout / MSEC_PER_SEC,
			(autoupdate_timeout % MSEC_PER_SEC) * NSEC_PER_USEC);

	hrtimer_init(&timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	timer.function = timer_func;
	hrtimer_start(&timer, ktime_set(0, 100 * NSEC_PER_USEC), HRTIMER_MODE_REL);

	pr_info("DHT22 module finished loading.\n");
	goto out;

sysfs_err:
	kobject_put(dht22_kobj);
irq_err:
	gpio_unexport(gpio);
	gpio_free(gpio);
gpio_err:
	destroy_sm(sm);
out:
	return ret;
}

static void __exit dht22_exit(void)
{
	hrtimer_cancel(&timer);
	kobject_put(dht22_kobj);
	free_irq(irq_number, NULL);
	gpio_unexport(gpio);
	gpio_free(gpio);
	destroy_sm(sm);

	pr_info("DHT22 module unloaded\n");
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
	gpio_export(gpio, true);

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

	irq_set_affinity_hint(irq_number, NULL);

	return ret;
}

static void verify_timeout(void)
{
	if (autoupdate_timeout < AUTOUPDATE_TIMEOUT_MIN)
		autoupdate_timeout = AUTOUPDATE_TIMEOUT_MIN;

	if (autoupdate_timeout > AUTOUPDATE_TIMEOUT_MAX)
		autoupdate_timeout = AUTOUPDATE_TIMEOUT_MAX;
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

static void trigger_sensor(struct work_struct *work)
{
	/*
	 * According to datasheet the triggering signal is as follows:
	 * - prepare (wait some time while line is HIGH): 250 ms
	 * - send start signal (pull line LOW): 20 ms LOW
	 * - end start signal (stop pulling LOW): 40 us HIGH
	 */
	sm->triggered = true;
	sm->state = RESPONDING;
	getnstimeofday64(&ts_prev_reading);

	mdelay(TRIGGER_DELAY);

	gpio_direction_output(gpio, LOW);
	mdelay(TRIGGER_SIGNAL_LEN);

	gpio_direction_input(gpio);
	udelay(TRIGGER_POST_DELAY);
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

	kt_interval = ktime_set(autoupdate_timeout / MSEC_PER_SEC,
			(autoupdate_timeout % MSEC_PER_SEC) * NSEC_PER_USEC);
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

	queue_work(queue, &trigger_work);

	hrtimer_forward_now(hrtimer, ktime_add(kt_interval, delay));
	return (autoupdate ? HRTIMER_RESTART : HRTIMER_NORESTART);
}

static irqreturn_t dht22_irq_handler(int irq, void *data)
{
	struct timespec64 ts_current_irq, ts_diff;

	if (!sm->triggered || processed_irq_count >= EXPECTED_IRQ_COUNT) {
		sm->error = true;
		goto handle_irq;
	}

	getnstimeofday64(&ts_current_irq);
	ts_diff = timespec64_sub(ts_current_irq, ts_prev_gpio_switch);

	irq_deltas[processed_irq_count] = (int)(ts_diff.tv_nsec / NSEC_PER_USEC);
	processed_irq_count++;
	ts_prev_gpio_switch = ts_current_irq;

	if (processed_irq_count == EXPECTED_IRQ_COUNT) {
		sm->finished = true;
	}

handle_irq:
	queue_work(queue, workers[processed_irq_count % ARRAY_SIZE(workers)]);

	return IRQ_HANDLED;
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
		queue_work(queue, &cleanup_work);
		return;
	}

	humidity = ((sensor_data[0] << BITS_PER_BYTE) | sensor_data[1]);
	temperature = ((sensor_data[2] << BITS_PER_BYTE) | sensor_data[3]);

	if (sensor_data[2] & 0x80)
		temperature *= -1;

	raw_humidity = humidity;
	raw_temperature = temperature;

	pr_info("Temperature: %d.%d C; Humidity: %d.%d%%\n",
		temperature / 10,
		temperature % 10,
		humidity / 10,
		humidity % 10);

	queue_work(queue, &cleanup_work);
}

static ssize_t
gpio_number_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", gpio);
}

static ssize_t
autoupdate_show(struct kobject *kobj,
		struct kobj_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%d\n", autoupdate);
}

static ssize_t
autoupdate_store(struct kobject *kobj,
		struct kobj_attribute *attr,
		const char *buf,
		size_t count)
{
	int temp;

	sscanf(buf, "%d\n", &temp);
	autoupdate = temp;
	if (autoupdate && !hrtimer_active(&timer))
		hrtimer_restart(&timer);

	return count;
}

static ssize_t
autoupdate_timeout_ms_show(struct kobject *kobj,
			struct kobj_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%d\n", autoupdate_timeout);
}

static ssize_t
autoupdate_timeout_ms_store(struct kobject *kobj,
			struct kobj_attribute *attr,
			const char *buf,
			size_t count)
{
	sscanf(buf, "%d\n", &autoupdate_timeout);
	verify_timeout();

	return count;
}

static ssize_t
temperature_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,
		"%d.%d\n",
		raw_temperature / 10,
		raw_temperature % 10);
}

static ssize_t
humidity_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d.%d%%\n",
		raw_humidity / 10,
		raw_humidity % 10);
}

module_init(dht22_init);
module_exit(dht22_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Filip Kolev");
MODULE_DESCRIPTION("A test module for the DHT22 sensor.");
MODULE_VERSION("0.1");
