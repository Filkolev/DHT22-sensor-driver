#include <linux/hrtimer.h>

#define GPIO_DEFAULT 23
#define AUTOUPDATE_DEFAULT false

/*
 * 2s minimum between readings. Empirically observed that a small timeout
 * leads to more errors (missed interrupts or hash mismatches)
 */
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

static void sm_work_func(struct work_struct *work);
static void cleanup_work_func(struct work_struct *work);
