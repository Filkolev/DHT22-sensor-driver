#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/slab.h>

#include "dht22_sm.h"

static void reset_dht22_sm(struct dht22_sm *sm);

static enum dht22_state get_next_state_idle(struct dht22_sm *sm);
static enum dht22_state get_next_state_responding(struct dht22_sm *sm);
static enum dht22_state get_next_state_finished(struct dht22_sm *sm);
static enum dht22_state get_next_state_error(struct dht22_sm *sm);

static void handle_idle(struct dht22_sm *sm);
static void handle_finished(struct dht22_sm *sm);
static void noop(struct dht22_sm *sm);

static void change_dht22_sm_state(struct dht22_sm *sm);
static void handle_dht22_state(struct dht22_sm *sm);

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

static void noop(struct dht22_sm *sm) { }

struct dht22_sm *
create_sm(struct work_struct *work,
	struct work_struct *cleanup_work,
	struct workqueue_struct *wq)
{
	struct dht22_sm *sm;

	sm = kmalloc(sizeof(struct dht22_sm), GFP_KERNEL);
	if (!sm) {
		pr_err("Could not create state machine. Exiting...\n");
		return ERR_PTR(-ENOMEM);
	}

	sm->reset = reset_dht22_sm;
	sm->change_state = change_dht22_sm_state;
	sm->handle_state = handle_dht22_state;
	sm->work = work;
	sm->cleanup_work = cleanup_work;
	sm->wq = wq;
	sm->reset(sm);

	return sm;
}

void destroy_sm(struct dht22_sm *sm)
{
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

static void handle_idle(struct dht22_sm *sm)
{
	if (sm->dirty)
		queue_work(sm->wq, sm->cleanup_work);
}

static void handle_finished(struct dht22_sm *sm)
{
	queue_work(sm->wq, sm->work);
}

static void change_dht22_sm_state(struct dht22_sm *sm)
{
	sm->state = (*state_functions[sm->state])(sm);
}

static void handle_dht22_state(struct dht22_sm *sm)
{
	(*handler_functions[sm->state])(sm);
}

static void reset_dht22_sm(struct dht22_sm *sm)
{
	sm->state = IDLE;
	sm->finished = false;
	sm->triggered = false;
	sm->error = false;
	sm->dirty = false;
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Filip Kolev");
MODULE_DESCRIPTION("A test module for the DHT22 sensor.");
MODULE_VERSION("0.1");
