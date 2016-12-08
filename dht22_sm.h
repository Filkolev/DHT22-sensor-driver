#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/slab.h>

struct dht22_sm *
create_sm(struct work_struct *work,
	struct work_struct *cleanup_work,
	struct workqueue_struct *wq);

void destroy_sm(struct dht22_sm *sm);

enum dht22_state {
	IDLE = 0,
	RESPONDING,
	FINISHED,
	ERROR,
	COUNT_STATES
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
	struct work_struct *work;
	struct work_struct *cleanup_work;
	struct workqueue_struct *wq;
};
