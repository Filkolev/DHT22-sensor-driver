#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/slab.h>

struct dht22_sm * create_sm(void);
void destroy_sm(struct dht22_sm *sm);

void reset_dht22_sm(struct dht22_sm *sm);

enum dht22_state get_next_state_idle(struct dht22_sm *sm);
enum dht22_state get_next_state_responding(struct dht22_sm *sm);
enum dht22_state get_next_state_finished(struct dht22_sm *sm);
enum dht22_state get_next_state_error(struct dht22_sm *sm);

void handle_idle(struct dht22_sm *sm);
void handle_finished(struct dht22_sm *sm);
void noop(struct dht22_sm *sm);

void change_dht22_sm_state(struct dht22_sm *sm);
void handle_dht22_state(struct dht22_sm *sm);

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
	struct work_struct work;
	struct work_struct cleanup_work;
};
