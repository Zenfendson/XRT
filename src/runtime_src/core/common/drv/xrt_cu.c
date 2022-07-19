/* SPDX-License-Identifier: GPL-2.0 OR Apache-2.0 */
/*
 * Xilinx Unify CU Model
 *
 * Copyright (C) 2020-2022 Xilinx, Inc. All rights reserved.
 *
 * Authors: min.ma@xilinx.com
 *
 * This file is dual-licensed; you may select either the GNU General Public
 * License version 2 or Apache License, Version 2.0.
 */

#include <linux/delay.h>
#include "kds_client.h"
#include "xrt_cu.h"
#include <asm/timex.h>

inline void xrt_cu_circ_produce(struct xrt_cu *xcu, u32 stage, uintptr_t cmd)
{
	unsigned long head = xcu->crc_buf.head;
	unsigned long tail = xcu->crc_buf.tail;
	struct xrt_cu_log *item;

	if (!xcu->debug)
		return;

	/* Will overwrite oldest data if buffer is full */
	item = (struct xrt_cu_log *)&xcu->crc_buf.buf[head];
	item->stage = stage;
	item->cmd_id = cmd;
	item->ts = ktime_to_ns(ktime_get());

	if (CIRC_SPACE(head, tail, CIRC_BUF_SIZE) < sizeof(struct xrt_cu_log)) {
		tail += sizeof(struct xrt_cu_log);
		if (tail >= CIRC_BUF_SIZE)
			tail = 0;
	}

	head += sizeof(struct xrt_cu_log);
	if (head >= CIRC_BUF_SIZE)
		head = 0;

	xcu->crc_buf.head = head;
	xcu->crc_buf.tail = tail;
}

ssize_t xrt_cu_circ_consume_all(struct xrt_cu *xcu, char *buf, size_t size)
{
	unsigned long head = xcu->crc_buf.head;
	unsigned long tail = xcu->crc_buf.tail;
	size_t cnt = CIRC_CNT(head, tail, CIRC_BUF_SIZE);
	/* return min(count to the end of buffer, CIRC_CNT()) */
	size_t cnt_to_end = CIRC_CNT_TO_END(head, tail, CIRC_BUF_SIZE);
	size_t nread = 0;

	if (size < cnt)
		nread = size;
	else
		nread = cnt;

	if (nread <= cnt_to_end) {
		memcpy(buf, xcu->crc_buf.buf + tail, nread);
		xcu->crc_buf.tail += nread;
	} else {
		/* Needs two times of copy */
		memcpy(buf, xcu->crc_buf.buf + tail, cnt_to_end);
		memcpy(buf + cnt_to_end, xcu->crc_buf.buf, nread - cnt_to_end);
		xcu->crc_buf.tail = nread - cnt_to_end;
	}

	return nread;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0)
static void cu_timer(unsigned long data)
{
	struct xrt_cu *xcu = (struct xrt_cu *)data;
#else
static void cu_timer(struct timer_list *t)
{
	struct xrt_cu *xcu = from_timer(xcu, t, timer);
#endif

	xcu_dbg(xcu, "%s tick\n", xcu->info.iname);

	atomic_inc(&xcu->tick);

	mod_timer(&xcu->timer, jiffies + CU_TIMER);
}

static inline struct kds_client *
first_event_client_or_null(struct xrt_cu *xcu)
{
	struct kds_client *curr = NULL;

	if (list_empty(&xcu->events))
		return NULL;

	mutex_lock(&xcu->ev_lock);
	if (list_empty(&xcu->events))
		goto done;

	curr = list_first_entry(&xcu->events, struct kds_client, ev_entry);

done:
	mutex_unlock(&xcu->ev_lock);
	return curr;
}

static inline void
move_to_queue(struct kds_command *xcmd, struct list_head *dst_q, u32 *dst_len)
{
	if (dst_q)
		list_move_tail(&xcmd->list, dst_q);
	++(*dst_len);
}

static bool abort_client(struct kds_command *xcmd, void *cond)
{
	void *client = cond;

	return (xcmd->client == client)? true : false;
}

static bool abort_handle(struct kds_command *xcmd, void *cond)
{
	u32 handle = *(u32 *)cond;

	return (xcmd->exec_bo_handle == handle)? true : false;
}

/**
 * process_cq() - Process completed queue
 * @xcu: Target XRT CU
 */
static inline void process_cq(struct xrt_cu *xcu)
{
	struct kds_command *xcmd;

	if (!xcu->num_cq)
		return;

	/* Notify host and free command
	 *
	 * NOTE: Use loop to handle completed commands could improve
	 * performance (Reduce sleep time and increase chance for more
	 * pending commands at one time)
	 */
	while (xcu->num_cq) {
		xcmd = list_first_entry(&xcu->cq, struct kds_command, list);
		set_xcmd_timestamp(xcmd, xcmd->status);
		xrt_cu_circ_produce(xcu, CU_LOG_STAGE_CQ, (uintptr_t)xcmd);
		xcmd->cb.notify_host(xcmd, xcmd->status);
		if (xcu->stats_enabled) {
			update_ecmd_time(xcu, xcmd->start_ts, xcmd->end_ts);
			incr_ecmd_count(xcu);
		}
		list_del(&xcmd->list);
		xcmd->cb.free(xcmd);
		--xcu->num_cq;
		xcu->cu_stat.usage++;
	}
}

/**
 * __process_sq() - Process submitted queue
 * @xcu: Target XRT CU
 */
static inline void __process_sq(struct xrt_cu *xcu)
{
	struct kds_command *xcmd;
	struct kds_client *ev_client = NULL;
	unsigned int tick;

	/* CU is ready to accept more commands
	 * Return credits to allow submit more commands
	 */
	if (xcu->ready_cnt) {
		xrt_cu_put_credit(xcu, xcu->ready_cnt);
		xcu->ready_cnt = 0;
	}

	if (xcu->stats_enabled) {
		xrt_cu_get_time(&xcu->sq_time);
		if((xcu->sq_time - xcu->last_sq_time) > xcu->sq_barrer) {
			incr_sq_count(xcu);
		}
	}
	/* Sometimes a CU done but it doesn't ready for new command.
	 * In this case, sq could be empty.
	 */
	if (!xcu->num_sq)
		return;

	ev_client = first_event_client_or_null(xcu);
	if (unlikely(ev_client)) {
		tick = atomic_read(&xcu->tick);
		if (xcu->start_tick == 0) {
			xcu->start_tick = tick;
			goto get_complete_and_out;
		}

		if (tick - xcu->start_tick < CU_EXEC_DEFAULT_TTL)
			goto get_complete_and_out;

		if (xrt_cu_cmd_abort(xcu, ev_client, abort_client))
			xcu->bad_state = true;

		xcu->start_tick = 0;
	}

get_complete_and_out:
	xcmd = xrt_cu_get_complete(xcu);
	while (xcmd) {
		xrt_cu_circ_produce(xcu, CU_LOG_STAGE_SQ, (uintptr_t)xcmd);
		move_to_queue(xcmd, &xcu->cq, &xcu->num_cq);
		--xcu->num_sq;
		xcmd = xrt_cu_get_complete(xcu);
	}
	if (xcu->stats_enabled) {
		if (xcu->num_sq == 0 && xcu->num_rq == 0 && xcu->num_pq == 0) {
			// printk("idle start");
			xrt_cu_get_time(&xcu->idle_start);
			xcu->idle = 1;
		}
	}

}

/**
 * process_sq() - Process submitted queue
 * @xcu: Target XRT CU
 */
static inline void process_sq(struct xrt_cu *xcu)
{
	/* A submitted command might be done but the CU is still
	 * not ready for next command. In this case, check CU status
	 * to get ready count.
	 */
	if (!xcu->num_sq && !is_zero_credit(xcu))
		return;

	/* If no command is running on the hardware,
	 * this would not really access hardware.
	 */
	xrt_cu_check(xcu);

	__process_sq(xcu);
}

/**
 * process_rq() - Process run queue
 * @xcu: Target XRT CU
 *
 * Return: return 0 if run queue is empty or no credit
 *	   Otherwise, return 1
 */
static inline int process_rq(struct xrt_cu *xcu)
{
	struct kds_command *xcmd;
	struct kds_client *ev_client;
	struct list_head *dst_q;
	int *dst_len;

	if (!xcu->num_rq)
		return 0;

	xcmd = list_first_entry(&xcu->rq, struct kds_command, list);

	ev_client = first_event_client_or_null(xcu);
	if (unlikely(xcu->bad_state || (ev_client == xcmd->client))) {
		xcmd->status = KDS_ABORT;
		dst_q = &xcu->cq;
		dst_len = &xcu->num_cq;
		goto move_cmd;
	}

	if (!xrt_cu_get_credit(xcu))
		return 0;

	/* if successfully get credit, you must start cu */
	if (xrt_cu_submit_config(xcu, xcmd)) {
		xrt_cu_put_credit(xcu, 1);
		return 0;
	}
	xrt_cu_start(xcu);
	set_xcmd_timestamp(xcmd, KDS_RUNNING);
	xrt_cu_circ_produce(xcu, CU_LOG_STAGE_RQ, (uintptr_t)xcmd);

	dst_q = NULL;
	dst_len = &xcu->num_sq;
	xrt_cu_get_time(&xcu->newest_cmd_start_ts);
	/* ktime_get_* is still heavy. This impact ~20% of IOPS on echo mode.
	 * For some sort of CU, which usually has a relative long execute time,
	 * we could hide this overhead and provide timestamp for each command.
	 * But this implementation is used for general purpose. Please create CU
	 * specific thread if needed.
	 */
	//xcmd->start = ktime_get_raw_fast_ns();
move_cmd:
	move_to_queue(xcmd, dst_q, dst_len);
	--xcu->num_rq;

	return 1;
}

/**
 * process_pq() - Process pending queue
 * @xcu: Target XRT CU
 *
 * Move all of the pending queue commands to the tail of run queue
 * and re-initialized pending queue
 */
static inline void process_pq(struct xrt_cu *xcu)
{
	unsigned long flags;
	u64           idle_start;

	/* Get pending queue command number without lock.
	 * The idea is to reduce the possibility of conflict on lock.
	 * Need to check pending command number again after lock.
	 */
	if (!xcu->num_pq)
		return;
	if (xcu->stats_enabled) {
		// printk("num_pq: %u, num_sq: %u, num_rq: %u", xcu->num_pq, xcu->num_sq, xcu->num_rq);
		if (xcu->num_pq > 0 && xcu->num_sq == 0 && xcu->num_rq == 0) {
			// printk("idle end");
			idle_start = xcu->idle_start;
			if (xcu->idle) {
				xrt_cu_get_time(&xcu->idle_end);
				xcu->idle_total += xcu->idle_end - idle_start;
				// printk("idle_total: %llu", xcu->idle_total);
				xcu->idle = 0;
			}
		} 
	}
	spin_lock_irqsave(&xcu->pq_lock, flags);
	if (xcu->num_pq) {
		list_splice_tail_init(&xcu->pq, &xcu->rq);
		xcu->num_rq += xcu->num_pq;
		xcu->num_pq = 0;
	}
	spin_unlock_irqrestore(&xcu->pq_lock, flags);
	if (xcu->max_running < xcu->num_rq)
		xcu->max_running = xcu->num_rq;
}

static inline void
try_abort_cmd(struct xrt_cu *xcu, struct kds_command *abort_cmd)
{
	struct kds_command *xcmd;
	struct kds_command *tmp;
	u32 handle;

	/* Never call this function on the performance critical path */
	handle = *(u32 *)abort_cmd->info;
	list_for_each_entry_safe(xcmd, tmp, &xcu->rq, list) {
		if (xcmd->exec_bo_handle != handle)
			continue;

		/* Found the xcmd to abort! */
		abort_cmd->status = KDS_COMPLETED;

		xcu_info(xcu, "Abort command(%d) on running queue", handle);
		xcmd->status = KDS_ABORT;
		move_to_queue(xcmd, &xcu->cq, &xcu->num_cq);
		--xcu->num_rq;
		return;
	}

	if (!xcu->num_sq)
		return;

	if (xrt_cu_cmd_abort(xcu, &handle, abort_handle)) {
		xcu->bad_state = true;
		abort_cmd->status = KDS_TIMEOUT;
	} else {
		abort_cmd->status = KDS_COMPLETED;
	}

	xcmd = xrt_cu_get_complete(xcu);
	while (xcmd) {
		move_to_queue(xcmd, &xcu->cq, &xcu->num_cq);
		--xcu->num_sq;
		xcmd = xrt_cu_get_complete(xcu);
	}
}

/**
 * process_hpq() - Process high priority queue
 * @xcu: Target XRT CU
 *
 */
static inline void process_hpq(struct xrt_cu *xcu)
{
	struct kds_command *xcmd;
	struct kds_command *tmp;
	unsigned long flags;

	if (!xcu->num_hpq)
		return;

	/* slowpath */
	spin_lock_irqsave(&xcu->hpq_lock, flags);
	if (!xcu->num_hpq)
		goto done;

	list_for_each_entry_safe(xcmd, tmp, &xcu->hpq, list) {
		if (xcmd->opcode == OP_ABORT)
			try_abort_cmd(xcu, xcmd);

		list_del(&xcmd->list);
		--xcu->num_hpq;
	}

done:
	spin_unlock_irqrestore(&xcu->hpq_lock, flags);
	complete(&xcu->comp);
}

int xrt_cu_process_queues(struct xrt_cu *xcu)
{
	xcu->stats_enabled = 1;
	/* Move pending commands to running queue */
	process_pq(xcu);

	/* Make sure to submit as many commands as possible */
	while (process_rq(xcu));

	/* process completed queue before submitted queue, for
	 * two reasons:
	 * - The last submitted command may be still running
	 * - while handling completed queue, running command might done
	 * - process_sq will check CU status, which is thru slow bus
	 */
	process_cq(xcu);
	process_sq(xcu);

	/* process rare commands with very high priority. For example
	 * abort command.
	 * If this kind of command don't not exist, this would be a very
	 * low overhead and should not impact performance.
	 * But if this kind of command exist, this would be a slow path.
	 */
	process_hpq(xcu);

	if (xcu->num_rq || xcu->num_sq || xcu->num_cq)
		return XCU_BUSY;

	return XCU_IDLE;
}

int xrt_cu_intr_thread(void *data)
{
	struct xrt_cu *xcu = (struct xrt_cu *)data;
	int ret = 0;
	int loop_cnt = 0;

	xcu_info(xcu, "CU[%d] start", xcu->info.cu_idx);
	mod_timer(&xcu->timer, jiffies + CU_TIMER);
	xrt_cu_enable_intr(xcu, CU_INTR_DONE | CU_INTR_READY);
	while (!xcu->stop) {
		/* Make sure to submit as many commands as possible.
		 * This is why we call continue here. This is important to make
		 * CU busy, especially CU has hardware queue.
		 */
		if (process_rq(xcu))
			continue;

		if (xcu->num_sq || is_zero_credit(xcu)) {
			xrt_cu_check(xcu);
			if (!xcu->done_cnt || !xcu->ready_cnt) {
				xcu->sleep_cnt++;
				/* Don't use down_interruptible() here.
				 * If CU hang, this thread would keep waiting.
				 * Host application is not able to exit since
				 * there are outstading commands.
				 *
				 * CU_TIMER is runing at low frequence. For
				 * normal CU, it will unlikely timeout.
				 */
				if (down_timeout(&xcu->sem_cu, CU_TIMER))
					ret = -ERESTARTSYS;
				xrt_cu_check(xcu);
			}
			__process_sq(xcu);
		}

		process_cq(xcu);
		process_hpq(xcu);

		/* Avoid large num_rq leads to more 120 sec blocking */
		if (++loop_cnt == 8) {
			loop_cnt = 0;
			schedule();
		}

		/* Continue until run queue empty */
		if (xcu->num_rq)
			continue;

		if (!xcu->num_sq && !xcu->num_cq) {
			loop_cnt = 0;
			/* Record CU status before sleep */
			if (!xcu->num_pq)
				xrt_cu_check_force(xcu);
			if (down_interruptible(&xcu->sem))
				ret = -ERESTARTSYS;
		}

		process_pq(xcu);
	}
	xrt_cu_disable_intr(xcu, CU_INTR_DONE | CU_INTR_READY);
	del_timer_sync(&xcu->timer);

	if (xcu->bad_state)
		ret = -EBUSY;

	xcu_info(xcu, "CU[%d] thread end, bad state %d\n",
		 xcu->info.cu_idx, xcu->bad_state);
	return ret;
}

void xrt_cu_submit(struct xrt_cu *xcu, struct kds_command *xcmd)
{
	unsigned long flags;
	bool first_command = false;

	/* Add command to pending queue
	 * wakeup CU thread if it is the first command
	 */
	spin_lock_irqsave(&xcu->pq_lock, flags);
	list_add_tail(&xcmd->list, &xcu->pq);
	++xcu->num_pq;
	first_command = (xcu->num_pq == 1);
	spin_unlock_irqrestore(&xcu->pq_lock, flags);
	if (first_command)
		up(&xcu->sem);
}

void xrt_cu_hpq_submit(struct xrt_cu *xcu, struct kds_command *xcmd)
{
	unsigned long flags;

	/* This high priority queue is designed to handle those hight priority
	 * but low frequency commands. Like abort command.
	 */
	spin_lock_irqsave(&xcu->hpq_lock, flags);
	list_add_tail(&xcmd->list, &xcu->hpq);
	++xcu->num_hpq;
	spin_unlock_irqrestore(&xcu->hpq_lock, flags);
	up(&xcu->sem);

	wait_for_completion(&xcu->comp);
}

/**
 * xrt_cu_abort() - Set an abort event in the CU for client
 * @xcu: Target XRT CU
 * @client: The client tries to abort commands
 *
 * This is used to ask CU thread to abort all commands from the client.
 */
void xrt_cu_abort(struct xrt_cu *xcu, struct kds_client *client)
{
	struct kds_client *curr;

	mutex_lock(&xcu->ev_lock);
	if (list_empty(&xcu->events))
		goto add_event;

	/* avoid re-add the same client */
	list_for_each_entry(curr, &xcu->events, ev_entry) {
		if (client == curr)
			goto done;
	}

add_event:
	client->ev_type = EV_ABORT;
	list_add_tail(&client->ev_entry, &xcu->events);
done:
	mutex_unlock(&xcu->ev_lock);
}

/**
 * xrt_cu_abort_done() - Let CU know client asked abort is done
 * @xcu: Target XRT CU
 * @client: The client tries to abort commands
 *
 */
bool xrt_cu_abort_done(struct xrt_cu *xcu, struct kds_client *client)
{
	struct kds_client *curr;
	struct kds_client *next;

	mutex_lock(&xcu->ev_lock);
	if (list_empty(&xcu->events))
		goto done;

	list_for_each_entry_safe(curr, next, &xcu->events, ev_entry) {
		if (client != curr)
			continue;

		list_del(&curr->ev_entry);
		break;
	}

done:
	mutex_unlock(&xcu->ev_lock);

	return xcu->bad_state;
}

int xrt_cu_cfg_update(struct xrt_cu *xcu, int intr)
{
	int err = 0;

	/* Check if CU support interrupt in hardware */
	if (!xcu->info.intr_enable)
		return -ENOSYS;

	if (xrt_cu_get_protocol(xcu) == CTRL_NONE) {
		xcu_warn(xcu, "Interrupt enabled value should be false for ap_ctrl_none cu\n");
		return -ENOSYS;
	}

	if (xcu->thread) {
		/* Stop old thread */
		xcu->stop = 1;
		up(&xcu->sem_cu);
		up(&xcu->sem);
		if (!IS_ERR(xcu->thread))
			(void) kthread_stop(xcu->thread);
		xcu->thread = NULL;
	}

	if (!intr)
		return 0;

	/* launch new thread */
	xcu->stop = 0;
	sema_init(&xcu->sem, 0);
	sema_init(&xcu->sem_cu, 0);
	atomic_set(&xcu->tick, 0);
	xcu->thread = kthread_run(xrt_cu_intr_thread, xcu, xcu->info.iname);
	if (IS_ERR(xcu->thread)) {
		err = IS_ERR(xcu->thread);
		xcu->thread = NULL;
		xcu_err(xcu, "Create CU thread failed, err %d\n", err);
	}

	return err;
}

/*
 * If KDS has to manage PLRAM resources, we should come up with a better design.
 * Ideally, CU subdevice should request for cmdmem resource instead of KDS assign
 * cmdmem resource to CU.
 * Or, another solution is to let KDS create CU subdevice indtead of
 * icap/zocl_xclbin.
 */
static u32 cu_fa_get_desc_size(struct xrt_cu *xcu)
{
	struct xrt_cu_info *info = &xcu->info;
	u32 size = sizeof(descriptor_t);
	int i;

	for (i = 0; i < info->num_args; i++) {
		/* The "nextDescriptorAddr" argument is a fast adapter argument.
		 * It is not required to construct descriptor
		 */
		if (!strcmp(info->args[i].name, "nextDescriptorAddr") || !info->args[i].size)
			continue;

		size += (sizeof(descEntry_t)+info->args[i].size);
	}

	return round_up_to_next_power2(size);
}

int xrt_is_fa(struct xrt_cu *xcu, u32 *size)
{
	struct xrt_cu_info *info = &xcu->info;
	int ret;

	ret = (info->model == XCU_FA)? 1 : 0;

	if (ret && size)
		*size = cu_fa_get_desc_size(xcu);

	return ret;
}

int xrt_fa_cfg_update(struct xrt_cu *xcu, u64 bar, u64 dev, void __iomem *vaddr, u32 num_slots)
{
	struct xrt_cu_fa *cu_fa = xcu->core;
	u32 slot_size;

	if (bar == 0) {
		cu_fa->cmdmem = NULL;
		cu_fa->paddr = 0;
		cu_fa->slot_sz = 0;
		cu_fa->num_slots = 0;
		return 0;
	}

	slot_size = cu_fa_get_desc_size(xcu);

	cu_fa->cmdmem = vaddr;
	cu_fa->paddr = dev;
	cu_fa->slot_sz = slot_size;
	cu_fa->num_slots = num_slots;

	cu_fa->credits = (cu_fa->num_slots < cu_fa->max_credits)?
			 cu_fa->num_slots : cu_fa->max_credits;

	return 0;
}

int xrt_cu_get_protocol(struct xrt_cu *xcu)
{
	return xcu->info.protocol;
}

u32 xrt_cu_get_status(struct xrt_cu *xcu)
{
	return xcu->status;
}

int xrt_cu_regmap_size(struct xrt_cu *xcu)
{
	int max_off_idx = 0;
	int max_off = 0;
	int i;

	if (!xcu->info.num_args)
		return xcu->info.size;

	/* Max regmap size is max offset + register size */
	for (i = 0; i < xcu->info.num_args; i++) {
		if (max_off < xcu->info.args[i].offset) {
			max_off = xcu->info.args[i].offset;
			max_off_idx = i;
		}
	}

	return max_off + xcu->info.args[max_off_idx].size;
}

int xrt_cu_init(struct xrt_cu *xcu)
{
	int err = 0;
	char *name = xcu->info.iname;

	/* TODO A workaround to avoid m2m subdev launch thread */
	if (!strlen(name))
		return 0;

	/* Use list for driver space command queue
	 * Should we consider ring buffer?
	 */

	/* Initialize pending queue and lock */
	INIT_LIST_HEAD(&xcu->pq);
	spin_lock_init(&xcu->pq_lock);
	/* Initialize run queue */
	INIT_LIST_HEAD(&xcu->rq);
	/* Initialize completed queue */
	INIT_LIST_HEAD(&xcu->cq);

	mutex_init(&xcu->ev_lock);
	INIT_LIST_HEAD(&xcu->events);
	sema_init(&xcu->sem, 0);
	sema_init(&xcu->sem_cu, 0);
	mutex_init(&xcu->read_regs.xcr_lock);
	xcu->read_regs.xcr_start = -1;
	xcu->read_regs.xcr_end = -1;

	INIT_LIST_HEAD(&xcu->hpq);
	spin_lock_init(&xcu->hpq_lock);
	init_completion(&xcu->comp);

	xcu->crc_buf.head = 0;
	xcu->crc_buf.tail = 0;
	xcu->crc_buf.buf = xcu->log_buf;

	memset(&xcu->cu_stat, 0, sizeof(struct per_custat));
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0)
	setup_timer(&xcu->timer, cu_timer, (unsigned long)xcu);
#else
	timer_setup(&xcu->timer, cu_timer, 0);
#endif
	atomic_set(&xcu->tick, 0);
	xcu->start_tick = 0;
	xcu->thread = NULL;

	mod_timer(&xcu->timer, jiffies + CU_TIMER);
	init_ecmd_count(xcu);
	init_cu_time(xcu);
	return err;
}

void xrt_cu_fini(struct xrt_cu *xcu)
{
	/* TODO A workaround for m2m subdev */
	if (!strlen(xcu->info.iname))
		return;

	xcu->stop = 1;
	up(&xcu->sem_cu);
	up(&xcu->sem);
	if (xcu->thread && !IS_ERR(xcu->thread))
		(void) kthread_stop(xcu->thread);

	del_timer_sync(&xcu->timer);
	return;
}

ssize_t show_cu_stat(struct xrt_cu *xcu, char *buf)
{
	ssize_t sz = 0;

	/* Add CU dynamic statistic information in below */
	sz += scnprintf(buf+sz, PAGE_SIZE - sz, "Pending queue:    %d\n",
			xcu->num_pq);
	sz += scnprintf(buf+sz, PAGE_SIZE - sz, "Running queue:    %d\n",
			xcu->num_rq);
	sz += scnprintf(buf+sz, PAGE_SIZE - sz, "Submitted queue:  %d\n",
			xcu->num_sq);
	sz += scnprintf(buf+sz, PAGE_SIZE - sz, "Completed queue:  %d\n",
			xcu->num_cq);
	sz += scnprintf(buf+sz, PAGE_SIZE - sz, "Bad state:        %d\n",
			xcu->bad_state);
	sz += scnprintf(buf+sz, PAGE_SIZE - sz, "Current credit:   %d\n",
			xrt_cu_peek_credit(xcu));
	sz += scnprintf(buf+sz, PAGE_SIZE - sz, "CU status:        0x%x\n",
			xcu->status);
	sz += scnprintf(buf+sz, PAGE_SIZE - sz, "sleep cnt:        %d\n",
			xcu->sleep_cnt);
	sz += scnprintf(buf+sz, PAGE_SIZE - sz, "max running:      %d\n",
			xcu->max_running);

	if (xcu->info.model == XCU_FA) {
		sz += scnprintf(buf+sz, PAGE_SIZE - sz, "-- FA CU specific --\n");
		sz += scnprintf(buf+sz, PAGE_SIZE - sz, "Check count: %lld\n",
				to_cu_fa(xcu->core)->check_count);
	}

	if (sz < PAGE_SIZE - 1)
		buf[sz++] = 0;
	else
		buf[PAGE_SIZE - 1] = 0;

	return sz;
}

ssize_t show_cu_info(struct xrt_cu *xcu, char *buf)
{
	struct xrt_cu_info *info = &xcu->info;
	ssize_t sz = 0;
	char dir[10];
	int i;

	/* Add any CU static information in below */
	sz += scnprintf(buf+sz, PAGE_SIZE - sz, "Kernel name: %s\n",
			info->kname);
	sz += scnprintf(buf+sz, PAGE_SIZE - sz, "Instance(CU) name: %s\n",
			info->iname);
	sz += scnprintf(buf+sz, PAGE_SIZE - sz, "CU address: 0x%llx\n",
			info->addr);
	sz += scnprintf(buf+sz, PAGE_SIZE - sz, "CU index: %d\n",
			info->cu_idx);
	sz += scnprintf(buf+sz, PAGE_SIZE - sz, "Protocol: %s\n",
			prot2str(info->protocol));
	sz += scnprintf(buf+sz, PAGE_SIZE - sz, "Interrupt cap: %d\n",
			info->intr_enable);
	sz += scnprintf(buf+sz, PAGE_SIZE - sz, "Interrupt ID:  %d\n",
			info->intr_id);
	sz += scnprintf(buf+sz, PAGE_SIZE - sz, "SW Resettable: %d\n",
			info->sw_reset);

	sz += scnprintf(buf+sz, PAGE_SIZE - sz, "--- Arguments ---\n");
	sz += scnprintf(buf+sz, PAGE_SIZE - sz, "Number of arguments: %d\n",
			info->num_args);
	for (i = 0; i < info->num_args; i++) {
		if (info->args[i].dir == DIR_INPUT)
			strcpy(dir, "input");
		else if (info->args[i].dir == DIR_OUTPUT)
			strcpy(dir, "output");
		else
			strcpy(dir, "unknown");

		sz += scnprintf(buf+sz, PAGE_SIZE - sz, "arg name: %s\n",
				info->args[i].name);
		sz += scnprintf(buf+sz, PAGE_SIZE - sz, "  size: %d\n",
				info->args[i].size);
		sz += scnprintf(buf+sz, PAGE_SIZE - sz, "  offset: 0x%x\n",
				info->args[i].offset);
		sz += scnprintf(buf+sz, PAGE_SIZE - sz, "  direction: %s\n",
				dir);
	}

	if (sz < PAGE_SIZE - 1)
		buf[sz++] = 0;
	else
		buf[PAGE_SIZE - 1] = 0;


	return sz;
}

ssize_t show_formatted_cu_stat(struct xrt_cu *xcu, char *buf)
{
	ssize_t sz = 0;
	char *fmt = "%lld %d\n";
	char *fmt1 = "average queue length: %llu ";
	char *fmt2 = "max running cmds: %llu ";
	char *fmt3 = "cmds runned total: %llu";
	char *fmt4 = "cmds runned since last: %llu ";
	int in_flight = xcu->num_sq;
	u64 average_sq_len;
	u64 incremental_ecmd_count;

	average_sq_len = get_average_sq(xcu);
	incremental_ecmd_count = get_incre_ecmd_count(xcu);

	sz += scnprintf(buf+sz, PAGE_SIZE - sz, fmt, xcu->cu_stat.usage, in_flight);
	sz += scnprintf(buf+sz, PAGE_SIZE - sz, fmt1, average_sq_len);
	sz += scnprintf(buf+sz, PAGE_SIZE - sz, fmt2, xcu->max_running);
	sz += scnprintf(buf+sz, PAGE_SIZE - sz, fmt3, xcu->usage_curr);
	sz += scnprintf(buf+sz, PAGE_SIZE - sz, fmt4, incremental_ecmd_count);

	return sz;
}

void compare_time(u64 *time)
{
	u64 counter = 1000000;
	int i;

	*time = ktime_to_ns(ktime_get());
	// printk("ktime start: %llu", *time);
	for(i = 0; i < counter; i++) {
		*time = ktime_to_ns(ktime_get());
	}
	*time = ktime_to_ns(ktime_get());
	// printk("ktime end: %llu", *time);

	*time = get_cycles();
	// printk("rdstc start counter: %llu", *time);
	for(i = 0; i < counter; i++) {
		*time = get_cycles();
	}
	*time = get_cycles();
	// printk("rdtsc end counter: %llu", *time);
}

int init_cu_time(struct xrt_cu *xcu)
{
	int err = 0;
	// u64 time;

	xrt_cu_get_time(&xcu->last_timestamp);
	//compare_time(&time);

	return err; 
}

int init_ecmd_count(struct xrt_cu *xcu)
{
	int err = 0;

	xcu->last_cmd_time_total = 0;
	xcu->cmd_time_total = 0;
	xcu->newest_cmd_start_ts = 0;
	xcu->last_xcmd_end_time = 0;
	xcu->sq_total = 0;
	xcu->sq_count = 0;

	//printk("init cmd time total: %llu", xcu->cmd_time_total);

	return err;
}

void incr_ecmd_count(struct xrt_cu *xcu)
{
	xcu->usage_curr += 1;
}

void reset_sq_count(struct xrt_cu *xcu, u64 sq_barrer)
{
	xcu->sq_total = 0;
	xcu->sq_count = 0;
	xcu->sq_barrer = sq_barrer;
}

void incr_sq_count(struct xrt_cu *xcu)
{
	xcu->sq_total += xcu->num_sq;
	xcu->sq_count += 1;
	xcu->last_sq_time = xcu->sq_time;
}

void update_ecmd_time(struct xrt_cu *xcu, u32 start_time, u32 end_time)
{
	//printk("start, end time in xcmd: %u, %u", start_time, end_time);
	// assume ert works at 250 Mhz
	//printk("last cmd time total: %llu", xcu->cmd_time_total);
	xcu->cmd_time_total += (end_time - start_time) * 4;
	xrt_cu_get_time(&xcu->last_xcmd_end_time);
	// end lock
	//printk("xcmd usage: %u", (end_time-start_time));
	//printk("update cmd time total(ns): %llu", xcu->cmd_time_total);
}

ssize_t show_cu_busy(struct xrt_cu *xcu, char *buf)
{
	ssize_t sz = 0;
	char *fmt1 = "cu idx: %d ,";
	char *fmt2 = "busy percentage since last measurement: %llu\r\n";
	u64 new_ts;
	u64 cu_stats;
	u64 delta_cmd_time_total;
	u64 delta_xcu_time;
	u64 last_cmd_time_total;
	u64 newest_cmd_start_ts;
	
	last_cmd_time_total = xcu->last_xcmd_end_time;
	newest_cmd_start_ts = xcu->newest_cmd_start_ts;
	xrt_cu_get_time(&new_ts);
	delta_cmd_time_total = (xcu->cmd_time_total - xcu->last_cmd_time_total);
	delta_xcu_time = new_ts - xcu->last_timestamp_cu_busy;

	if (delta_cmd_time_total > 0) {
		/* between two measurement, at least one cmd has completed*/
		if (xcu->num_sq) {
			/* another cmds submitted, check if the new cmd is behind the last cmd end time*/
			if (last_cmd_time_total < newest_cmd_start_ts) {
				compensate_count(delta_cmd_time_total, &xcu->time_adjust, 250);
				cu_stats = (delta_cmd_time_total - xcu->time_adjust + new_ts - newest_cmd_start_ts) * 100 / delta_xcu_time;
				xcu->time_adjust = new_ts - newest_cmd_start_ts; 
				// this should minus - the start time of cmd just after the completed cmd. Or there is no problem because the below branch. 
			} else {
				/* no data from ert for the rest running commands, assume it is running*/
				compensate_count(delta_cmd_time_total, &xcu->time_adjust, 250);
				cu_stats = (delta_cmd_time_total - xcu->time_adjust + new_ts - last_cmd_time_total) * 100 / delta_xcu_time;
				xcu->time_adjust = new_ts - last_cmd_time_total;  
			}
		}
		else {
			compensate_count(delta_cmd_time_total, &xcu->time_adjust, 250);
			cu_stats = (delta_cmd_time_total - xcu->time_adjust) * 100 / delta_xcu_time;
			xcu->time_adjust = 0;
		}
	} else if (delta_cmd_time_total == 0) {
		if (xcu->num_sq) {
		/* no update from ert, but cu is running */
			cu_stats = 100;
			if(newest_cmd_start_ts > xcu->last_timestamp_cu_busy) 
				xcu->time_adjust += new_ts - newest_cmd_start_ts;
			else
				xcu->time_adjust += delta_xcu_time;
		}	
		else {
		/* cu is idle */
			cu_stats = 0;
		}
	}
	xcu->last_cmd_time_total = xcu->cmd_time_total;
	xcu->last_timestamp_cu_busy = new_ts;
	sz += scnprintf(buf+sz, PAGE_SIZE - sz, fmt1,
			xcu->info.cu_idx);
	sz += scnprintf(buf+sz, PAGE_SIZE - sz, fmt2,
			cu_stats);
	sz += scnprintf(buf+sz, PAGE_SIZE - sz, "delta_cmd_time_total:   %llu ",
			delta_cmd_time_total);
	sz += scnprintf(buf+sz, PAGE_SIZE - sz, "new time adjust:   %llu \r\n",
			xcu->time_adjust);
	sz += scnprintf(buf+sz, PAGE_SIZE - sz, "delta_xcu_time: %llu \r\n",
			delta_xcu_time);
	// sz += scnprintf(buf+sz, PAGE_SIZE - sz, "last_xcmd_end_time:  %llu \r\n",
	// 		last_cmd_time_total);
	// sz += scnprintf(buf+sz, PAGE_SIZE - sz, "newest_cmd_start_ts: %llu \r\n",
	// 		newest_cmd_start_ts);
	// sz += scnprintf(buf+sz, PAGE_SIZE - sz, "time_adjust: %llu \r\n",
	// 		xcu->time_adjust);
	return sz;
}

ssize_t show_cu_idle(struct xrt_cu *xcu, char *buf)
{
	ssize_t   sz = 0;
	char      *fmt1 = "cu idx: %d ,";
	u64       new_ts;
	u64       delta_xcu_time;
	u64       delta_idle_time;
	u64       cu_idle;
	
	u64       idle_start;
	u64       last_read_idle_start;
	u64       ts_status;

	idle_start = xcu->idle_start;
	last_read_idle_start = xcu->last_read_idle_start;
	delta_idle_time = xcu->idle_total - xcu->last_idle_total;
	xrt_cu_get_time(&new_ts);
	delta_xcu_time = new_ts - xcu->last_timestamp;

	if(xcu->num_pq == 0 && xcu->num_rq == 0 && xcu->num_sq == 0) {
		if (xcu->last_ts_status) {
			cu_idle = delta_idle_time + new_ts - idle_start;
		} else {
			if (delta_idle_time == 0) {
				cu_idle = delta_xcu_time;
			} else {
				cu_idle = delta_idle_time - (xcu->last_timestamp - last_read_idle_start) + (new_ts - idle_start); 
			}
		}
		ts_status = 0;
	} else if (xcu->num_pq > 0 || xcu->num_rq > 0 || xcu->num_sq > 0) {
		if (xcu->last_ts_status) {
			cu_idle = delta_idle_time;
		} else {
			cu_idle = delta_idle_time - (xcu->last_timestamp - last_read_idle_start);
		}
		ts_status = 1;
	}

	cu_idle = cu_idle * 100 / delta_xcu_time;
	xcu->last_read_idle_start = idle_start;
	xcu->last_idle_total = xcu->idle_total;
	xcu->last_timestamp = new_ts;
	xcu->last_ts_status = ts_status;

	sz += scnprintf(buf+sz, PAGE_SIZE - sz, fmt1, xcu->info.cu_idx);

	sz += scnprintf(buf+sz, PAGE_SIZE - sz, " current idle percentage %llu \r\n", cu_idle);
	sz += scnprintf(buf+sz, PAGE_SIZE - sz, "delta_idle_time:   %llu \r\n",
		delta_idle_time);
	sz += scnprintf(buf+sz, PAGE_SIZE - sz, "delta_xcu_time:   %llu \r\n",
		delta_xcu_time);

	return sz;
}

u64 get_incre_ecmd_count(struct xrt_cu *xcu)
{
	u64 incre_ecmds = 0;

	incre_ecmds = xcu->usage_curr - xcu->usage_prev;
	xcu->usage_prev = xcu->usage_curr;

	return incre_ecmds;
}

u64 get_average_sq(struct xrt_cu *xcu)
{
	u64 average_sq_len = 0;
	
	average_sq_len = xcu->sq_total / xcu->sq_count;
	// the number means every 0.01 seconds update the sq count
	reset_sq_count(xcu, 10000000);

	return average_sq_len;
}

void xrt_cu_get_time(u64 *time)
{
	*time = ktime_to_ns(ktime_get());
	// printk("ktime: %llu", *time);
	// *time = get_cycles();
	// printk("get cycle time: %llu", *time);
}

void compensate_count(u64 count, u64 *offset, u32 freq)
{
	int i = 0;
	u64 one_overflow = 4294967295 * (1000 / freq);

	while ( *offset > one_overflow) {
		*offset -= one_overflow;
		i += 1;
	}
	// if (count < *offset) {
	// 	*offset = 0;
	// }
}