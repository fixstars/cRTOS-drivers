/*
 * Copyright 2016 Mans Rullgard <mans@mansr.com>
 * Copyright 2019 Jan Kiszka <jan.kiszka@siemens.com>
 * Copyright 2020 ChungFan Yang @ Fixstars corporation
 *                              <chungfan.yang@fixstars.com>
 * Copyright (c) Siemens AG, 2016-2020
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/sched.h>
#include <linux/virtio_ring.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/string.h>
#include <linux/kprobes.h>
#include <linux/poll.h>

#include "ivshmem.h"

/******************************************************************************
 * Pre-processor Marco
******************************************************************************/

#define DRV_NAME                  "shadow-process"

#define SHADOW_MAX_DEVICES        1U

#define SHADOW_MAX_PROCESS        16U

#define SHADOW_STATE_RESET        0
#define SHADOW_STATE_INIT         1
#define SHADOW_STATE_READY        2
#define SHADOW_STATE_RUN          3

#define SHADOW_FLAG_RUN           0

#define SHADOW_FRAME_SIZE(s) ALIGN(18 + (s), SMP_CACHE_BYTES)

#define SHADOW_REGION_RX          3
#define SHADOW_REGION_TX          4

#define SHADOW_VQ_ALIGN           64
#define SHADOW_MTU                512

#define SHADOW_VECTOR_STATE        0
#define SHADOW_VECTOR_TX_RX        1
#define SHADOW_NUM_VECTORS         2

#define SPDRBASE 'k'
#define SPDR_TCB _IOR(SPDRBASE, 1, u64)
#define SPDR_DEL _IOW(SPDRBASE, 2, u64)
#define SPDR_SIG _IOW(SPDRBASE, 8, u64)

/******************************************************************************
 * Private Types
******************************************************************************/

static struct pci_driver shadow_pci_driver;
struct shadow_device;

struct shadow_info {
    const char *name;
    const char *version;
};

struct shadow_mem {
    const char      *name;
    phys_addr_t     paddr;
    void            *addr;
    unsigned long   offs;
    resource_size_t size;
    bool            readonly;
};

struct shadow_process_item {
    u64 nbr;
    u64 arguments[6];
    u64 nuttx_tcb;
    u64 nuttx_prio;

    u64 linux_tcb;
    wait_queue_head_t poll_lock;
    atomic_t has_msg;
    struct shadow_device *dev;
};
// {nbr, 1~6, recved nuttx tcb addr, linux tcb addr}

struct shadow_queue {
    struct vring vr;
    u32 free_head;
    u32 num_free;
    u32 num_added;
    u16 last_avail_idx;
    u16 last_used_idx;

    void *data;
    void *end;
    u32 size;
    u32 head;
    u32 tail;
};

struct shadow_device {
    struct pci_dev *pdev;
    struct device   dev;
    struct cdev     cdev;

    struct ivshm_regs __iomem *regs;
    int peer_id;
    int vectors;

    struct mutex       info_lock;
    struct shadow_info info;
    int                minor;

    struct mutex       list_lock;
    struct shadow_process_item process_list[SHADOW_MAX_PROCESS];

    struct shadow_mem mem[5];

    struct shadow_queue rx;
    struct shadow_queue tx;

    u32 vrsize;
    u32 qlen;
    u32 qsize;

    u32 state;
    u32 last_peer_state;
    u32 *state_table;

    u32 tx_rx_vector;

    unsigned long flags;

    struct workqueue_struct *state_wq;
    struct work_struct state_work;

    struct workqueue_struct *recv_wq;
    struct work_struct recv_work;
};

/******************************************************************************
 * Private Function Prototypes
******************************************************************************/

static int shadow_calc_qsize(struct shadow_device* dev);
static void shadow_init_queue(struct shadow_device *dev,
                 struct shadow_queue *q,
                 void *mem, unsigned int len);
static void shadow_init_queues(struct shadow_device *dev);

static void shadow_notify_tx(struct shadow_device *dev, unsigned int num, u64 prio);
static void shadow_enable_rx_irq(struct shadow_device *dev);

static void *shadow_desc_data(struct shadow_device *dev,
                              struct shadow_queue *q,
                              unsigned int region,
                              struct vring_desc *desc,
                              u32 *len);
static struct vring_desc *shadow_rx_desc(struct shadow_device *dev);
static void shadow_rx_finish(struct shadow_device *dev, struct vring_desc *desc);

static size_t shadow_tx_space(struct shadow_device *dev);
static bool shadow_tx_ok(struct shadow_device *dev);
static u32 shadow_tx_advance(struct shadow_queue *q, u32 *pos, u32 len);
static bool shadow_tx_clean(struct shadow_device *dev);

static void shadow_transmit(struct shadow_device* dev, uint64_t* data, uint64_t prio);
static void shadow_receive(struct shadow_device* dev, uint64_t* data);

static void shadow_set_state(struct shadow_device *dev, u32 state);
static void shadow_check_state(struct shadow_device *dev);
static void shadow_state_change(struct work_struct *work);

static void shadow_run(struct shadow_device *dev);
static void shadow_do_stop(struct shadow_device *dev);

static void shadow_recv_handler(struct work_struct *work);

static irqreturn_t shadow_int_state(int irq, void *data);
static irqreturn_t shadow_int_tx_rx(int irq, void *data);

// fops
static int     shadow_open(struct inode *inodep, struct file *filep);
static int     shadow_release(struct inode *inodep, struct file *filep);
static long    shadow_ioctl(struct file *filep, unsigned int cmd, unsigned long arg);
static ssize_t shadow_read(struct file *filep, char __user *addr, size_t len, loff_t *off);
static ssize_t shadow_write(struct file *filep, const char __user *addr, size_t len, loff_t *off);
static unsigned int shadow_poll(struct file *filep, poll_table *wait);
static int shadow_mmap(struct file *filep, struct vm_area_struct *vma);

static int shadow_pci_probe(struct pci_dev *pdev,
                            const struct pci_device_id *pci_id);


static int shadow_major_init(void);
static void shadow_major_cleanup(void);
static int shadow_get_minor(struct shadow_device *dev);
static void shadow_free_minor(struct shadow_device *dev);
static int __init shadow_init(void);
static void __exit shadow_exit(void);

/******************************************************************************
 * Private Data and getters
******************************************************************************/

ulong shmaddr = 0x0;

static ssize_t name_show(struct device *dev,
                         struct device_attribute *attr, char *buf)
{
    struct shadow_device *idev = dev_get_drvdata(dev);
    int ret;

    mutex_lock(&idev->info_lock);

    ret = sprintf(buf, "%s\n", idev->info.name);

    mutex_unlock(&idev->info_lock);
    return ret;
}
static DEVICE_ATTR_RO(name);

static ssize_t version_show(struct device *dev,
                            struct device_attribute *attr, char *buf)
{
    struct shadow_device *idev = dev_get_drvdata(dev);
    int ret;

    mutex_lock(&idev->info_lock);

    ret = sprintf(buf, "%s\n", idev->info.version);

    mutex_unlock(&idev->info_lock);
    return ret;
}
static DEVICE_ATTR_RO(version);

static ssize_t event_show(struct device *dev,
                          struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%u\n", 0);
}
static DEVICE_ATTR_RO(event);

// SYSFS CLASS related
static int          shadow_major;
static struct cdev *shadow_cdev;

static struct attribute *shadow_attrs[] = {
  &dev_attr_name.attr,
  &dev_attr_version.attr,
  &dev_attr_event.attr,
  NULL,
};
ATTRIBUTE_GROUPS(shadow);

/* SHADOW class infrastructure */
static struct class shadow_class = {
    .name = "shadow-process",
    .dev_groups = shadow_groups,
};

static bool shadow_class_registered;

static DEFINE_MUTEX(minor_lock);
static DEFINE_IDR(shadow_idr);

// Character device ops
static struct file_operations shadow_fops =
{
   .open           = shadow_open,
   .release        = shadow_release,
   .unlocked_ioctl = shadow_ioctl,
   .read           = shadow_read,
   .write          = shadow_write,
   .poll           = shadow_poll,
   .mmap           = shadow_mmap,
};

static const struct vm_operations_struct
shadow_physical_vm_ops = {
#ifdef CONFIG_HAVE_IOREMAP_PROT
    .access = generic_access_phys,
#endif
};

/***************************************
 * Queue Helpers
 ***************************************/

static int shadow_calc_qsize(struct shadow_device* dev)
{
    unsigned int vrsize;
    unsigned int qsize;
    unsigned int qlen;

    for (qlen = 4096; qlen > 32; qlen >>= 1) {
        vrsize = vring_size(qlen, SHADOW_VQ_ALIGN);
        vrsize = ALIGN(vrsize, SHADOW_VQ_ALIGN);
        if (vrsize < (dev->mem[SHADOW_REGION_TX].size) / 8)
            break;
    }

    if (vrsize > dev->mem[SHADOW_REGION_TX].size)
        return -EINVAL;

    qsize = dev->mem[SHADOW_REGION_TX].size - vrsize;

    if (qsize < 4 * SHADOW_MTU)
        return -EINVAL;

    dev->vrsize = vrsize;
    dev->qlen = qlen;
    dev->qsize = qsize;

    return 0;
}

static void shadow_init_queue(struct shadow_device *dev,
                 struct shadow_queue *q,
                 void *mem, unsigned int len)
{
    memset(q, 0, sizeof(*q));

    vring_init(&q->vr, len, mem, SHADOW_VQ_ALIGN);
    q->data = mem + dev->vrsize;
    q->end = q->data + dev->qsize;
    q->size = dev->qsize;
}

static void shadow_init_queues(struct shadow_device *dev)
{
    int i;
    void *tx = (void*)dev->mem[SHADOW_REGION_TX].addr;
    void *rx = (void*)dev->mem[SHADOW_REGION_RX].addr;

    memset(tx, 0, dev->mem[SHADOW_REGION_TX].size);

    shadow_init_queue(dev, &dev->tx, tx, dev->qlen);
    shadow_init_queue(dev, &dev->rx, rx, dev->qlen);

    swap(dev->rx.vr.used, dev->tx.vr.used);

    dev->tx.num_free = dev->tx.vr.num;

    for (i = 0; i < dev->tx.vr.num - 1; i++)
        dev->tx.vr.desc[i].next = i + 1;
}

/***************************************
 * IRQ Helpers
 ***************************************/

static void shadow_notify_tx(struct shadow_device *dev, unsigned int num, u64 prio)
{
    u16 evt, old, new;
    u64 remote_prio;

    virt_mb();

    evt = READ_ONCE(vring_avail_event(&dev->tx.vr));
    old = dev->tx.last_avail_idx - num;
    new = dev->tx.last_avail_idx;
    remote_prio = *(volatile uint64_t*)(dev->mem[SHADOW_REGION_RX].addr + dev->mem[SHADOW_REGION_RX].size);

    /* only send IPI if necessary */
    if (remote_prio <= prio){
      writel(dev->tx_rx_vector | (dev->peer_id << 16),
             &dev->regs->doorbell);
    }
}

static void shadow_enable_rx_irq(struct shadow_device *dev)
{
    vring_avail_event(&dev->rx.vr) = dev->rx.last_avail_idx;
    virt_wmb();
}

/***************************************
 * Syscall request processer
 ***************************************/

static void *shadow_desc_data(struct shadow_device *dev,
                              struct shadow_queue *q,
                              unsigned int region,
                              struct vring_desc *desc,
                              u32 *len)
{
    u64 offs = READ_ONCE(desc->addr);
    u32 dlen = READ_ONCE(desc->len);
    u16 flags = READ_ONCE(desc->flags);
    void *data;

    if (flags)
        return NULL;

    if (offs >= dev->mem[region].size)
        return NULL;

    data = (void*)(dev->mem[region].addr + offs);

    if (data < q->data || data >= q->end)
        return NULL;

    if (dlen > q->end - data)
        return NULL;

    *len = dlen;

    return data;
}

static struct vring_desc *shadow_rx_desc(struct shadow_device *dev)
{
    struct shadow_queue *rx = &dev->rx;
    struct vring *vr = &rx->vr;
    unsigned int avail;
    u16 avail_idx;

    avail_idx = virt_load_acquire(&vr->avail->idx);

    if (avail_idx == rx->last_avail_idx)
        return NULL;

    avail = vr->avail->ring[rx->last_avail_idx++ & (vr->num - 1)];
    if (avail >= vr->num) {
        printk("invalid rx avail %d\n", avail);
        return NULL;
    }

    return &vr->desc[avail];
}

static void shadow_rx_finish(struct shadow_device *dev, struct vring_desc *desc)
{
    struct shadow_queue *rx = &dev->rx;
    struct vring *vr = &rx->vr;
    unsigned int desc_id = desc - vr->desc;
    unsigned int used;

    used = rx->last_used_idx++ & (vr->num - 1);
    vr->used->ring[used].id = desc_id;
    vr->used->ring[used].len = 1;

    virt_store_release(&vr->used->idx, rx->last_used_idx);
}

static size_t shadow_tx_space(struct shadow_device *dev)
{
    struct shadow_queue *tx = &dev->tx;
    u32 tail = tx->tail;
    u32 head = tx->head;
    u32 space;

    if (head < tail)
        space = tail - head;
    else
        space = max(tx->size - head, tail);

    return space;
}

static bool shadow_tx_ok(struct shadow_device *dev)
{
    return dev->tx.num_free >= 2 &&
           shadow_tx_space(dev) >= 2 * SHADOW_FRAME_SIZE(SHADOW_MTU);
}

static u32 shadow_tx_advance(struct shadow_queue *q, u32 *pos, u32 len)
{
    u32 p = *pos;

    len = SHADOW_FRAME_SIZE(len);

    if (q->size - p < len)
        p = 0;
    *pos = p + len;

    return p;
}

static bool shadow_tx_clean(struct shadow_device *dev)
{
    struct shadow_queue *tx = &dev->tx;
    struct vring_used_elem *used;
    struct vring *vr = &tx->vr;
    struct vring_desc *desc;
    struct vring_desc *fdesc;
    u16 last = tx->last_used_idx;
    unsigned int num;
    bool tx_ok;
    u32 fhead;

    fdesc = NULL;
    fhead = 0;
    num = 0;

    while (last != virt_load_acquire(&vr->used->idx)) {
        void *data;
        u32 len;
        u32 tail;

        used = vr->used->ring + (last % vr->num);
        if (used->id >= vr->num || used->len != 1) {
            printk("invalid tx used->id %d ->len %d\n",
                   used->id, used->len);
            break;
        }

        desc = &vr->desc[used->id];

        data = shadow_desc_data(dev, &dev->tx, SHADOW_REGION_TX,
                       desc, &len);
        if (!data) {
            printk("bad tx descriptor, data == NULL\n");
            break;
        }

        tail = shadow_tx_advance(tx, &tx->tail, len);
        if (data != tx->data + tail) {
            printk("bad tx descriptor\n");
            break;
        }

        if (!num)
            fdesc = desc;
        else
            desc->next = fhead;

        tx->last_used_idx = ++last;
        num++;
        tx->num_free++;
        BUG_ON(tx->num_free > vr->num);

        tx_ok = shadow_tx_ok(dev);
    }

    if (num) {
        fdesc->next = tx->free_head;
        tx->free_head = fhead;
    } else {
        tx_ok = shadow_tx_ok(dev);
    }

    return tx_ok;
}

static void shadow_transmit(struct shadow_device* dev, uint64_t* data, uint64_t prio)
{
    struct shadow_queue *tx = &dev->tx;
    struct vring *vr = &tx->vr;
    struct vring_desc *desc;
    unsigned int desc_idx;
    unsigned int avail;
    u32 head;
    u32 len = sizeof(u64) * 2;
    void* buf;

    if (!shadow_tx_clean(dev)) {
      BUG();
    }

    desc_idx = tx->free_head;
    desc = &vr->desc[desc_idx];
    tx->free_head = desc->next;
    tx->num_free--;

    head = shadow_tx_advance(tx, &tx->head, len);

    buf = tx->data + head;
    *(u64*)buf = *data;
    *((u64*)buf + 1) = *(data + 1);

    desc->addr = buf - (void*)dev->mem[SHADOW_REGION_TX].addr;
    desc->len = len;
    desc->flags = 0;

    avail = tx->last_avail_idx++ & (vr->num - 1);
    vr->avail->ring[avail] = desc_idx;
    tx->num_added++;

    virt_store_release(&vr->avail->idx, tx->last_avail_idx);

    shadow_notify_tx(dev, tx->num_added, prio);

    tx->num_added = 0;
}

static void shadow_receive(struct shadow_device* dev, uint64_t* data)
{
    struct vring_desc *desc;
    u64 *buf;
    int len;

    desc = shadow_rx_desc(dev);
    if (!desc)
        return;

    buf = (u64*)shadow_desc_data(dev, &dev->rx, SHADOW_REGION_RX,
                   desc, &len);
    if (!buf) {
        printk("bad rx descriptor\n");
        return;
    }

    memcpy(data, buf, sizeof(u64) * 10);

    shadow_rx_finish(dev, desc);
}

/***************************************
 * Connection State machine
 ***************************************/

static void shadow_set_state(struct shadow_device *dev, u32 state)
{
    virt_wmb();
    WRITE_ONCE(dev->state, state);
    writel(state, &dev->regs->state);
}

static void shadow_check_state(struct shadow_device *dev)
{
    if (dev->state_table[dev->peer_id] != dev->last_peer_state ||
        !test_bit(SHADOW_FLAG_RUN, &dev->flags))
        queue_work(dev->state_wq, &dev->state_work);
}

static void shadow_state_change(struct work_struct *work)
{
    struct shadow_device *dev = container_of(work, struct shadow_device, state_work);
    u32 peer_state = READ_ONCE(dev->state_table[dev->peer_id]);

    switch (dev->state) {
        case SHADOW_STATE_RESET:
          /*
           * Wait for the remote to leave READY/RUN before transitioning
           * to INIT.
           */
          if (peer_state < SHADOW_STATE_READY)
            shadow_set_state(dev, SHADOW_STATE_INIT);
          break;

        case SHADOW_STATE_INIT:
          /*
           * Wait for the remote to leave RESET before performing the
           * initialization and moving to READY.
           */
          if (peer_state > SHADOW_STATE_RESET) {
            shadow_init_queues(dev);
            shadow_set_state(dev, SHADOW_STATE_READY);
          }
          break;

        case SHADOW_STATE_READY:
          /*
           * Link is up and we are running once the remote is in READY or
           * RUN.
           */
          if (peer_state >= SHADOW_STATE_READY) {
            shadow_run(dev);
            break;
          }
          /* fall through */
        case SHADOW_STATE_RUN:
          /*
           * If the remote goes to RESET, we need to follow immediately.
           */
          if (peer_state == SHADOW_STATE_RESET) {
              shadow_do_stop(dev);
          }
          break;
    }

    virt_wmb();
    WRITE_ONCE(dev->last_peer_state, peer_state);
}

static void shadow_run(struct shadow_device *dev)
{
    if (dev->state < SHADOW_STATE_READY)
        return;

    if (test_and_set_bit(SHADOW_FLAG_RUN, &dev->flags))
        return;

    shadow_set_state(dev, SHADOW_STATE_RUN);
    shadow_enable_rx_irq(dev);
}

static void shadow_do_stop(struct shadow_device *dev)
{
    shadow_set_state(dev, SHADOW_STATE_RESET);

    if (!test_and_clear_bit(SHADOW_FLAG_RUN, &dev->flags))
        return;
}

/***************************************
 * Recv processing
 ***************************************/

static void shadow_recv_handler(struct work_struct *work)
{
    struct shadow_device *dev = container_of(work, struct shadow_device, recv_work);
    u64 buf[10];

    // received data {nbr, arguments 1~6, nuttx tcb addr, process_list idx}
    shadow_receive(dev, buf);

    if (dev->process_list[buf[9]].linux_tcb) {

        // Copy the payload only
        memcpy(&dev->process_list[buf[9]], buf, sizeof(u64) * 9);

        // Has message
        atomic_set(&dev->process_list[buf[9]].has_msg, 1);

        wake_up_interruptible_sync(&dev->process_list[buf[9]].poll_lock);
    }else{
        printk("FATAL: SYSCALL without target to wakeup\n");
    }
}

/***************************************
 * IRQ handler
 ***************************************/

static irqreturn_t shadow_int_state(int irq, void *data)
{
    struct shadow_device *dev = data;

    shadow_check_state(dev);

    return IRQ_HANDLED;
}

static irqreturn_t shadow_int_tx_rx(int irq, void *data)
{
    struct shadow_device *dev = data;

    shadow_recv_handler(&dev->recv_work);

    return IRQ_HANDLED;
}

/******************************************************************************
 * Fops
******************************************************************************/

static int shadow_open(struct inode *inodep, struct file *filep)
{
    struct shadow_device *dev;
    int i;

    mutex_lock(&minor_lock);
    dev = idr_find(&shadow_idr, iminor(inodep));
    mutex_unlock(&minor_lock);
    if (!dev)
      return -ENODEV;

    get_device(&dev->dev);

    for (i = 0; i < SHADOW_MAX_PROCESS; i++){
        if (dev->process_list[i].linux_tcb == (uint64_t)current){
            return -EINVAL;
        }
    }

    mutex_lock(&dev->list_lock);

    for (i = 0; i < SHADOW_MAX_PROCESS; i++){
        if (dev->process_list[i].linux_tcb == 0){
            memset(&dev->process_list[i], 0, sizeof(dev->process_list[i]));

            dev->process_list[i].linux_tcb = (uint64_t)current;
            atomic_set(&dev->process_list[i].has_msg, 0);
            dev->process_list[i].dev = dev;

            init_waitqueue_head(&dev->process_list[i].poll_lock);

            printk("Process: %llx got slot: %d\n",
                   dev->process_list[i].linux_tcb, i);
            break;
        }
    }

    filep->private_data = dev->process_list + i;

    mutex_unlock(&dev->list_lock);

    put_device(&dev->dev);

    return 0;
}

static int shadow_release(struct inode *inodep, struct file *filep)
{
    struct shadow_process_item* item = filep->private_data;

    // XXX: Send a signal to there

    item->linux_tcb = 0;

    return 0;
}

static long shadow_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
    struct shadow_process_item* this_entry = (struct shadow_process_item*) filep->private_data;
    struct shadow_device *dev = this_entry->dev;
    u64 i;
    u64 buf[2];

    if (_IOC_TYPE(cmd) != SPDRBASE)
        return -EINVAL;

    switch(cmd) {
        case SPDR_TCB:
            i = ((struct shadow_process_item*) filep->private_data - dev->process_list);
            copy_to_user((uint64_t*)arg, (uint64_t*)&i, sizeof(u64));
            return 0;
        case SPDR_SIG:
            buf[0] = arg; // The arg is the signal number
            buf[1] = (1ULL << 63) | current->pid;
            shadow_transmit(dev, buf, this_entry->nuttx_prio);
            return 0;
        default:
            printk("Unknown ioctl command to %p", dev);
            return -EINVAL;
    }
}


static ssize_t shadow_read(struct file *filep, char __user *addr, size_t len, loff_t *off)
{
    struct shadow_process_item* this_entry = (struct shadow_process_item*) filep->private_data;
    int ret;

    ret = wait_event_interruptible(this_entry->poll_lock,
                                   (unsigned long)atomic_read(&this_entry->has_msg));

    if (ret < 0) {
        if (fatal_signal_pending(current)) {
           // Must be SIGKILL or SIGSTOP
           printk("Oh oh, %p was killed!\n", current);
        }
        return -EINTR;
    }

    copy_to_user(addr, this_entry, sizeof(u64) * 9);

    atomic_set(&this_entry->has_msg, 0);

    return sizeof(u64) * 9;
}


static ssize_t shadow_write(struct file *filep, const char __user *addr, size_t len, loff_t *off)
{
    struct shadow_process_item* this_entry = (struct shadow_process_item*) filep->private_data;
    struct shadow_device *dev = this_entry->dev;
    u64 buf[2];

    copy_from_user(buf, addr, sizeof(u64));

    buf[1] = this_entry->nuttx_tcb;

    shadow_transmit(dev, buf, this_entry->nuttx_prio);

    return len;
}

static unsigned int shadow_poll(struct file *filep, poll_table *wait)
{
    struct shadow_process_item* this_entry = (struct shadow_process_item*) filep->private_data;
    unsigned int mask = POLLOUT;

    if ((unsigned long)atomic_read(&this_entry->has_msg))
        mask |= POLLIN;

    poll_wait(filep, &this_entry->poll_lock, wait);

    return mask;
}

static int shadow_mmap(struct file *filep, struct vm_area_struct *vma)
{
    struct shadow_process_item* this_entry = (struct shadow_process_item*) filep->private_data;
    struct shadow_device *dev = this_entry->dev;
    unsigned long requested_pages, actual_pages;
    int ret = 0;

    /* Mmap maps the rw_region */

    if (vma->vm_end < vma->vm_start)
        return -EINVAL;

    vma->vm_private_data = dev;

    mutex_lock(&dev->info_lock);

    requested_pages = vma_pages(vma);
    actual_pages = (((ulong)dev->mem[2].addr & ~PAGE_MASK)
        + dev->mem[2].size + PAGE_SIZE -1) >> PAGE_SHIFT;
    if (requested_pages > actual_pages) {
      ret = -EINVAL;
      goto out;
    }

    vma->vm_ops = &shadow_physical_vm_ops;

    ret = remap_pfn_range(vma,
                          vma->vm_start,
                          ((ulong)dev->mem[2].paddr >> PAGE_SHIFT) + vma->vm_pgoff,
                          vma->vm_end - vma->vm_start,
                          vma->vm_page_prot);

out:
    mutex_unlock(&dev->info_lock);
    return ret;
}

/***************************************
 * Probe and Remove
 ***************************************/

static u64 get_config_qword(struct pci_dev *pdev, unsigned int pos)
{
    u32 lo, hi;

    pci_read_config_dword(pdev, pos, &lo);
    pci_read_config_dword(pdev, pos + 4, &hi);
    return lo | ((u64)hi << 32);
}

static void shadow_device_release(struct device *dev)
{
}

static int shadow_pci_probe(struct pci_dev *pdev,
                            const struct pci_device_id *pci_id)
{
    resource_size_t rw_section_sz, output_section_sz;
    struct shadow_device *dev;
    phys_addr_t section_addr;
    int err, vendor_cap;
    unsigned int cap_pos;
    struct shadow_mem *mem;
    char *device_name;
    char *tx_rx_irq_name;
    u32 dword;

    dev = devm_kzalloc(&pdev->dev, sizeof(struct shadow_device),
                                 GFP_KERNEL);
    if (!dev)
        return -ENOMEM;

    dev->pdev = pdev;
    pci_set_drvdata(pdev, dev);

    err = pcim_enable_device(pdev);
    if (err) {
        dev_err(&pdev->dev, "pci_enable_device: %d\n", err);
        return err;
    }

    device_name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "%s[%s]", DRV_NAME,
                                 dev_name(&pdev->dev));
    if (!device_name)
        return -ENOMEM;

    dev->info.name = device_name;
    dev->info.version = "1";

    err = pcim_iomap_regions(pdev, BIT(0), device_name);
    if (err) {
        dev_err(&pdev->dev, "pcim_iomap_regions: %d\n", err);
        return err;
    }
    dev->regs = pcim_iomap_table(pdev)[0];

    if(readl(&dev->regs->max_peers) != 2)
        return -EINVAL;

    dev->peer_id = !readl(&dev->regs->id);

    mem = &dev->mem[0];

    mem->name = "registers";
    mem->paddr = pci_resource_start(pdev, 0);
    if (!mem->paddr)
      return -ENODEV;
    mem->size = pci_resource_len(pdev, 0);

    vendor_cap = pci_find_capability(pdev, PCI_CAP_ID_VNDR);
    if (vendor_cap < 0) {
        dev_err(&pdev->dev, "missing vendor capability\n");
        return -ENODEV;
    }

    if (pci_resource_len(pdev, 2) > 0) {
      section_addr = pci_resource_start(pdev, 2);
    } else {
      cap_pos = vendor_cap + IVSHM_CFG_ADDRESS;
      section_addr = get_config_qword(pdev, cap_pos);
    }

    mem++;
    mem->name = "state_table";
    mem->paddr = section_addr;
    cap_pos = vendor_cap + IVSHM_CFG_STATE_TAB_SZ;
    pci_read_config_dword(pdev, cap_pos, &dword);
    mem->size = dword;
    mem->readonly = true;
    if (!devm_request_mem_region(&pdev->dev, mem->paddr, mem->size,
                                 device_name))
        return -EBUSY;

    mem->addr = dev->state_table =
      devm_memremap(&pdev->dev, mem->paddr, mem->size, MEMREMAP_WB);
    if (!mem->addr)
        return -ENOMEM;

    dev_info(&pdev->dev, "%s at %pa, size %pa\n", mem->name, &mem->paddr,
       &mem->size);

    cap_pos = vendor_cap + IVSHM_CFG_RW_SECTION_SZ;
    rw_section_sz = get_config_qword(pdev, cap_pos);
    if (rw_section_sz <= 0)
        return -EINVAL;

    section_addr += mem->size;

    mem++;
    mem->name = "rw_section";
    mem->paddr = shmaddr;
    mem->size = rw_section_sz;
    if (!devm_request_mem_region(&pdev->dev, mem->paddr, mem->size,
                                 device_name))
        return -EBUSY;

    dev_info(&pdev->dev, "%s at %pa, size %pa\n", mem->name,
             &mem->paddr, &mem->size);

    cap_pos = vendor_cap + IVSHM_CFG_OUTPUT_SECTION_SZ;
    output_section_sz = get_config_qword(pdev, cap_pos);
    if (output_section_sz < 0x2000)
        return -EINVAL;

    mem++;
    mem->name = "input_sections";
    mem->paddr = section_addr + dev->peer_id * output_section_sz;
    mem->size = output_section_sz - 0x1000;
    mem->readonly = true;
    if (!devm_request_mem_region(&pdev->dev, mem->paddr, mem->size + 0x1000,
                                 device_name))
        return -EBUSY;

    mem->addr = devm_memremap(&pdev->dev, mem->paddr, mem->size + 0x1000, MEMREMAP_WB);
    if (!mem->addr)
        return -ENOMEM;

    dev_info(&pdev->dev, "%s at %pa, size %pa\n", mem->name,
             &mem->paddr, &mem->size);

    mem++;
    mem->name = "output_section";
    mem->paddr = section_addr + !(dev->peer_id) * output_section_sz;
    mem->size = output_section_sz - 0x1000;
    if (!devm_request_mem_region(&pdev->dev, mem->paddr, mem->size + 0x1000,
                                 device_name))
        return -EBUSY;

    mem->addr = devm_memremap(&pdev->dev, mem->paddr, mem->size + 0x1000, MEMREMAP_WB);
    if (!mem->addr)
        return -ENOMEM;

    dev_info(&pdev->dev, "%s at %pa, size %pa\n", mem->name,
             &mem->paddr, &mem->size);

    memset(mem->addr, 0, mem->size + 0x1000);

    pci_write_config_byte(pdev, vendor_cap + IVSHM_CFG_PRIV_CNTL,
                          IVSHM_PRIV_CNTL_ONESHOT_INT);

    dev->vectors = SHADOW_NUM_VECTORS;

    /* register at sysfs */

    if (!shadow_class_registered) {
        err = -EPROBE_DEFER;
        goto error;
    }

    mutex_init(&dev->info_lock);

    err = shadow_get_minor(dev);
    if (err)
        goto error;

    device_initialize(&dev->dev);
    dev->dev.devt = MKDEV(shadow_major, dev->minor);
    dev->dev.class = &shadow_class;
    dev->dev.parent = &pdev->dev;
    dev->dev.release = shadow_device_release;
    dev_set_drvdata(&dev->dev, dev);

    err = dev_set_name(&dev->dev, "shadow-process%d", dev->minor);
    if (err)
        goto err_device_create;

    err = device_add(&dev->dev);
    if (err)
        goto err_device_create;

    /*err = uio_dev_add_attributes(dev);*/
    /*if (err)*/
        /*goto err_uio_dev_add_attributes;*/

    // Init process List
    mutex_init(&dev->list_lock);
    memset(dev->process_list, 0, sizeof(dev->process_list));

    err = shadow_calc_qsize(dev);
    if (err)
        goto err_qsize;

    dev->state_wq = alloc_ordered_workqueue(device_name, 0);
    if (!dev->state_wq)
        goto err_queue;
    INIT_WORK(&dev->state_work, shadow_state_change);

    dev->recv_wq  = create_singlethread_workqueue("shadow-recv");
    if (!dev->recv_wq)
        goto err_recv_queue;
    INIT_WORK(&dev->recv_work, shadow_recv_handler);

    err = pci_alloc_irq_vectors(pdev, dev->vectors,
                                dev->vectors,
                                PCI_IRQ_LEGACY | PCI_IRQ_MSIX);
    if (err != SHADOW_NUM_VECTORS) {
        err = -EBUSY;
        goto err_request_irqs;
    }

    if (!pdev->msix_enabled) {
        err = -EBUSY;
        goto err_request_irqs;
    }

    err = request_any_context_irq(pci_irq_vector(pdev, SHADOW_VECTOR_STATE),
                                  shadow_int_state, 0, device_name, dev);
    if (err)
        goto err_request_state_irq;

    tx_rx_irq_name = devm_kasprintf(&pdev->dev, GFP_KERNEL,
                              "%s-tx-rx[%s]", DRV_NAME,
                              dev_name(&pdev->dev));
    if (!tx_rx_irq_name) {
        err = -ENOMEM;
        goto err_request_state_irq;
    }

    err = request_any_context_irq(pci_irq_vector(pdev, SHADOW_VECTOR_TX_RX),
                                  shadow_int_tx_rx, 0, tx_rx_irq_name, dev);
    if (err)
        goto err_request_tx_rx_irq;

    dev->tx_rx_vector = SHADOW_VECTOR_TX_RX;

    pci_set_master(pdev);

    pci_write_config_byte(pdev, vendor_cap + IVSHM_CFG_PRIV_CNTL, 0);
    writel(IVSHM_INT_ENABLE, &dev->regs->int_control);

    writel(SHADOW_STATE_RESET, &dev->regs->state);
    writel(SHADOW_STATE_RESET, &dev->state);
    shadow_check_state(dev);

    *(volatile uint64_t*)(dev->mem[SHADOW_REGION_TX].addr + dev->mem[SHADOW_REGION_TX].size) = 0;
    virt_wmb();

    return 0;

err_request_tx_rx_irq:
    free_irq(pci_irq_vector(pdev, SHADOW_VECTOR_STATE), dev);
err_request_state_irq:
err_request_irqs:
    pci_free_irq_vectors(pdev);
err_recv_queue:
    destroy_workqueue(dev->recv_wq);
err_queue:
    destroy_workqueue(dev->state_wq);
err_qsize:
    /*uio_dev_del_attributes(idev);*/
/*err_uio_dev_add_attributes:*/
    device_del(&dev->dev);
err_device_create:
    shadow_free_minor(dev);
    put_device(&dev->dev);
error:
    return err;
}

static void shadow_pci_remove(struct pci_dev *pdev)
{

    struct shadow_device *dev = pci_get_drvdata(pdev);
    int i;

    writel(0, &dev->regs->int_control);
    pci_clear_master(pdev);

    shadow_free_minor(dev);

    /*mutex_lock(&dev->info_lock);*/
    /*uio_dev_del_attributes(idev);*/
    /*mutex_unlock(&dev->info_lock);*/

    for (i = 0; i < SHADOW_MAX_PROCESS; i++){
        if (dev->process_list[i].linux_tcb != 0){
            wake_up_interruptible(&dev->process_list[i].poll_lock);
        }
    }

    device_del(&dev->dev);
    put_device(&dev->dev);

    writel(SHADOW_STATE_RESET, &dev->regs->state);
    writel(SHADOW_STATE_RESET, &dev->state);

    for (i = 0; i < dev->vectors; i++)
        free_irq(pci_irq_vector(pdev, i), dev);

    pci_free_irq_vectors(pdev);

    cancel_work_sync(&dev->state_work);
    destroy_workqueue(dev->state_wq);
    cancel_work_sync(&dev->recv_work);
    destroy_workqueue(dev->recv_wq);

    return;
}

/* For capturing SIGKILL and SIGSTOP */
/*static struct kprobe kp = {*/
    /*.symbol_name    = "send_signal",*/
/*};*/

/*static int handler_pre(struct kprobe *p, struct pt_regs *regs)*/
/*{*/
    /*int sig = regs->di;*/
    /*struct task_struct* t = regs->dx;*/

    /*int i;*/

    /*for (i = 0; i < 128; i++) {*/
        /*if (process_list[i].linux_tcb == t) {*/
            /*[> these 2 are tricky, we handle them here <]*/
            /*if (sig == SIGKILL || sig == SIGSTOP) {*/
                /*printk("%llx signaled %llx with %d\n", current, t, sig);*/

                /*u64 buf[2];*/
                /*buf[0] = sig;*/
                /*buf[1] = (1ULL << 63) | t->pid;*/

                /*shadow_transmit(process_list[i].in, buf, process_list[i].nuttx_prio);*/
            /*}*/
        /*}*/
    /*}*/

    /*return 0;*/
/*}*/

/*static void handler_post(struct kprobe *p, struct pt_regs *regs,*/
                /*unsigned long flags)*/
/*{*/
    /*return;*/
/*}*/

/*static int handler_fault(struct kprobe *p, struct pt_regs *regs, int trapnr)*/
/*{*/
    /*return 0;*/
/*}*/

/******************************************************************************
 * Initialization Functions
******************************************************************************/

static int shadow_major_init(void)
{
    static const char name[] = DRV_NAME;
    struct cdev *cdev = NULL;
    dev_t shadow_dev = 0;
    int result;

    result = alloc_chrdev_region(&shadow_dev, 0, SHADOW_MAX_DEVICES, name);
    if (result)
        goto out;

    result = -ENOMEM;
    cdev = cdev_alloc();
    if (!cdev)
        goto out_unregister;

    cdev->owner = THIS_MODULE;
    cdev->ops = &shadow_fops;
    kobject_set_name(&cdev->kobj, "%s", name);

    result = cdev_add(cdev, shadow_dev, SHADOW_MAX_DEVICES);
    if (result)
        goto out_put;

    shadow_major = MAJOR(shadow_dev);
    shadow_cdev = cdev;
    return 0;
out_put:
    kobject_put(&cdev->kobj);
out_unregister:
    unregister_chrdev_region(shadow_dev, SHADOW_MAX_DEVICES);
out:
    return result;
}

static void shadow_major_cleanup(void)
{
    unregister_chrdev_region(MKDEV(shadow_major, 0), SHADOW_MAX_DEVICES);
    cdev_del(shadow_cdev);
}

static int shadow_get_minor(struct shadow_device *dev)
{
    int ret = -ENOMEM;

    mutex_lock(&minor_lock);
    ret = idr_alloc(&shadow_idr, dev, 0, SHADOW_MAX_DEVICES, GFP_KERNEL);
    if (ret >= 0) {
        dev->minor = ret;
        ret = 0;
    } else if (ret == -ENOSPC) {
        dev_err(&dev->dev, "too many shadow devices\n");
        ret = -EINVAL;
    }
    mutex_unlock(&minor_lock);
    return ret;
}

static void shadow_free_minor(struct shadow_device *dev)
{
    mutex_lock(&minor_lock);
    idr_remove(&shadow_idr, dev->minor);
    mutex_unlock(&minor_lock);
}

static int __init shadow_init(void)
{
    //Drivers entry function. register with the pci core and spawn class
    int ret;

    /*kp.pre_handler = handler_pre;*/
    /*kp.post_handler = handler_post;*/
    /*kp.fault_handler = handler_fault;*/

    /*ret = register_kprobe(&kp);*/

    /* This is the first time in here, set everything up properly */
    ret = shadow_major_init();
    if (ret)
        goto exit;

    ret = class_register(&shadow_class);
    if (ret) {
        printk(KERN_ERR "class_register failed for shadow process\n");
        goto err_class_register;
    }

    shadow_class_registered = true;

    ret = pci_register_driver(&shadow_pci_driver);
    if (ret < 0){
        printk(KERN_ERR "In %s pci_register_driver FAILED\n", __FUNCTION__);
        goto err_class_register;
    }

    return 0;

err_class_register:
    shadow_major_cleanup();
exit:
    return ret;
}

static void __exit shadow_exit(void)
{
    //Drivers exit function. Unregister everything
    shadow_class_registered = false;
    class_unregister(&shadow_class);
    shadow_major_cleanup();
    pci_unregister_driver(&shadow_pci_driver);
    /*unregister_kprobe(&kp);*/
}

/******************************************************************************
 * Kernel Module Definitions
******************************************************************************/

module_param(shmaddr, ulong, 0);

module_init(shadow_init);
module_exit(shadow_exit);

static struct pci_device_id shadow_pci_ids[] = {
    { PCI_DEVICE(PCI_VENDOR_ID_REDHAT_QUMRANET, 0x1110),
        (PCI_CLASS_OTHERS << 16) | (0xffff), 0xffff00 },
    { PCI_DEVICE(PCI_VENDOR_ID_SIEMENS, PCI_DEVICE_ID_IVSHMEM),
        (PCI_CLASS_OTHERS << 16) | IVSHM_PROTO_SHADOW, 0xffffff },
    { 0 }
};

static struct pci_driver shadow_pci_driver = {
    .name       = DRV_NAME,
    .id_table   = shadow_pci_ids,
    .probe      = shadow_pci_probe,
    .remove     = shadow_pci_remove,
};

MODULE_DEVICE_TABLE(pci, shadow_pci_ids);
MODULE_AUTHOR("Yang Chung-Fan (sonic.tw.tp@gmail.com)");
MODULE_LICENSE("GPL v2");
