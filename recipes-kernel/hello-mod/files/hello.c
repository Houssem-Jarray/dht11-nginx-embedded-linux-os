#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/version.h>
#include <linux/uaccess.h>
#include <linux/module.h>  // for module macros

#define DEVICE_NAME "hello"

static int major;
static struct class *hello_class;
static struct cdev hello_cdev;

static int hello_open(struct inode *inode, struct file *file) {
    printk(KERN_INFO "hello: device opened\n");
    return 0;
}

static int hello_release(struct inode *inode, struct file *file) {
    printk(KERN_INFO "hello: device closed\n");
    return 0;
}

static ssize_t hello_read(struct file *file, char __user *buf, size_t count, loff_t *ppos) {
    const char *msg = "Hello from kernel!\n";
    ssize_t ret = simple_read_from_buffer(buf, count, ppos, msg, strlen(msg));
    printk(KERN_INFO "hello: read %zd bytes\n", ret);
    return ret;
}

static ssize_t hello_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos) {
    char kbuf[128] = {0};
    size_t to_copy = min(count, sizeof(kbuf) - 1);

    if (copy_from_user(kbuf, buf, to_copy)) {
        printk(KERN_ERR "hello: failed to copy data from user\n");
        return -EFAULT;
    }

    printk(KERN_INFO "hello: received from user: %s\n", kbuf);
    return count;
}

static const struct file_operations hello_fops = {
    .owner = THIS_MODULE,
    .open = hello_open,
    .release = hello_release,
    .read = hello_read,
    .write = hello_write,
};

static int __init hello_init(void) {
    printk(KERN_INFO "hello: initializing module\n");

    major = register_chrdev(0, DEVICE_NAME, &hello_fops);
    if (major < 0) {
        printk(KERN_ERR "hello: failed to register char device\n");
        return major;
    }
    printk(KERN_INFO "hello: registered with major number %d\n", major);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0)
    hello_class = class_create("hello");
#else
    hello_class = class_create(THIS_MODULE, "hello");
#endif
    if (IS_ERR(hello_class)) {
        unregister_chrdev(major, DEVICE_NAME);
        printk(KERN_ERR "hello: failed to create device class\n");
        return PTR_ERR(hello_class);
    }
    printk(KERN_INFO "hello: device class created\n");

    if (device_create(hello_class, NULL, MKDEV(major, 0), NULL, DEVICE_NAME) == NULL) {
        class_destroy(hello_class);
        unregister_chrdev(major, DEVICE_NAME);
        printk(KERN_ERR "hello: failed to create device\n");
        return -1;
    }
    printk(KERN_INFO "hello: device created successfully\n");

    return 0;
}

static void __exit hello_exit(void) {
    printk(KERN_INFO "hello: unloading module\n");
    device_destroy(hello_class, MKDEV(major, 0));
    class_destroy(hello_class);
    unregister_chrdev(major, DEVICE_NAME);
    printk(KERN_INFO "hello: module unloaded\n");
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("Simple Hello Kernel Module with /dev/hello");

module_init(hello_init);
module_exit(hello_exit);
