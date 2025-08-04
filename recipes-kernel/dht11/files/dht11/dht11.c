#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/version.h>

#define GPIO_PIN 4 // Use GPIO 4 for DHT11
#define DEVICE_NAME "dht11"
#define CLASS_NAME "dht"

#define GPIO_BASE_PHYS 0xFE200000 // Physical base address for GPIO (Raspberry Pi 4)
#define GPIO_SIZE 0x100

#define GPFSEL_OFFSET 0x00
#define GPSET_OFFSET 0x1C
#define GPCLR_OFFSET 0x28
#define GPLEV_OFFSET 0x34

static dev_t dev_num;
static struct cdev dht11_cdev;
static struct class *dht11_class;

static void __iomem *gpio_base;

static void gpio_set_input(void)
{
    u32 val = readl(gpio_base + GPFSEL_OFFSET + (GPIO_PIN / 10) * 4);
    val &= ~(0x7 << ((GPIO_PIN % 10) * 3));
    writel(val, gpio_base + GPFSEL_OFFSET + (GPIO_PIN / 10) * 4);
}

static void gpio_set_output(void)
{
    u32 val = readl(gpio_base + GPFSEL_OFFSET + (GPIO_PIN / 10) * 4);
    val &= ~(0x7 << ((GPIO_PIN % 10) * 3));
    val |= (0x1 << ((GPIO_PIN % 10) * 3));
    writel(val, gpio_base + GPFSEL_OFFSET + (GPIO_PIN / 10) * 4);
}

static void gpio_write(int val)
{
    if (val)
        writel(1 << GPIO_PIN, gpio_base + GPSET_OFFSET);
    else
        writel(1 << GPIO_PIN, gpio_base + GPCLR_OFFSET);
}

static int gpio_read(void)
{
    return !!(readl(gpio_base + GPLEV_OFFSET) & (1 << GPIO_PIN));
}

static int wait_for_pin(int value, int timeout_us)
{
    while (gpio_read() != value && timeout_us-- > 0)
        udelay(1);
    return timeout_us > 0 ? 0 : -1;
}

static int dht11_read(u8 data[5])
{
    int i, j;
    u8 byte = 0;

    memset(data, 0, 5);

    // Send start signal
    gpio_set_output();
    gpio_write(0);
    mdelay(20); // At least 18ms
    gpio_write(1);
    udelay(40);
    gpio_set_input();

    // Wait for response from sensor (low, then high)
    if (wait_for_pin(0, 100) < 0)
        return -1;
    if (wait_for_pin(1, 100) < 0)
        return -1;
    if (wait_for_pin(0, 100) < 0)
        return -1;

    // Read 5 bytes
    for (i = 0; i < 5; i++)
    {
        byte = 0;
        for (j = 0; j < 8; j++)
        {
            if (wait_for_pin(1, 100) < 0) // wait for high
                return -1;
            udelay(30);
            byte <<= 1;
            if (gpio_read())
                byte |= 1;
            if (wait_for_pin(0, 100) < 0) // wait for low
                return -1;
        }
        data[i] = byte;
    }

    // Verify checksum
    if ((u8)(data[0] + data[1] + data[2] + data[3]) != data[4])
        return -2;

    return 0;
}

static ssize_t dht11_read_file(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    u8 data[5];
    char result[64];
    int ret;

    ret = dht11_read(data);
    if (ret)
        return -EIO;

    snprintf(result, sizeof(result), "Humidity: %d.%d %%  Temp: %d.%d C\n",
             data[0], data[1], data[2], data[3]);

    return simple_read_from_buffer(buf, count, ppos, result, strlen(result));
}

static struct file_operations fops = {
    .owner = THIS_MODULE,
    .read = dht11_read_file,
};

static int __init dht11_init(void)
{
    int ret;

    gpio_base = ioremap(GPIO_BASE_PHYS, GPIO_SIZE);
    if (!gpio_base)
    {
        pr_err("DHT11: Failed to ioremap GPIO\n");
        return -ENOMEM;
    }

    ret = alloc_chrdev_region(&dev_num, 0, 1, DEVICE_NAME);
    if (ret)
    {
        pr_err("DHT11: Failed to allocate chrdev\n");
        iounmap(gpio_base);
        return ret;
    }

    cdev_init(&dht11_cdev, &fops);
    dht11_cdev.owner = THIS_MODULE;

    ret = cdev_add(&dht11_cdev, dev_num, 1);
    if (ret)
    {
        pr_err("DHT11: Failed to add cdev\n");
        unregister_chrdev_region(dev_num, 1);
        iounmap(gpio_base);
        return ret;
    }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0)
    dht11_class = class_create(DEVICE_NAME);
#else
    dht11_class = class_create(THIS_MODULE, DEVICE_NAME);
#endif
    if (IS_ERR(dht11_class))
    {
        pr_err("DHT11: Failed to create class\n");
        cdev_del(&dht11_cdev);
        unregister_chrdev_region(dev_num, 1);
        iounmap(gpio_base);
        return PTR_ERR(dht11_class);
    }

    device_create(dht11_class, NULL, dev_num, NULL, DEVICE_NAME);

    pr_info("DHT11: Module loaded. Device: /dev/%s\n", DEVICE_NAME);
    return 0;
}

static void __exit dht11_exit(void)
{
    device_destroy(dht11_class, dev_num);
    class_destroy(dht11_class);
    cdev_del(&dht11_cdev);
    unregister_chrdev_region(dev_num, 1);
    if (gpio_base)
        iounmap(gpio_base);
    pr_info("DHT11: Module unloaded.\n");
}

module_init(dht11_init);
module_exit(dht11_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("DHT11 Driver using ioremap on Raspberry Pi 4");
MODULE_VERSION("1.0");