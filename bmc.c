/*
 * BMC
 * 	+----------+     +-------------------+
 * 	| I2C host *---->* BMC   I2C (1,2)   *-----*--------*---------*---------*---------+
 * 	+----------+     |       GPIO A (PA) |     |        |         |         |         |
 * 	                 |       GPIO B (PB) |  +--*---+ +--*---+ +---*---+ +---*---+ +---*---+
 * 	                 |       GPIO C (PC) |  | 0x48 | | 0x49 | | 0x10  | | 0x1F  | | 0x69  |
 * 	                 |       GPIO D (PD) |  | LTC  | | LTC  | | TPS54 | | TPS54 | | TPS65 |
 * 	                 |       Temp 1      |  +------+ +------+ +-------+ +-------+ +-------+
 * 	                 |       Temp 2      |
 * 	                 | (Reg) Reset       |
 * 	                 | (Reg) FRAM        |
 * 	                 | (Reg) MAC1-3      |
 * 	                 +-------------------+
 *
 * There are 3 I2C's connected to BMC: 2 of them are masters.
 * From software, they seems like one: read and write transmissions operate through BMC 0x0, 0x1 and 0x2 registers.
 * 0x0 register trigger the transmission.
 *
 * Temp [1-2] are equal to ltc 0x1a and 0x1b registers. The measure is divided in two parts: 9 integer and 4 fractional.
 * 12-0 bits are valid. 1 step is 0.0625 C. To get a value in Celsius, division on 16 must be done.
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/gpio.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <linux/sizes.h>

#define DRV_VER "1.3"

#define MAC1	1
#define MAC2	2
#define MAC3	3

#define FRU_SIZE	SZ_256

#define BMC_REG_UPD		0x8
#define BMC_REG_RESET		0x18
#define BMC_REG_POWER_OFF	0x19
#define BMC_REG_FRAM_MEM	0x1a
#define BMC_REG_MAC1		0x1b
#define BMC_REG_MAC2		0x1c
#define BMC_REG_MAC3		0x1d
#define BMC_REG_STATUS		0x1e
#define BMC_REG_FIMVER		0x1f
#define BMC_REG_PAR		0x22

#define BMC_REG_GPIOA_L		0x28
#define BMC_REG_GPIOA_H		0x29
#define BMC_REG_GPIOB_L		0x30
#define BMC_REG_GPIOB_H		0x31
#define BMC_REG_GPIOC_L		0x32
#define BMC_REG_GPIOC_H		0x33
#define BMC_REG_GPIOD_L		0x34
#define BMC_REG_GPIOD_H		0x35

#define BMC_REG_LTC1_H		0xaa
#define BMC_REG_LTC1_L		0xab
#define BMC_REG_LTC2_H		0x4a
#define BMC_REG_LTC2_L		0x4b


#define BMC_CPC313_PRMT		23
#define BMC_CPC516_PRMT		26

static bool board;
module_param(board, bool, 0);
MODULE_PARM_DESC(board, "0 - for CPC313 boards (default); 1 - for CPC516");

struct bmc_srvc_dt {
	struct i2c_client *client;
	struct device *hwmon_dev;
	wait_queue_head_t wait;
	struct mutex mtx;
	int prmt; // amount of parameters
//	struct gpio_chip chip;
};
static void *bmc;

#define TEMP_M1		1
#define TEMP_M2		2
#define TEMP_PLX	3
#define TEMP_BMC	4

#define VOLT_0p9	0
#define VOLT_0p95	1
#define VOLT_1		2
#define VOLT_1p2	3
#define VOLT_1p5	4
#define VOLT_1p8	5
#define VOLT_2p5	6
#define VOLT_3p3	7
#define VOLT_3p3s	8
#define VOLT_5		9
#define VOLT_5a		10
#define VOLT_12s	11
#define VOLT_M1		12
#define VOLT_M2		13

#define STAT_0p9_V	 0
#define STAT_0p95_V	 1
#define STAT_1p5_V	 2
#define STAT_1p2_V	 3
#define STAT_1p8_V	 4
#define STAT_2p5_V	 5
#define STAT_1_V	 6
#define STAT_5_V	 7
#define STAT_TEMP_M1	 8
#define STAT_VOLT_M1	 9
#define STAT_TEMP_PLX_M1 10
#define STAT_3p3_V	 13
#define STAT_3p3s_V	 14
#define STAT_12s_V	 15
#define STAT_5a_V	 16
#define STAT_TEMP_M2	 18
#define STAT_VOLT_M2	 19
#define STAT_TEMP_BMC_M2 20

static char *stat_names[] = {
	"0.9V", 	"mV",	//0
	"0.95V",	"mV",	//2
	"1.5V", 	"mV",	//4
	"1.2V",		"mV",	//6
	"1.8V",		"mV",	//8
	"2.5V",		"mV",	//10
	"1.0V",		"mV",	//12
	"5V_IN",	"mV",	//14
	"Tmon",		"°C",	//16
	"Vmon",		"mV",	//18

	"Tplx",		"°C",	//20
	NULL, 		NULL,	//22
	NULL, 		NULL,	//24
	"3.3V",		"mV",	//26
	"3.3_stack",	"mV",	//28
	"12_stack",	"mV",	//30
	"5 always",	"mV",	//32
	NULL,		NULL,	//34
	"Tmon",		"°C",	//36
	"Vmon",		"mV",	//38

	"Tbmc",		"°C",	//40

	"DA24.P0",	NULL,	//42
	"DA12.P0",	NULL,	//44
	"DA13.P0",	NULL,	//46
	"DA13.P1",	NULL,	//48
	"DA13.P2",	NULL,	//50
	"DA13.P3",	NULL	//52
};
/*
static int bmc_gpio_get(struct gpio_chip *gpio, unsigned offset)
{
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,5,0) )
	struct bmc_srvc_dt *data = dev_get_drvdata(gpio->dev);
#else
	struct bmc_srvc_dt *data = gpiochip_get_data(gpio);
#endif
	int res, reg;

	reg = 0x20 + (offset >> 3);
	res = i2c_smbus_read_byte_data(data->client, reg);
	return (res < 0) ? res : (res & (1 << (offset & 0x7)));
};

static void bmc_gpio_set(struct gpio_chip *gpio, unsigned offset, int set_val)
{
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,5,0) )
	struct bmc_srvc_dt *data = dev_get_drvdata(gpio->dev);
#else
	struct bmc_srvc_dt *data = gpiochip_get_data(gpio);
#endif
	int res, reg, val;

	reg = 0x20 + (offset >> 3);
	res = i2c_smbus_read_byte_data(data->client, reg);
	if (res < 0)
		return;

	if (set_val)
		val = res | (1 << (offset & 0x7));
	else
		val = res & ~(1 << (offset & 0x7));

	res = i2c_smbus_write_byte_data(data->client, reg, val);
	return;
};
*/


/* --- Auxiliary --- */
/**
 * bmc_read() - read data from device
 * @data: pointer to bmc service data structure
 * @buf: store buffer
 * @len: buffer size
 * @amnt: amount of bytes in the head of buf that treats as cmd
 *
 * Returns a negative errno if some errors rise while reading byte.
 * Otherwise return number of executed messages.
 */
static int bmc_read(struct bmc_srvc_dt *data, u8 *buf, u16 len, u8 amnt)
{
	int res;

	struct i2c_msg msgs[] = {
		{
		 .addr = data->client->addr,
		 .flags = 0,
		 .len = amnt,
		 .buf = buf
		},
		{
		 .addr = data->client->addr,
		 .flags = I2C_M_RD,
		 .len = len,
		 .buf = buf
		}
	};

	res = mutex_lock_interruptible(&data->mtx);
	if (res < 0)
		return res;

	res = i2c_transfer(data->client->adapter, msgs, 2);
	mutex_unlock(&data->mtx);
	return res;
};

/**
 * bmc_write() - write data to device
 * @data: pointer to bmc service data structure
 * @buf: array of values that must be written
 * @len: array size
 *
 * Returns a negative errno if some errors rise while writing byte.
 * Otherwise return number of executed messages.
 */
static int bmc_write(struct bmc_srvc_dt *data, u8 *buf, u16 len)
{
	int res = 0;

	struct i2c_msg msgs[] = {
		{
		 .addr = data->client->addr,
		 .flags = 0,
		 .len = len,
		 .buf = buf
		}
	};

	dev_dbg(&data->client->dev, "%x: writing %x\n", buf[0], buf[1]);
	res = mutex_lock_interruptible(&data->mtx);
	if (res < 0)
		return res;

	res = i2c_transfer(data->client->adapter, msgs, 1);
	mutex_unlock(&data->mtx);
	return res;
};
/* --- Auxiliary --- */


/* --- Sysfs --- */
/* 0-06f/ */
static ssize_t bmc_set_reset(struct device *dev, struct device_attribute *da, const char *ubuf, size_t count)
{
	struct bmc_srvc_dt *data = dev_get_drvdata(dev);
	int res;
	u8 buf[2];

	if ('1' != ubuf[0]) {
		return count;
	};

	buf[0] = BMC_REG_RESET;
	buf[1] = 0x55;

	res = bmc_write(data, buf, 2);
	if (res > 0)
		res = count;

	return res;
};
static DEVICE_ATTR(bmc_reset, S_IWUSR, NULL, bmc_set_reset);

static ssize_t bmc_show_fimver(struct device *dev, struct device_attribute *da, char *ubuf)
{
	struct bmc_srvc_dt *data = dev_get_drvdata(dev);
	int res;
	u8 buf[4];

	buf[0] = BMC_REG_FIMVER;

	res = bmc_read(data, buf, ARRAY_SIZE(buf), 1);
	if (res < 0)
		return res;

	return sprintf(ubuf, "BMC version: %u.%u; Fldr version: %u.%u\n", buf[0], buf[1], buf[2], buf[3]);
};
static DEVICE_ATTR(bmc_version, S_IRUGO, bmc_show_fimver, NULL);

/*
static ssize_t bmc_set_update(struct device *dev, struct device_attribute *da, const char *ubuf, size_t count)
{
	struct bmc_srvc_dt *data = dev_get_drvdata(dev);
	int res;
	u8 buf[2];

	if ('1' != ubuf[0])
		buf[1] = 0xf7;
	else
		buf[1] = 1;

	buf[0] = BMC_REG_UPD;

	res = bmc_write(data, buf, 2);
	if (!res)
		res = count;

	return res;
};
static DEVICE_ATTR(bmc_update, S_IWUSR, NULL, bmc_set_update);
*/
static struct attribute *bmc_reset_attrs[] = {
	&dev_attr_bmc_reset.attr,
	&dev_attr_bmc_version.attr,
//	&dev_attr_bmc_update.attr,

	NULL
};

static const struct attribute_group bmc_reset_group = {
	.attrs = bmc_reset_attrs,
};
/* ------- */

/* 0-06f/dump/ */
static ssize_t bmc_show_stat(struct device *dev, struct device_attribute *da, char *ubuf)
{
	struct bmc_srvc_dt *data = dev_get_drvdata(dev);
	u8 buf[2] = {0};
	int i, j, res;

	for (i = 0, j = 0; i < data->prmt; i += 2) {
		if (stat_names[i] == NULL)
			continue;

		buf[0] = BMC_REG_PAR;
		buf[1] = i / 2;
		res = bmc_read(data, buf, ARRAY_SIZE(buf), 2);
		if (res < 0)
			return res;

		j += sprintf(ubuf + j, " %s = %5d %s\n", stat_names[i], (signed short) ((buf[1] << 8) | buf[0]), stat_names[i + 1]);
	};

	return j;
};
static DEVICE_ATTR(status, S_IRUGO, bmc_show_stat, NULL);

/*
static ssize_t bmc_show_gpio(struct device *dev, struct device_attribute *da, char *ubuf)
{
	struct bmc_srvc_dt *data = dev_get_drvdata(dev);
	int i, j;

	for (i = BMC_REG_GPIOA_L, j = 0; i <= BMC_REG_GPIOD_H; i++)
	{
		char buf[2];
		int res;
		buf[0] = i;

		res = bmc_read(data, buf, 2, 1);
		if (res < 0)
			return 0;

		j += scnprintf(ubuf + j, 4, "%x\n", (u8)buf[1]);
		dev_dbg(&data->client->dev, "bmc: i:%u j:%u res:%u ret:%x\n", i, j, res, (u8)buf[1]);
	};

	ubuf[j-1] = '\n';
	ubuf[j] = '\0';
	return j;
};
static DEVICE_ATTR(gpio_regs_dump, S_IRUGO, bmc_show_gpio, NULL);
*/

static struct attribute *bmc_dump_attrs[] = {
//	&dev_attr_gpio_regs_dump.attr,
	&dev_attr_status.attr,

	NULL
};

static const struct attribute_group bmc_dump_group = {
	.name = "dump",
	.attrs = bmc_dump_attrs,
};
/* ------- */

/* 0-06f/fram/ */
static ssize_t bmc_show_mem(struct device *dev, struct device_attribute *da, char *ubuf)
{
	struct bmc_srvc_dt *data = dev_get_drvdata(dev);
	u8 buf[FRU_SIZE];

	buf[0] = BMC_REG_FRAM_MEM;
	bmc_read(data, buf, FRU_SIZE, 1);

	memcpy(ubuf, buf, FRU_SIZE);
	return FRU_SIZE;
};

static ssize_t bmc_store_mem(struct device *dev, struct device_attribute *da, const char *ubuf, size_t count)
{
	struct bmc_srvc_dt *data = dev_get_drvdata(dev);
	u8 buf[FRU_SIZE + 1] = {0};

	buf[0] = BMC_REG_FRAM_MEM;
	memcpy(&buf[1], ubuf, count > FRU_SIZE ? FRU_SIZE : count);
	bmc_write(data, buf, ARRAY_SIZE(buf));

	return count;
};
static DEVICE_ATTR(mem, S_IRUGO | S_IWUSR, bmc_show_mem, bmc_store_mem);

static ssize_t bmc_show_mac(struct device *dev, struct device_attribute *da, char *ubuf)
{
	struct bmc_srvc_dt *data = dev_get_drvdata(dev);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	int i, j, res;
	u8 buf[6];

	switch (attr->index) {
	case MAC1:
		buf[0] = BMC_REG_MAC1;
		break;
	case MAC2:
		buf[0] = BMC_REG_MAC2;
		break;
	case MAC3:
		buf[0] = BMC_REG_MAC3;
		break;
	default:
		return -EINVAL;
	};

	res = bmc_read(data, buf, ARRAY_SIZE(buf), 1);
	if (res < 0)
		return res;

	for (i = 0, j = 0; i < ARRAY_SIZE(buf); i++)
		j += scnprintf(ubuf + j, 4, "%02x:", buf[i]);

	ubuf[j-1] = '\n';
	ubuf[j] = '\0';
	return j;
};

static ssize_t bmc_store_mac(struct device *dev, struct device_attribute *da, const char *ubuf, size_t count)
{
	struct bmc_srvc_dt *data = dev_get_drvdata(dev);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	int i, j, res;
	u8 buf[7];

	if (count != 18)
		return -EINVAL;

	switch (attr->index) {
	case MAC1:
		buf[0] = BMC_REG_MAC1;
		break;
	case MAC2:
		buf[0] = BMC_REG_MAC2;
		break;
	case MAC3:
		buf[0] = BMC_REG_MAC3;
		break;
	default:
		return -EINVAL;
	};

	for (i = 1, j = 0; i < ARRAY_SIZE(buf); i++, j += 3)
	{
		char val[3];
		val[0] = *(ubuf + j);
		val[1] = *(ubuf + j + 1);
		val[2] = '\0';

		if ((res = kstrtou8(val, 16, &buf[i])) < 0)
			return res;
	};

	res = bmc_write(data, buf, ARRAY_SIZE(buf));
	if (res < 0)
		return res;

	return count;
};
static SENSOR_DEVICE_ATTR(mac1, S_IRUGO | S_IWUSR, bmc_show_mac, bmc_store_mac, MAC1);
static SENSOR_DEVICE_ATTR(mac2, S_IRUGO | S_IWUSR, bmc_show_mac, bmc_store_mac, MAC2);
static SENSOR_DEVICE_ATTR(mac3, S_IRUGO | S_IWUSR, bmc_show_mac, bmc_store_mac, MAC3);

static struct attribute *bmc_fram_attrs[] = {
	&dev_attr_mem.attr,
	&sensor_dev_attr_mac1.dev_attr.attr,
	&sensor_dev_attr_mac2.dev_attr.attr,
	&sensor_dev_attr_mac3.dev_attr.attr,

	NULL
};

static const struct attribute_group bmc_fram_group = {
	.name = "fram",
	.attrs = bmc_fram_attrs,
};
/* ------- */

/* 0-06f/hwmon/ */
static ssize_t bmc_ltc_show_temp(struct device *dev, struct device_attribute *da, char *ubuf)
{
	struct bmc_srvc_dt *data = dev_get_drvdata(dev);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	u8 buf[2] = {0};
	int res;

	switch (attr->index) {
	case TEMP_M1:
		buf[1] = STAT_TEMP_M1;
		break;
	case TEMP_M2:
		buf[1] = STAT_TEMP_M2;
		break;
	case TEMP_PLX:
		buf[1] = STAT_TEMP_PLX_M1;
		break;
	case TEMP_BMC:
		buf[1] = STAT_TEMP_BMC_M2;
		break;
	default:
		dev_err(dev, "Invalid index number: %d", attr->index);
		return -EINVAL;
	};

	buf[0] = BMC_REG_PAR;
	res = bmc_read(data, buf, 2, 2);
	if (res < 0)
		return res;

	res = (signed short) ((buf[1] << 8) | buf[0]);
	return sprintf(ubuf, "%d °C\n", res);
};
static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO, bmc_ltc_show_temp, NULL, TEMP_M1);
static SENSOR_DEVICE_ATTR(temp2_input, S_IRUGO, bmc_ltc_show_temp, NULL, TEMP_M1);
static SENSOR_DEVICE_ATTR(temp_plx, S_IRUGO, bmc_ltc_show_temp, NULL, TEMP_PLX);
static SENSOR_DEVICE_ATTR(temp_bmc, S_IRUGO, bmc_ltc_show_temp, NULL, TEMP_BMC);

static ssize_t bmc_show_volt(struct device *dev, struct device_attribute *da, char *ubuf)
{
	struct bmc_srvc_dt *data = dev_get_drvdata(dev);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	u8 buf[2] = {0};
	int res;

	switch(attr->index) {
	case VOLT_0p9:
		buf[1] = STAT_0p9_V;
		break;
	case VOLT_0p95:
		buf[1] = STAT_0p95_V;
		break;
	case VOLT_1:
		buf[1] = STAT_1_V;
		break;
	case VOLT_1p2:
		buf[1] = STAT_1p2_V;
		break;
	case VOLT_1p5:
		buf[1] = STAT_1p5_V;
		break;
	case VOLT_1p8:
		buf[1] = STAT_1p8_V;
		break;
	case VOLT_2p5:
		buf[1] = STAT_2p5_V;
		break;
	case VOLT_3p3:
		buf[1] = STAT_3p3_V;
		break;
	case VOLT_3p3s:
		buf[1] = STAT_3p3s_V;
		break;
	case VOLT_5:
		buf[1] = STAT_5_V;
		break;
	case VOLT_5a:
		buf[1] = STAT_5a_V;
		break;
	case VOLT_12s:
		buf[1] = STAT_12s_V;
		break;
	case VOLT_M1:
		buf[1] = STAT_VOLT_M1;
		break;
	case VOLT_M2:
		buf[1] = STAT_VOLT_M2;
		break;
	default:
		dev_err(dev, "Invalid index number: %d", attr->index);
		return -EINVAL;
	}

	buf[0] = BMC_REG_PAR;
	res = bmc_read(data, buf, 2, 2);
	if (res < 0)
		return res;

	res = (signed short) ((buf[1] << 8) | buf[0]);
	return sprintf(ubuf, "%d mV\n", res);

};
static SENSOR_DEVICE_ATTR(volt_0p9, 	S_IRUGO, bmc_show_volt, NULL, VOLT_0p9);
static SENSOR_DEVICE_ATTR(volt_0p95, 	S_IRUGO, bmc_show_volt, NULL, VOLT_0p95);
static SENSOR_DEVICE_ATTR(volt_1, 	S_IRUGO, bmc_show_volt, NULL, VOLT_1);
static SENSOR_DEVICE_ATTR(volt_1p2, 	S_IRUGO, bmc_show_volt, NULL, VOLT_1p2);
static SENSOR_DEVICE_ATTR(volt_1p5, 	S_IRUGO, bmc_show_volt, NULL, VOLT_1p5);
static SENSOR_DEVICE_ATTR(volt_1p8, 	S_IRUGO, bmc_show_volt, NULL, VOLT_1p8);
static SENSOR_DEVICE_ATTR(volt_2p5, 	S_IRUGO, bmc_show_volt, NULL, VOLT_2p5);
static SENSOR_DEVICE_ATTR(volt_3p3, 	S_IRUGO, bmc_show_volt, NULL, VOLT_3p3);
static SENSOR_DEVICE_ATTR(volt_3p3s, 	S_IRUGO, bmc_show_volt, NULL, VOLT_3p3s);
static SENSOR_DEVICE_ATTR(volt_5,	S_IRUGO, bmc_show_volt, NULL, VOLT_5);
static SENSOR_DEVICE_ATTR(volt_5a, 	S_IRUGO, bmc_show_volt, NULL, VOLT_5a);
static SENSOR_DEVICE_ATTR(volt_12s, 	S_IRUGO, bmc_show_volt, NULL, VOLT_12s);
static SENSOR_DEVICE_ATTR(volt_mon1, 	S_IRUGO, bmc_show_volt, NULL, VOLT_M1);
static SENSOR_DEVICE_ATTR(volt_mon2, 	S_IRUGO, bmc_show_volt, NULL, VOLT_M2);

static struct attribute *bmc_hwmon_attrs[] = {
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_temp2_input.dev_attr.attr,
	&sensor_dev_attr_temp_plx.dev_attr.attr,
	&sensor_dev_attr_temp_bmc.dev_attr.attr,
	&sensor_dev_attr_volt_0p9.dev_attr.attr,
	&sensor_dev_attr_volt_0p95.dev_attr.attr,
	&sensor_dev_attr_volt_1.dev_attr.attr,
	&sensor_dev_attr_volt_1p2.dev_attr.attr,
	&sensor_dev_attr_volt_1p5.dev_attr.attr,
	&sensor_dev_attr_volt_1p8.dev_attr.attr,
	&sensor_dev_attr_volt_2p5.dev_attr.attr,
	&sensor_dev_attr_volt_3p3.dev_attr.attr,
	&sensor_dev_attr_volt_3p3s.dev_attr.attr,
	&sensor_dev_attr_volt_5.dev_attr.attr,
	&sensor_dev_attr_volt_5a.dev_attr.attr,
	&sensor_dev_attr_volt_12s.dev_attr.attr,
	&sensor_dev_attr_volt_mon1.dev_attr.attr,
	&sensor_dev_attr_volt_mon2.dev_attr.attr,
	
	NULL
};
ATTRIBUTE_GROUPS(bmc_hwmon);
/* --- Sysfs --- */


static int bmc_notify(struct notifier_block *this, unsigned long code, void *unused)
{
	struct bmc_srvc_dt *data = (struct bmc_srvc_dt *)bmc;

	if (code == SYS_RESTART) {
		u8 buf[2] = {BMC_REG_RESET, 0x55};
		bmc_write(data, buf, ARRAY_SIZE(buf));
	};

	if ((code == SYS_POWER_OFF) || (code == SYS_HALT)) {
		u8 buf[2] = {BMC_REG_POWER_OFF, 0x55};
		bmc_write(data, buf, ARRAY_SIZE(buf));
	};

	return NOTIFY_DONE;
};

static struct notifier_block bmc_notifier = {
	.notifier_call = bmc_notify,
};


/* --- Probe/Remove --- */
static int bmc_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int res;
	struct bmc_srvc_dt *data;
	u8 buf[4];
	
	data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
	if (data == NULL) {
		res = -ENOMEM;
		dev_err(&client->dev, "Can't allocate mem for data\n");
		goto bip_err;
	};

/*
	data->chip.label = "bmc_gpio";
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,5,0) )
	data->chip.dev = &client->dev;
#else
	data->chip.parent = &client->dev;
#endif
	data->chip.owner = THIS_MODULE;
	data->chip.get = bmc_gpio_get;
	data->chip.set = bmc_gpio_set;
	data->chip.base = -1;
	data->chip.ngpio = 8;
	data->chip.can_sleep = false;
//	data->chip.request = gpiochip_generic_request;
//	data->chip.free	= gpiochip_generic_free;
*/

	data->client = client;
	if (!board)
		data->prmt = BMC_CPC313_PRMT;
	else
		data->prmt = BMC_CPC516_PRMT;

	bmc = (void *)data;
	i2c_set_clientdata(client, data);
/*
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,6,0) )
	res = gpiochip_add(&data->chip);
#else
	res = devm_gpiochip_add_data(&client->dev, &data->chip, data);
#endif
	if (res < 0)
		goto err;
*/
	res = sysfs_create_group(&client->dev.kobj, &bmc_dump_group);
	if (res < 0)
		goto bip_unreg_gchip;

	res = sysfs_create_group(&client->dev.kobj, &bmc_reset_group);
	if (res < 0)
		goto bip_delete_sysfs_dump;

	res = sysfs_create_group(&client->dev.kobj, &bmc_fram_group);
	if (res < 0)
		goto bip_delete_sysfs_reset;

	res = register_reboot_notifier(&bmc_notifier);
	if (res) {
		dev_err(&client->dev, "Cannot register reboot notifier (err=%d)\n", res);
		goto bip_delete_sysfs_fram;
	};

	data->hwmon_dev = devm_hwmon_device_register_with_groups(&client->dev, "bmc_hwmon", data, bmc_hwmon_groups);
	if (IS_ERR(data->hwmon_dev)) {
		res = PTR_ERR(data->hwmon_dev);
		goto bip_unreg_notifier;
	};

	init_waitqueue_head(&data->wait);
	mutex_init(&data->mtx);

	buf[0] = BMC_REG_FIMVER;
	res = bmc_read(data, buf, ARRAY_SIZE(buf), 1);
	if (res < 0)
		goto bip_unreg_notifier;

	if ((buf[0] == 4 && buf[1] >= 19)
	 || ((buf[0] == 5) && buf[1] <= 3))
		dev_info(&client->dev, "BMC driver successfully loaded. BMC firmware %u.%u", buf[0], buf[1]);
	else {
		dev_warn(&client->dev, " was not being tested with BMC firmware %u.%u!", buf[0], buf[1]);
	}

	return 0;

bip_unreg_notifier:
	unregister_reboot_notifier(&bmc_notifier);
bip_delete_sysfs_fram:
	sysfs_remove_group(&client->dev.kobj, &bmc_fram_group);
bip_delete_sysfs_reset:
	sysfs_remove_group(&client->dev.kobj, &bmc_reset_group);
bip_delete_sysfs_dump:
	sysfs_remove_group(&client->dev.kobj, &bmc_dump_group);
bip_unreg_gchip:
/*
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,6,0) )
	gpiochip_remove(&data->chip);
#else
	devm_gpiochip_remove(&client->dev, &data->chip);
#endif
*/
bip_err:
	return res;
}



static int bmc_i2c_remove(struct i2c_client *client)
{
	struct bmc_srvc_dt *data = i2c_get_clientdata(client);

/*
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(4,6,0) )
	gpiochip_remove(&data->chip);
#endif
*/
	sysfs_remove_group(&client->dev.kobj, &bmc_fram_group);
	sysfs_remove_group(&data->client->dev.kobj, &bmc_reset_group);
	sysfs_remove_group(&data->client->dev.kobj, &bmc_dump_group);
	unregister_reboot_notifier(&bmc_notifier);
	return 0;
};

/* --- Probe/Remove --- */


/* --- Init data --- */
#ifdef CONFIG_OF
static const struct of_device_id bmc_dt_match[] = {
	{ .compatible = "bmc" },
	{ },
};
#endif

static const struct i2c_device_id bmc_i2c_id[] = {
	{ "bmc", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, bmc_i2c_id);

static struct i2c_driver bmc_i2c_drv = {
	.driver 	= {
			.name = "baikal_bmc",
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(bmc_dt_match),
#endif
	},
	.probe 		= bmc_i2c_probe,
	.remove		= bmc_i2c_remove,
	.id_table	= bmc_i2c_id,
};
module_i2c_driver(bmc_i2c_drv);

MODULE_DESCRIPTION("I2C driver for BMC on CPC313 board.");
MODULE_AUTHOR("Karamov Artur <karamov@fastwel.ru>");
MODULE_VERSION(DRV_VER);
MODULE_LICENSE("GPL");
