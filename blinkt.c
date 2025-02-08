/*
 * Copyright (C) 2024,2025 Dan Arrhenius <dan@ultramarin.se>
 *
 * This file is part of the blinkt kernel module.
 *
 * The blinkt kernel module is free software; you can redistribute it
 * and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 2 of the License,
 * or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/workqueue.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_device.h>
#include <linux/gpio/consumer.h>
#include <linux/leds.h>
#include <linux/led-class-multicolor.h>

#define NUM_LEDS 8
#define CLK_DELAY 1
#define INIT_ITERATIONS 32
#define COMMIT_ITERATIONS 36

struct blinkt_leds_priv;

struct blinkt_led_data {
	struct led_classdev_mc mc;
	struct blinkt_leds_priv *priv;
	uint32_t *values;
};

struct blinkt_leds_priv {
	struct gpio_desc *clk_pin;
	struct gpio_desc *dat_pin;
	struct work_struct work;
	spinlock_t lock;
	uint32_t led_buffer[NUM_LEDS];
	struct blinkt_led_data leds[NUM_LEDS];
};


static char *prefix = NULL;
module_param(prefix, charp, 0440);
MODULE_PARM_DESC(prefix, "Optional LED name prefix. The name the LEDs will be \"prefix[0-7]\".");

static uint8_t brgb0[4] = {0, 0, 0, 0}; /* brightness, red, green, blue */
static uint8_t brgb1[4] = {0, 0, 0, 0};
static uint8_t brgb2[4] = {0, 0, 0, 0};
static uint8_t brgb3[4] = {0, 0, 0, 0};
static uint8_t brgb4[4] = {0, 0, 0, 0};
static uint8_t brgb5[4] = {0, 0, 0, 0};
static uint8_t brgb6[4] = {0, 0, 0, 0};
static uint8_t brgb7[4] = {0, 0, 0, 0};
static unsigned brgb0_count = 0;
static unsigned brgb1_count = 0;
static unsigned brgb2_count = 0;
static unsigned brgb3_count = 0;
static unsigned brgb4_count = 0;
static unsigned brgb5_count = 0;
static unsigned brgb6_count = 0;
static unsigned brgb7_count = 0;
module_param_array(brgb0, byte, &brgb0_count, 0440);
module_param_array(brgb1, byte, &brgb1_count, 0440);
module_param_array(brgb2, byte, &brgb2_count, 0440);
module_param_array(brgb3, byte, &brgb3_count, 0440);
module_param_array(brgb4, byte, &brgb4_count, 0440);
module_param_array(brgb5, byte, &brgb5_count, 0440);
module_param_array(brgb6, byte, &brgb6_count, 0440);
module_param_array(brgb7, byte, &brgb7_count, 0440);
MODULE_PARM_DESC(brgb0, "Array of max 4 bytes. [brightness, red_intensity, green_intensity, blue_intensity] for LED 0");
MODULE_PARM_DESC(brgb1, "Array of max 4 bytes. [brightness, red_intensity, green_intensity, blue_intensity] for LED 1");
MODULE_PARM_DESC(brgb2, "Array of max 4 bytes. [brightness, red_intensity, green_intensity, blue_intensity] for LED 2");
MODULE_PARM_DESC(brgb3, "Array of max 4 bytes. [brightness, red_intensity, green_intensity, blue_intensity] for LED 3");
MODULE_PARM_DESC(brgb4, "Array of max 4 bytes. [brightness, red_intensity, green_intensity, blue_intensity] for LED 4");
MODULE_PARM_DESC(brgb5, "Array of max 4 bytes. [brightness, red_intensity, green_intensity, blue_intensity] for LED 5");
MODULE_PARM_DESC(brgb6, "Array of max 4 bytes. [brightness, red_intensity, green_intensity, blue_intensity] for LED 6");
MODULE_PARM_DESC(brgb7, "Array of max 4 bytes. [brightness, red_intensity, green_intensity, blue_intensity] for LED 7");

static char *trigger0 = NULL;
static char *trigger1 = NULL;
static char *trigger2 = NULL;
static char *trigger3 = NULL;
static char *trigger4 = NULL;
static char *trigger5 = NULL;
static char *trigger6 = NULL;
static char *trigger7 = NULL;
module_param(trigger0, charp, 0440);
module_param(trigger1, charp, 0440);
module_param(trigger2, charp, 0440);
module_param(trigger3, charp, 0440);
module_param(trigger4, charp, 0440);
module_param(trigger5, charp, 0440);
module_param(trigger6, charp, 0440);
module_param(trigger7, charp, 0440);
MODULE_PARM_DESC(trigger0, "Optional trigger name for LED 0");
MODULE_PARM_DESC(trigger1, "Optional trigger name for LED 1");
MODULE_PARM_DESC(trigger2, "Optional trigger name for LED 2");
MODULE_PARM_DESC(trigger3, "Optional trigger name for LED 3");
MODULE_PARM_DESC(trigger4, "Optional trigger name for LED 4");
MODULE_PARM_DESC(trigger5, "Optional trigger name for LED 5");
MODULE_PARM_DESC(trigger6, "Optional trigger name for LED 6");
MODULE_PARM_DESC(trigger7, "Optional trigger name for LED 7");

static uint8_t *brgb_param[8] = {
	brgb0,
	brgb1,
	brgb2,
	brgb3,
	brgb4,
	brgb5,
	brgb6,
	brgb7,
};
static unsigned *brgb_param_count[8] = {
	&brgb0_count,
	&brgb1_count,
	&brgb2_count,
	&brgb3_count,
	&brgb4_count,
	&brgb5_count,
	&brgb6_count,
	&brgb7_count,
};
static char **trigger_param[8] = {
	&trigger0,
	&trigger1,
	&trigger2,
	&trigger3,
	&trigger4,
	&trigger5,
	&trigger6,
	&trigger7,
};


static struct blinkt_led_data *mc_to_led_data(struct led_classdev_mc *mc_cdev)
{
	return container_of(mc_cdev, struct blinkt_led_data, mc);
}


static void dat_on(struct blinkt_leds_priv *priv)
{
	gpiod_set_value(priv->dat_pin, 1);
}


static void dat_off(struct blinkt_leds_priv *priv)
{
	gpiod_set_value(priv->dat_pin, 0);
}


static void clk_tick(struct blinkt_leds_priv *priv)
{
	/* clk on */
	gpiod_set_value(priv->clk_pin, 1);
	udelay(CLK_DELAY);
	/* clk off */
	gpiod_set_value(priv->clk_pin, 0);
	udelay(CLK_DELAY);
}


static void clear_led_buffer(struct blinkt_leds_priv *priv)
{
	for (int i=0; i<NUM_LEDS; ++i)
		priv->led_buffer[i] = 0xea000000;
}


static void update_led_buffer(struct led_classdev_mc *mc,
			      struct blinkt_led_data *led)
{
	*led->values &= 0xff000000;
	*led->values |= ((uint32_t)mc->subled_info[0].brightness);
	*led->values |= ((uint32_t)mc->subled_info[1].brightness) <<  8;
	*led->values |= ((uint32_t)mc->subled_info[2].brightness) << 16;
}


static void commit_led_buffer(struct blinkt_leds_priv *priv)
{
	/* Send led values */
	for (int i=0; i<NUM_LEDS; ++i) {
		/* Send individual bits, most significant bit first */
		for (uint32_t mask=0x80000000; mask!=0; mask>>=1) {
			if (priv->led_buffer[i] & mask)
				dat_on(priv);
			else
				dat_off(priv);
			clk_tick(priv);
		}
	}

	/* Send commit sequence */
	dat_off(priv);
	for (int i=0; i<COMMIT_ITERATIONS; ++i)
		clk_tick(priv);
}


static void led_worker(struct work_struct *work)
{
	struct blinkt_leds_priv *priv;
	priv = container_of(work, struct blinkt_leds_priv, work);

	spin_lock_irq(&priv->lock);
	commit_led_buffer(priv);
	spin_unlock_irq(&priv->lock);
}


static void set_led(struct led_classdev *led_cdev,
		    enum led_brightness brightness)
{
	struct led_classdev_mc *mc = lcdev_to_mccdev(led_cdev);
	struct blinkt_led_data *led = mc_to_led_data(mc);
	unsigned long flags;

	led_mc_calc_color_components(mc, brightness);

	spin_lock_irqsave(&led->priv->lock, flags);
	update_led_buffer(mc, led);
	spin_unlock_irqrestore(&led->priv->lock, flags);

	schedule_work(&led->priv->work);
}


static void init_led_trigger(struct device *dev,
			     struct led_classdev *led_cdev,
			     int index)
{
	char prop_name[20];
	const char *trigger;

	if (*trigger_param[index] && *trigger_param[index][0]) {
		led_cdev->default_trigger = *trigger_param[index];
	}else{
		snprintf(prop_name, sizeof(prop_name), "blinkt-%d-trigger", index);
		of_property_read_string(dev->of_node, prop_name, &trigger);
		if (trigger)
			led_cdev->default_trigger = trigger;
	}
}


static void init_led_values(struct device *dev,
			    struct led_classdev_mc *mc,
			    int index)
{
	if (*brgb_param_count[index]) {
		mc->led_cdev.brightness = brgb_param[index][0];
		mc->subled_info[0].intensity = brgb_param[index][1];
		mc->subled_info[1].intensity = brgb_param[index][2];
		mc->subled_info[2].intensity = brgb_param[index][3];
	}else{
		char prop_name[16];
		u8 rgb[3] = {0x0, 0x0, 0x0};
		snprintf(prop_name, sizeof(prop_name), "blinkt-%d-rgb", index);
		of_property_read_u8_array(dev->of_node, prop_name, rgb, 3);
		mc->led_cdev.brightness = (rgb[0]|rgb[1]|rgb[2]) ? 255 : 0;
		mc->subled_info[0].intensity = rgb[0];
		mc->subled_info[1].intensity = rgb[1];
		mc->subled_info[2].intensity = rgb[2];
	}
}


static int register_led (struct device *dev,
			 struct blinkt_leds_priv *priv,
			 int index,
			 const char *name_prefix,
			 size_t max_name_size)
{
	struct blinkt_led_data *led_data = &priv->leds[index];
	struct led_classdev_mc *mc = &led_data->mc;

	char *name = devm_kzalloc(dev, max_name_size, GFP_KERNEL);
	if (!name) {
		dev_err(dev, "Error allocating memory for LED name(s)\n");
		return -ENOMEM;
	}
	snprintf(name, max_name_size, "%s%d", name_prefix, index);

	led_data->values = &priv->led_buffer[index];
	led_data->priv = priv;
	mc->led_cdev.name = name;
	mc->led_cdev.max_brightness = 255;
	mc->led_cdev.brightness_set = set_led;
	init_led_trigger(dev, &mc->led_cdev, index);

	mc->subled_info = devm_kmalloc_array(dev, 3,
					     sizeof(*mc->subled_info),
					     GFP_KERNEL | __GFP_ZERO);
	if (!mc->subled_info) {
		dev_err(dev, "Error allocating memory for LED info\n");
		return -ENOMEM;
	}
	mc->subled_info[0].color_index = LED_COLOR_ID_RED;
	mc->subled_info[1].color_index = LED_COLOR_ID_GREEN;
	mc->subled_info[2].color_index = LED_COLOR_ID_BLUE;
	mc->num_colors = 3;
	init_led_values (dev, mc, index);

	int ret = devm_led_classdev_multicolor_register(dev,
							&led_data->mc);
	if (ret) {
		dev_err(dev, "Error registering LED %s%d\n",
			name_prefix, index);
		return ret;
	}

	/* Initialize LED value buffer */
	led_mc_calc_color_components(mc, mc->led_cdev.brightness);
	update_led_buffer(mc, led_data);

	return 0;
}


static int register_leds (struct platform_device *pdev,
			  struct blinkt_leds_priv *priv)
{
	struct device *dev = &pdev->dev;
	const char *name_prefix = NULL;
	size_t max_name_size = 0;
	int ret = 0;
	int i = 0;

	if (prefix && prefix[0]) {
		name_prefix = prefix;
	}else{
		of_property_read_string(dev->of_node, "blinkt-name-prefix", &name_prefix);
		if (!name_prefix)
			name_prefix = "blinkt";
	}
	max_name_size = strlen(name_prefix) + 4;

	for (i=0; i<NUM_LEDS; ++i) {
		ret = register_led (dev, priv, i, name_prefix, max_name_size);
		if (ret)
			break;
	}
	if (ret) {
		/* On error, unregister already registered LEDs */
		for (int n=0; n<i; ++n)
			devm_led_classdev_multicolor_unregister(dev, &priv->leds[n].mc);
	}else{
		dev_info(dev, "Registered LEDs %s0 ... %s%d\n",
			 name_prefix, name_prefix, NUM_LEDS);
	}
	return ret;
}


static int blinkt_init_probe(struct platform_device *pdev)
{
	struct blinkt_leds_priv *priv;
	struct device *dev = &pdev->dev;
	int ret = 0;

	priv = devm_kzalloc(dev, sizeof(struct blinkt_leds_priv), GFP_KERNEL);
	if (!priv) {
		dev_err(dev, "Error allocating device memory\n");
		ret = PTR_ERR(priv);
		goto out;
	}

	priv->clk_pin = devm_gpiod_get(&pdev->dev, "blinkt-clk", GPIOD_OUT_LOW);
	if (IS_ERR(priv->clk_pin)) {
		dev_err(dev, "Can't initialize clk pin\n");
		ret = PTR_ERR(priv);
		goto out;
	}

	priv->dat_pin = devm_gpiod_get(&pdev->dev, "blinkt-dat", GPIOD_OUT_LOW);
	if (IS_ERR(priv->dat_pin)) {
		dev_err(dev, "Can't initialize dat pin\n");
		devm_gpiod_put(&pdev->dev, priv->clk_pin);
		ret = PTR_ERR(priv);
		goto out;
	}

	/* Send start sequence */
	dat_off(priv);
	for (int i=0; i<INIT_ITERATIONS; ++i)
		clk_tick(priv);

	INIT_WORK(&priv->work, led_worker);
	spin_lock_init(&priv->lock);

	clear_led_buffer(priv);
	ret = register_leds(pdev, priv);
	if (ret) {
		cancel_work_sync(&priv->work);
		devm_gpiod_put(dev, priv->dat_pin);
		devm_gpiod_put(dev, priv->clk_pin);
		goto out;
	}

	/* Set LEDs with initial values */
	commit_led_buffer(priv);

	platform_set_drvdata(pdev, priv);
 out:
	return ret;
}


static int blinkt_exit_remove(struct platform_device *pdev)
{
	struct blinkt_leds_priv *priv = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	dev_info(dev, "Unegister LEDs\n");
	for (int i=0; i<NUM_LEDS; ++i)
		devm_led_classdev_multicolor_unregister(dev, &priv->leds[i].mc);

	cancel_work_sync(&priv->work);
	clear_led_buffer(priv);
	commit_led_buffer(priv);

	devm_gpiod_put(dev, priv->dat_pin);
	devm_gpiod_put(dev, priv->clk_pin);

	return 0;
}


static struct of_device_id blinkt_match[] = {
	{.compatible = "pimoroni,blinkt"},
	{}
};


static struct platform_driver blinkt_driver = {
	.probe = blinkt_init_probe,
	.remove = blinkt_exit_remove,
	.driver = {
		.name = "blinkt_driver",
		.owner = THIS_MODULE,
		.of_match_table = blinkt_match,
	}
};


module_platform_driver(blinkt_driver);

MODULE_AUTHOR("Dan Arrhenius <dan@ultramarin.se>");
MODULE_DESCRIPTION("Pimoroni Blinkt LED driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:blinkt_driver");
