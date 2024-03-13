/*
 * Copyright (C) 2024 Dan Arrhenius <dan@ultramarin.se>
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
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/gpio/consumer.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/led-class-multicolor.h>
#include <linux/of.h>
#include <linux/of_graph.h>

#define NUM_LEDS 8
#define CLK_DELAY 1
#define INIT_ITERATIONS 32
#define COMMIT_ITERATIONS 36

struct blinkt_leds_priv;

struct blinkt_led_data {
    struct led_classdev_mc mc;
    struct blinkt_leds_priv* priv;
    uint8_t* values;
};

struct blinkt_leds_priv {
    struct gpio_desc* clk_pin;
    struct gpio_desc* dat_pin;
    uint8_t led_buffer[NUM_LEDS][4];
    struct blinkt_led_data leds[NUM_LEDS];
};


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
static inline struct blinkt_led_data* mc_to_led_data (struct led_classdev_mc* mc_cdev)
{
    return container_of (mc_cdev, struct blinkt_led_data, mc);
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
static inline void clk_on (struct blinkt_leds_priv* priv)
{
    gpiod_set_value (priv->clk_pin, 1);
    udelay (CLK_DELAY);
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
static inline void clk_off (struct blinkt_leds_priv* priv)
{
    gpiod_set_value (priv->clk_pin, 0);
    udelay (CLK_DELAY);
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
static inline void dat_on (struct blinkt_leds_priv* priv)
{
    gpiod_set_value (priv->dat_pin, 1);
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
static inline void dat_off (struct blinkt_leds_priv* priv)
{
    gpiod_set_value (priv->dat_pin, 0);
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
static void clear (struct blinkt_leds_priv* priv)
{
    for (int i=0; i<NUM_LEDS; ++i) {
        //priv->led_buffer[i][0] = 0 | 0xe0;
        priv->led_buffer[i][0] = 254 | 0xe0;

        priv->led_buffer[i][1] = 0;
        priv->led_buffer[i][2] = 0;
        priv->led_buffer[i][3] = 0;
    }
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
static void commit_led_buffer (struct blinkt_leds_priv* priv)
{
    // Send led values
    //
    for (int i=0; i<NUM_LEDS; ++i) {
        for (size_t j=0; j<4; ++j) {
            uint8_t val = priv->led_buffer[i][j];
            for (int bit=0; bit<8; ++bit) {
                // Set individual bit, most significant bit first
                if (val & 0x80)
                    dat_on (priv);
                else
                    dat_off (priv);
                val <<= 1;

                clk_on (priv);
                clk_off (priv);
            }
        }
    }

    // Send commit sequence
    //
    dat_off (priv);
    for (int i=0; i<COMMIT_ITERATIONS; ++i) {
        clk_on (priv);
        clk_off (priv);
    }
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
static void set_led (struct led_classdev* led_cdev,
                     enum led_brightness brightness)
{
    struct led_classdev_mc* mc_cdev = lcdev_to_mccdev (led_cdev);
    struct blinkt_led_data* led = mc_to_led_data (mc_cdev);

    led_mc_calc_color_components (mc_cdev, brightness);

    //led->values[0] = brightness | 0xe0;
    led->values[1] = mc_cdev->subled_info[2].brightness;
    led->values[2] = mc_cdev->subled_info[1].brightness;
    led->values[3] = mc_cdev->subled_info[0].brightness;
    commit_led_buffer (led->priv);
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
static int register_leds (struct platform_device* pdev,
                          struct blinkt_leds_priv* priv,
                          const char* name_prefix)
{
    struct device *dev = &pdev->dev;
    size_t name_size = strlen(name_prefix) + 4;
    char trigger_property_name[16];
    char intensity_property_name[20];

    for (int i=0; i<NUM_LEDS; ++i) {
        struct mc_subled* mc_led_info;
        const char* trigger_name;
        u8 intensity[3];

        char* name = devm_kzalloc (dev, name_size, GFP_KERNEL);
        if (!name)
            return -ENOMEM;
        snprintf (name, name_size, "%s%d", name_prefix, i);

        priv->leds[i].values = priv->led_buffer[i];
        priv->leds[i].priv = priv;
        priv->leds[i].mc.led_cdev.name = name;
        priv->leds[i].mc.led_cdev.brightness = 0;
        priv->leds[i].mc.led_cdev.max_brightness = 255;
        priv->leds[i].mc.led_cdev.brightness_set = set_led;

        // Check for a default trigger name
        snprintf (trigger_property_name, sizeof(trigger_property_name), "led-%d-trigger", i);
        of_property_read_string (dev->of_node, trigger_property_name, &trigger_name);
        if (trigger_name)
            priv->leds[i].mc.led_cdev.default_trigger = trigger_name;

	mc_led_info = devm_kmalloc_array (dev, 3, sizeof(*mc_led_info),
                                          GFP_KERNEL | __GFP_ZERO);
	if (!mc_led_info)
            return -ENOMEM;
	mc_led_info[0].color_index = LED_COLOR_ID_RED;
	mc_led_info[1].color_index = LED_COLOR_ID_GREEN;
	mc_led_info[2].color_index = LED_COLOR_ID_BLUE;

        // Check for color intensity values
        snprintf (intensity_property_name, sizeof(intensity_property_name), "led-%d-intensity", i);
        if (of_property_read_u8_array(dev->of_node, intensity_property_name, intensity, 3)) {
            // Use default color intensity values
            intensity[0] = intensity[1] = intensity[2] = 0;
        }
        mc_led_info[0].intensity = intensity[0];
        mc_led_info[1].intensity = intensity[1];
        mc_led_info[2].intensity = intensity[2];

        priv->leds[i].mc.num_colors = 3;
        priv->leds[i].mc.subled_info = mc_led_info;

	if (devm_led_classdev_multicolor_register(dev, &priv->leds[i].mc)) {
            return -1;
        }
    }

    return 0;
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
static int blinkt_init_probe (struct platform_device* pdev)
{
    struct blinkt_leds_priv* priv;
    struct device* dev = &pdev->dev;
    const char* name_prefix = NULL;

    priv = devm_kzalloc (dev, sizeof(struct blinkt_leds_priv), GFP_KERNEL);
    if (!priv)
        return PTR_ERR (priv);

    priv->clk_pin = devm_gpiod_get (&pdev->dev, "led-clk", GPIOD_OUT_LOW);
    if (IS_ERR(priv->clk_pin)) {
        printk (KERN_ERR "Can't initialize clk pin\n");
        return PTR_ERR (priv->clk_pin);
    }

    priv->dat_pin = devm_gpiod_get (&pdev->dev, "led-dat", GPIOD_OUT_LOW);
    if (IS_ERR(priv->dat_pin)) {
        printk (KERN_ERR "Can't initialize dat pin\n");
        devm_gpiod_put (&pdev->dev, priv->clk_pin);
        return PTR_ERR (priv->dat_pin);
    }

    of_property_read_string (dev->of_node, "led-name-prefix", &name_prefix);
    if (!name_prefix) {
        name_prefix = "blinkt";
    }

    // Send start sequence
    //
    dat_off (priv);
    for (int i=0; i<INIT_ITERATIONS; ++i) {
        clk_on (priv);
        clk_off (priv);
    }

    // Clear LEDs
    //
    clear (priv);
    commit_led_buffer (priv);

    register_leds (pdev, priv, name_prefix);
    platform_set_drvdata (pdev, priv);

    return 0;
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
static int blinkt_exit_remove(struct platform_device *pdev)
{
    struct blinkt_leds_priv* priv = platform_get_drvdata (pdev);

    for (int i=0; i<NUM_LEDS; ++i) {
        devm_led_classdev_multicolor_unregister (&pdev->dev, &priv->leds[i].mc);
    }

    clear (priv);
    commit_led_buffer (priv);

    devm_gpiod_put (&pdev->dev, priv->dat_pin);
    devm_gpiod_put (&pdev->dev, priv->clk_pin);

    return 0;
}


//------------------------------------------------------------------------------
static struct of_device_id blinkt_match[] = {
    {.compatible = "pimoroni,blinkt"},
    {}
};


//------------------------------------------------------------------------------
static struct platform_driver blinkt_driver = {
    .probe = blinkt_init_probe,
    .remove = blinkt_exit_remove,
    .driver = {
        .name = "blinkt_driver",
        .owner = THIS_MODULE,
        .of_match_table = blinkt_match,
    }
};


//------------------------------------------------------------------------------
module_platform_driver(blinkt_driver);

MODULE_AUTHOR ("Dan Arrhenius");
MODULE_DESCRIPTION ("Pimoroni Blinkt LED driver");
MODULE_LICENSE ("GPL");
MODULE_ALIAS ("platform:blinkt_driver");
