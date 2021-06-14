/*
 * platform indepent driver interface
 *
 * Coypritht (c) 2017 Goodix
 */
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>
#include <linux/err.h>

#include "gf_spi.h"

#if defined(USE_SPI_BUS)
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#elif defined(USE_PLATFORM_BUS)
#include <linux/platform_device.h>
#endif

extern enum DEVICE_HWID g_ASUS_hwID;

int gf_parse_dts(struct gf_dev *gf_dev)
{
    int rc = 0;
    bool try_ldo = false;
    int fod_panel = -1;
    struct device *dev = &gf_dev->spi->dev;
    struct device_node *np = dev->of_node;
    u32 voltage_supply[2];
    u32 sd_voltage_supply[2];

/* +++ pars all gpio +++ */
    gf_dev->reset_gpio = of_get_named_gpio(np, "fp-gpio-reset", 0);
    if (gf_dev->reset_gpio < 0) {
        pr_err("falied to get reset gpio!\n");
        return gf_dev->reset_gpio;
    }

    gf_dev->irq_gpio = of_get_named_gpio(np, "fp-gpio-irq", 0);
    if (gf_dev->irq_gpio < 0) {
        pr_err("falied to get irq gpio!\n");
        return gf_dev->irq_gpio;
    }

    /* +++ asus bsp jacob add for pwr pin control +++ */
    gf_dev->pwr_gpio= of_get_named_gpio(np, "fp-gpio-vcc-enable", 0);
    if (gf_dev->pwr_gpio < 0) {
        pr_err("falied to get pwr gpio, try ldo!\n");
        try_ldo = true;
    }
      /* --- asus bsp jacob add for pwr pin control --- */
#if 1
    rc = of_property_read_u32(np, "fp-fod-panel", &fod_panel);
    if (fod_panel < 0) {
        pr_err("use no fod config !\n");
        fod_panel = false;
    } else {
        printk("[Jacob] fp-fod-panel value %d \n", fod_panel);        
    }
#endif
    if (fod_panel) {
        gf_dev->lhbm_exi1_gpio = of_get_named_gpio(np, "fp-FOD-EXI1", 0);
        if (gf_dev->lhbm_exi1_gpio < 0) {
            pr_err("falied to get lhbm_exi1 gpio !\n");
            return gf_dev->lhbm_exi1_gpio;
        }

        gf_dev->lhbm_exi2_gpio = of_get_named_gpio(np, "fp-FOD-EXI2", 0);
        if (gf_dev->lhbm_exi2_gpio < 0) {
            pr_err("falied to get lhbm_exi2 gpio!\n");
            return gf_dev->lhbm_exi2_gpio;
        }

        printk("[Jacob] gf_spi Get the pinctrl node \n");
        /* Get the pinctrl node */
        gf_dev->pinctrl = devm_pinctrl_get(dev);
        if (IS_ERR_OR_NULL(gf_dev->pinctrl)) {
           pr_err("%s: Failed to get pinctrl\n", __func__);
           return PTR_ERR(gf_dev->pinctrl);
        }
        printk("[Jacob] gf_spi Get the active setting \n");
        /* Get the active setting */
        gf_dev->pins_active = pinctrl_lookup_state(gf_dev->pinctrl,"fod_exi_active");
        if (IS_ERR_OR_NULL(gf_dev->pins_active)) {
            pr_err("%s: Failed to get pinctrl state active\n", __func__);
            return PTR_ERR(gf_dev->pins_active);
        }

        if (gf_dev->pinctrl && gf_dev->pins_active) {
            rc = pinctrl_select_state(gf_dev->pinctrl, gf_dev->pins_active);
            if (rc < 0) {
                pr_err("Set normal pin state error:%d", rc);
            }
        }
        printk("[Jacob] check FOD EXI  gpio num : EXI1 = %d, EXI2 = %d \n", gf_dev->lhbm_exi1_gpio, gf_dev->lhbm_exi2_gpio);
    }

/* --- pars all gpio --- */
        if (try_ldo)
            printk("[Jacob] check all gpio num : reset = %d, irq = %d \n", gf_dev->reset_gpio, gf_dev->irq_gpio);
        else
            printk("[Jacob] check all gpio num : reset = %d, irq = %d, vcc-enable = %d \n", gf_dev->reset_gpio, gf_dev->irq_gpio, gf_dev->pwr_gpio);

/* +++ request all gpio +++ */
    /* +++ asus bsp jacob add for pwr pin control +++ */
    if (!try_ldo) {
        rc = gpio_request(gf_dev->pwr_gpio, "goodix_pwr");
        if (rc) {
            pr_err("failed to request pwr gpio, rc = %d\n", rc);
            goto err_pwr;
        }
        //gpio_direction_output(gf_dev->pwr_gpio, 1);
        /* --- asus bsp jacob add for pwr pin control --- */
       } else {
      /* --- asus bsp jacob add for pwr ldo control +++ */
      rc = of_property_read_u32_array(np, "asus-fp,vcc-voltage", voltage_supply, 2);
      if (rc < 0) {
          printk("Failed to get regulator avdd voltage !\n");
          return rc;
      }
      printk("[FP] Regulator voltage get Max = %d, Min = %d \n", voltage_supply[1], voltage_supply[0]);   
      gf_dev->vcc = devm_regulator_get(dev, "vcc");
      if (IS_ERR( gf_dev->vcc)) {
          rc = PTR_ERR(gf_dev->vcc);
          printk("[FP] Regulator get vcc failed rc=%d\n", rc);
          goto reg_vdd_get_vtg;
          }
      if (regulator_count_voltages(gf_dev->vcc) > 0) {
          rc = regulator_set_voltage(gf_dev->vcc, voltage_supply[0],  voltage_supply[1]);
          if (rc) {
              printk("[FP] Regulator set_vcc failed vdd rc=%d\n", rc);
              goto reg_vcc_i2c_put;
          }
      }

      /**/
      /* --- asus bsp jacob add for SD ldo control +++ */
      rc = of_property_read_u32_array(np, "asus-sd,vcc-voltage", sd_voltage_supply, 2);
      if (rc < 0) {
          printk("Failed to get regulator avdd voltage !\n");
          return rc;
      }
      printk("[FP] Regulator voltage get Max = %d, Min = %d \n", sd_voltage_supply[1], sd_voltage_supply[0]);   
      gf_dev->sdvcc = devm_regulator_get(dev, "sdcard");
      if (IS_ERR( gf_dev->sdvcc)) {
          rc = PTR_ERR(gf_dev->sdvcc);
          printk("[FP] Regulator get sd vcc failed rc=%d\n", rc);
          goto reg_vdd_get_vtg;
          }
      if (regulator_count_voltages(gf_dev->sdvcc) > 0) {
          rc = regulator_set_voltage(gf_dev->sdvcc, voltage_supply[0],  voltage_supply[1]);
          if (rc) {
              printk("[FP] Regulator set_vcc failed vdd rc=%d\n", rc);
              goto reg_vcc_i2c_put;
          }
      }

      rc = regulator_enable(gf_dev->sdvcc);

      /**/

      }
      rc = gf_power_on(gf_dev);
      if (rc) {
          printk("[FP] Regulator set_vcc failed vdd rc=%d\n", rc);
       goto reg_vcc_i2c_put;
      }

      /* --- asus bsp jacob add for pwr ldo control --- */

      msleep(10);
    rc = gpio_request(gf_dev->reset_gpio, "goodix_reset");
    if (rc) {
        pr_err("failed to request reset gpio, rc = %d\n", rc);
        goto err_reset;
    }
    gpio_direction_output(gf_dev->reset_gpio, 1);


    rc = gpio_request(gf_dev->irq_gpio, "goodix_irq");
    if (rc) {
        pr_err("failed to request irq gpio, rc = %d\n", rc);
        goto err_irq;
    }
    gpio_direction_input(gf_dev->irq_gpio);

    if (fod_panel) {
        rc = gpio_request(gf_dev->lhbm_exi1_gpio, "goodix_lhbm_exi1");
        if (rc) {
            pr_err("failed to request goodix_lhbm_exi1 gpio, rc = %d\n", rc);
            goto err_lhbm_exi1;
        }

        rc = gpio_request(gf_dev->lhbm_exi2_gpio, "goodix_lhbm_exi2");
        if (rc) {
            pr_err("failed to request goodix_lhbm_exi2 gpio, rc = %d\n", rc);
            goto err_lhbm_exi2;
        }

        gpio_direction_output(gf_dev->lhbm_exi1_gpio, 0);
        gpio_direction_output(gf_dev->lhbm_exi2_gpio, 0);
        gpio_direction_output(gf_dev->reset_gpio, 0);

    }

/* --- request all gpio --- */
    return rc;

err_lhbm_exi2:
    gpio_free(gf_dev->lhbm_exi1_gpio);
err_lhbm_exi1:
    gpio_free(gf_dev->irq_gpio);
err_irq:
    gpio_free(gf_dev->reset_gpio);
err_reset:
err_pwr:
reg_vcc_i2c_put:
    if (try_ldo) {
        regulator_put(gf_dev->vcc);
    } else {
    gpio_free(gf_dev->pwr_gpio);
    } 
reg_vdd_get_vtg:

    return rc;
}

void gf_cleanup(struct gf_dev *gf_dev)
{
    pr_info("[info] %s\n", __func__);
#if 0
    if (gpio_is_valid(gf_dev->irq_gpio)) {
        gpio_free(gf_dev->irq_gpio);
        pr_info("remove irq_gpio success\n");
    }
    if (gpio_is_valid(gf_dev->reset_gpio)) {
        gpio_free(gf_dev->reset_gpio);
        pr_info("remove reset_gpio success\n");
    }
#endif
}

int gf_power_on(struct gf_dev *gf_dev)
{
    int rc = 0;
    printk("[FP][%s]pwr_gpio = %d  ! \n", __func__, gf_dev->pwr_gpio);
    if (gf_dev->pwr_gpio < 0) {
           if(gf_dev->power_init) {
               printk("[FP][%s]Regulator vcc already enable ! \n", __func__);
           } else {
               rc = regulator_enable(gf_dev->vcc);
            if (rc) {
            printk("[FP][%s]Regulator vcc enable failed rc=%d\n", __func__, rc);
            } else {
            printk("[FP][%s]Regulator vcc enable ! \n", __func__);
            gf_dev->power_init = true;
            }
           }          
       } else {
        rc = gpio_direction_output(gf_dev->pwr_gpio, 1);
           if (rc) {
              printk("[FP][%s]pwr_gpio enable failed rc=%d\n", __func__, rc);
        } else {
           printk("[FP][%s]pwr_gpio enable ! \n", __func__);
        gf_dev->power_init = true;
        }
       }
    /* TODO: add your power control here */
    return rc;
}

int gf_power_off(struct gf_dev *gf_dev)
{
    int rc = 0;
    printk("[FP][%s]pwr_gpio = %d  ! \n", __func__, gf_dev->pwr_gpio);
    if (gf_dev->pwr_gpio < 0) {
           if(gf_dev->power_init) {
               rc = regulator_disable(gf_dev->vcc);
            if (rc) {
            printk("[FP][%s]Regulator vcc disable failed rc=%d\n", __func__, rc);
            } else {
            printk("[FP][%s]Regulator vcc disabe ! \n", __func__);
            gf_dev->power_init = false;
            }
           } else {
               printk("[FP][%s]Regulator vcc already disable ! \n", __func__);
           }          
       } else {
        gpio_direction_output(gf_dev->pwr_gpio, 0);
       }

    /* TODO: add your power control here */

    return rc;
}

int gf_hw_reset(struct gf_dev *gf_dev, unsigned int delay_ms)
{
    if (gf_dev == NULL) {
        pr_info("Input buff is NULL.\n");
        return -1;
    }
    gpio_direction_output(gf_dev->reset_gpio, 1);
    gpio_set_value(gf_dev->reset_gpio, 0);
    printk("[FP][%s]gpio num %d value =  %d ! \n", __func__, gf_dev->reset_gpio, gpio_get_value(gf_dev->reset_gpio));
    mdelay(3);
    gpio_set_value(gf_dev->reset_gpio, 1);
    printk("[FP][%s]gpio num %d value =  %d ! \n", __func__, gf_dev->reset_gpio, gpio_get_value(gf_dev->reset_gpio));
    mdelay(delay_ms);
    return 0;
}

int gf_irq_num(struct gf_dev *gf_dev)
{
    if (gf_dev == NULL) {
        pr_info("Input buff is NULL.\n");
        return -1;
    } else {
        return gpio_to_irq(gf_dev->irq_gpio);
    }
}
