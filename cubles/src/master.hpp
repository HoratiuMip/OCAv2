#pragma once

#include "config.hpp"

/**
 * @brief I2C alpha bus configuration.
 */
constexpr i2c_master_bus_config_t   I2C_BUS_ALPHA_CONFIG   = {
	.i2c_port          = I2C_NUM_0,
	.sda_io_num        = GPIO_NUM_21,
	.scl_io_num        = GPIO_NUM_22,
	.clk_source        = I2C_CLK_SRC_DEFAULT,
	.glitch_ignore_cnt = 7,
	.trans_queue_depth = 0x0,
	.flags             = {
		.enable_internal_pullup = false,
		.allow_pd               = false
	}
};

/**
 * @brief Master.
 */
class _Master {
public:
    status_t init( void ) {
        i2c_new_master_bus( &I2C_BUS_ALPHA_CONFIG, &I2C_bus_alpha );
		
        return OK;
    }

public:
    i2c_master_bus_handle_t   I2C_bus_alpha   = NULL;

};
inline _Master Master;