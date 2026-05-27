#pragma once

#include "config.hpp"
#include "master.hpp"

#include <rgh/gep/versioned_atomics.hpp>
#include <rgh/ucp/esp32-5x/sns-drv/bmp280.hpp>

constexpr i2c_device_config_t I2C_DEV_BMP280_CONFIG = {
	.dev_addr_length = I2C_ADDR_BIT_LEN_7,
	.device_address  = snsd::BMP280::I2C_ADDRESS_SDO_VCC,
	.scl_speed_hz    = 100000,
};

/**
 * @brief Environment master.
 */
class _Environment {
public:
    status_t init( void ) {
        ASSERT_AND( OK == _BMP280_temp_alpha.bind( Master.I2C_bus_alpha, I2C_DEV_BMP280_CONFIG, 300 ) ) {
            _BMP280_temp_alpha.load_calibs();
        }

		ASSERT_OR( pdPASS == xTaskCreate(
			&_Environment::_main, "OCA-ENV-MAIN",
			8192, this, TaskPriority_Default, &_tsk_main
		) ) {
			ESP_LOGE( TAG, "env: bad main task." );
			return ERR_SYSCALL;
		}
   
        return OK;
    }

protected:
    TaskHandle_t          _tsk_main            = nullptr;
    esp32::snsd::BMP280   _BMP280_temp_alpha   = {};

public:
    Versioned_atomic< float >   temp_alpha   = { 0.0f };

protected:
    static void _main( void* arg_ ) {
        auto* self = ( _Environment* )arg_;

    for(;;) {
        self->_BMP280_temp_alpha.store_ctrl_meas( 
            snsd::BMP280::CtrlMeas_TemperatureSampling_1x | 
            snsd::BMP280::CtrlMeas_PressureSampling_1x    | 
            snsd::BMP280::CtrlMeas_Power_OneShot 
        );
        auto [ temp_alpha, _ ] = self->_BMP280_temp_alpha.load_dataf().value_or( make_tuple( 0.0f, 0.0f ) );
        self->temp_alpha.push( temp_alpha );

        vTaskDelay( 15000_pdms2t );
    } }

public:


};
inline _Environment Environment;