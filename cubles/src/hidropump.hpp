#pragma once

#include "config.hpp"

/**
 * @brief GPIO Solid State Relay Hop Config.
 * @details Configuration for the two relays that connect the SSR to the 24V line, thus connecting the live wire to the hidropump.
 */
static constexpr gpio_config_t GPIO_SSR_HOP_CFG = {
	.pin_bit_mask = BV( GPIO_Q_D1_1 ) | BV( GPIO_Q_D1_2 ),
	.mode         = GPIO_MODE_OUTPUT,
	.pull_up_en   = GPIO_PULLUP_DISABLE,
	.pull_down_en = GPIO_PULLDOWN_DISABLE,
	.intr_type    = GPIO_INTR_DISABLE
};

/**
 * @brief Hidropump master.
 */
class Hidro_pump {
public:
    status_t init( void ) {
        gpio_reset_pin( GPIO_Q_D1_1 ); 
        gpio_reset_pin( GPIO_Q_D1_2 );

        gpio_set_level( GPIO_Q_D1_1, HIGH );
        gpio_set_level( GPIO_Q_D1_2, HIGH );
        ASSERT_OR( ESP_OK == gpio_config( &GPIO_SSR_HOP_CFG ) ) return -0x1;

        _master_timer = xTimerCreate(
            "OCA-HP-MT", 1, pdFALSE, ( void* )this,
            [] ( TimerHandle_t hdl_ ) static -> void {
                auto* self = ( Hidro_pump* )pvTimerGetTimerID( hdl_ );
                self->disengage();
            }
        );
        ASSERT_OR( _master_timer ) return -0x1;

        return 0x0;
    }

protected:
    TimerHandle_t   _master_timer   = nullptr;

public:
    status_t engage( uint32_t period_ms_ ) {
        ASSERT_OR( _master_timer ) return -0x1;
        ASSERT_OR( period_ms_ <= HIDROPUMP_MAX_ENGAGE_PERIOD_MS ) return -0x1;

        ASSERT_OR( pdPASS == xTimerChangePeriod( _master_timer, pdMS_TO_TICKS( period_ms_ ), 0 ) ) return -0x1;
        ASSERT_OR( pdPASS == xTimerStart( _master_timer, 0 ) ) return -0x1;

        gpio_set_level( GPIO_Q_D1_1, LOW );
        vTaskDelay( 500_pdms2t );
        gpio_set_level( GPIO_Q_D1_2, LOW );

        return 0x0;
    }

    void disengage( void ) {
        gpio_set_level( GPIO_Q_D1_1, HIGH );
        gpio_set_level( GPIO_Q_D1_2, HIGH );
    }

};
inline Hidro_pump Hidro_Pump;