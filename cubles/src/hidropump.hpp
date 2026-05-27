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

constexpr gpio_config_t GPIO_OPTO_HP_FEED_CFG = {
	.pin_bit_mask = BV( GPIO_I_D1_4 ),
	.mode         = GPIO_MODE_INPUT,
	.pull_up_en   = GPIO_PULLUP_ENABLE,
	.pull_down_en = GPIO_PULLDOWN_DISABLE,
	.intr_type    = GPIO_INTR_DISABLE
};

/**
 * @brief Hidropump master.
 */
class Hidro_pump {
public:
    status_t init( void ) {
        gpio_reset_pin( GPIO_I_D1_4 );
        gpio_reset_pin( GPIO_Q_D1_1 ); 
        gpio_reset_pin( GPIO_Q_D1_2 );

        ASSERT_OR( ESP_OK == gpio_config( &GPIO_OPTO_HP_FEED_CFG ) ) return ERR_PLATFORMCALL;

        gpio_set_level( GPIO_Q_D1_1, HIGH );
        gpio_set_level( GPIO_Q_D1_2, HIGH );
        ASSERT_OR( ESP_OK == gpio_config( &GPIO_SSR_HOP_CFG ) ) return ERR_PLATFORMCALL;

        _master_timer = xTimerCreate(
            "OCA-HP-MT", 1, pdFALSE, ( void* )this,
            [] ( TimerHandle_t hdl_ ) static -> void {
                auto* self = ( Hidro_pump* )pvTimerGetTimerID( hdl_ );
                self->_logic_disengage();
            }
        );
        ASSERT_OR( _master_timer ) return ERR_PLATFORMCALL;

        return OK;
    }

protected:
    TimerHandle_t   _master_timer   = nullptr;

public:
    inline bool is_engaged_feedback( void ) const {
        return gpio_get_level( GPIO_I_D1_4 ) == LOW;
    }

    inline uint32_t remaining_minutes( void ) const {
        return xTimerIsTimerActive( _master_timer ) ? pdTICKS_TO_MS( xTimerGetExpiryTime( _master_timer ) - xTaskGetTickCount() ) / 60'000 : 0;
    }

protected:
    inline void _logic_disengage( void ) {
        gpio_set_level( GPIO_Q_D1_1, HIGH );
        gpio_set_level( GPIO_Q_D1_2, HIGH );
    }

public:
    status_t engage( uint32_t period_ms_ ) {
        ASSERT_OR( _master_timer ) return -0x1;
        ASSERT_OR( period_ms_ <= HIDROPUMP_MAX_ENGAGE_PERIOD_MS ) return ERR_LOGIC;

        if( period_ms_ == 0 ) { this->disengage(); return OK; }

        ASSERT_OR( pdPASS == xTimerChangePeriod( _master_timer, pdMS_TO_TICKS( period_ms_ ), 0 ) ) return ERR_SYSCALL;
        ASSERT_OR( pdPASS == xTimerStart( _master_timer, 0 ) ) return ERR_SYSCALL;

        gpio_set_level( GPIO_Q_D1_1, LOW );
        vTaskDelay( pdMS_TO_TICKS( HIDROPUMP_RELAY_HOP_GAP_MS ) );
        gpio_set_level( GPIO_Q_D1_2, LOW );

        return OK;
    }

    void disengage( void ) {
        this->_logic_disengage();
        xTimerStop( _master_timer, portMAX_DELAY );
    }

};
inline Hidro_pump Hidro_Pump;