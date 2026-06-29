/**
 * @brief: OCA - Outdoor Station version Alpha.
 * @details: 
 * @authors: Vatca "Mipsan" Tudor-Horatiu
 */

/* =-. ESP32 .-= */
#include <driver/gpio.h>

/* =-. RGH .-= */
#include <rgh/ucp/ino/wifi_daemon.hpp> 
#define RGH_INO_THINGSBOARD_DAEMON_MQTT_MAX_PACKET_SIZE_ 1024
#define RGH_INO_THINGSBOARD_DAEMON_CFG_                  thingsboard_daemon_static_default_cfg_t
#include <rgh/ucp/ino/thingsboard_daemon.hpp>

#include <rgh/ucp/esp32-5x/nvs_flash.hpp>
#include <rgh/ucp/esp32-5x/HRPT_array.hpp>
#include <rgh/ucp/esp32-5x/sns-drv/bmx280.hpp>

#include <rgh/gep/rtatomic.hpp>
#include <rgh/gep/text_utils.hpp>
#include <rgh/gep/fastcli.hpp>
using namespace rgh; using namespace rgh::ino; using namespace rgh::esp32; using namespace rgh::esp32::snsd;
using namespace rgh::freertos; using namespace rgh::freertos_literals;

/* =-. Others .-= */
#include ".env"
using namespace std; using namespace std::chrono_literals;

void query_serial_for_cli( void );

/* =-. Error handler .-= */
static uint32_t Error_word = 0x0;
void critical_handler( void ) { esp_restart(); } 

/* =-. IO .-= */
struct _Input {
public:
	/**
	 * @brief I2C ALPHA bus.
	 * @details Devices on this bus: BME280-ALPHA.
	 */
	static constexpr i2c_master_bus_config_t   I2C_BUS_ALPHA_CONFIG   = {
		.i2c_port          = I2C_NUM_0,
		.sda_io_num        = GPIO_NUM_8,
		.scl_io_num        = GPIO_NUM_9,
		.clk_source        = I2C_CLK_SRC_DEFAULT,
		.glitch_ignore_cnt = 7,
		.trans_queue_depth = 0x0,
		.flags             = {
			.enable_internal_pullup = false,
			.allow_pd               = false
		}
	};
	i2c_master_bus_handle_t                    I2C_bus_alpha          = nullptr;

	/**
	 * @brief BME280 - temperature, pressure and humidity ALPHA.
	 */
	static constexpr i2c_device_config_t   BME280_ALPHA_I2C_CONFIG   = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address  = BME280::I2C_ADDRESS_SDO_VCC,
		.scl_speed_hz    = 100000,
	};
	BME280                                 BME280_alpha              = {};

public:
	rt_atomic< float >   out_tmp   = { rt_atomic< float >::modch_absdiff< 0.5f > };
	rt_atomic< float >   out_prs   = { rt_atomic< float >::modch_absdiff< 0.3f > };
	rt_atomic< float >   out_hum   = { rt_atomic< float >::modch_absdiff< 3.0f > };

	status_t init( void ) {
		ASSERT_OR( ESP_OK == i2c_new_master_bus( &I2C_BUS_ALPHA_CONFIG, &I2C_bus_alpha ) ) return ERR_PLATFORMCALL;

		ASSERT_STATUS_AND( BME280_alpha.bind( I2C_bus_alpha, BME280_ALPHA_I2C_CONFIG, 300 ) ) {
            for( uint8_t n = 1; n <= 3; ++n ) ASSERT_STATUS_AND( BME280_alpha.load_calibs() ) break;
        }

		ASSERT_STATUS_OR_RET( _tsk_main.launch( "input", 1024, 4, [ this ] ( auto* ) -> void {
			auto bme_data = BME280_alpha.oneshot_1xf(); 
			ASSERT_AND( bme_data.has_value() ) {
				out_tmp.push( get<0>( bme_data.value() ) );
			}

			vTaskDelay( 1000_pdms2t );
		} ) );

		return OK;
	}	

protected:
	Dynamic_task   _tsk_main   = {};

} Input;

/* =-. Daemons .-= */
Daemon_cluster_FreeRTOS   Daemon_Cluster   = {};

RGH_INO_WIFI_DAEMON_QUICK_DRIDGE( WiFi_Dridge,
([] ( void ) static -> auto { return (void)unique_lock{ EEPROM }, wifi_daemon_start_args_t{
	.ssid    = EEPROM.getString( "wifi-ssid", ENV_DEFAULT_WIFI_SSID ),
	.pwrd    = EEPROM.getString( "wifi-pwrd", ENV_DEFAULT_WIFI_PWRD ),
	.vrf_ms  = 1'000,
	.vrf_cnt = 5
}; }) )
RGH_INO_THINGSBOARD_DAEMON_QUICK_DRIDGE( Thingsboard_Dridge,
([] ( void ) static -> auto { return (void)unique_lock{ EEPROM }, thingsboard_daemon_start_args_t{
	.server  = ENV_DEFAULT_TB_SERVER,
	.token   = ENV_DEAFULT_TB_TOKEN,
	.port    = ENV_DEFAULT_TB_PORT,
	.loop_cb = [
		TIMS    = HRPT_array< 2 >{ { 30s, 60s } },
		
		out_tmp = Input.out_tmp.get_hook()
	] ( void ) mutable -> void {
		TIMS.disarm();
		
		RGH_ESP32_IF_HRPT_OUT_OR_RTA_MOD( Input.out_tmp, &out_tmp, TIMS, 0 ) Thingsboard_Daemon.send_tlmtr( "o-tmp", rtav );

		if( TIMS(1) ) Thingsboard_Daemon.send_attr( "wifi-rssi", WiFi_Daemon.rssi() );
		
		TIMS.arm();
	},
	.loop_prio  = 3,
	.loop_int_ms = 200 
}; }) )

/* =-. Main .-= */
void init_static( void ) {
	ASSERT_STATUS_OR( Input.init() ) critical_handler();

	Daemon_Cluster.push( {
		.ref       = WiFi_Daemon,
		.state_ctl = Daemon::StateCtl_KEEP_ALIVE,
		.rst_if    = [] ( const auto& ) static -> bool {
			return not WiFi_Daemon.connected();
		},
		.ctx       = &WiFi_Dridge
	} );

	Thingsboard_Daemon.daemon_set_deps( { WiFi_Daemon } );
	Daemon_Cluster.push( {
		.ref       = Thingsboard_Daemon,
		.state_ctl = Daemon::StateCtl_KEEP_ALIVE,
		.rst_if    = [] ( const auto& ) static -> bool {
			return not Thingsboard_Daemon.connected();
		},
		.ctx       = &Thingsboard_Dridge
	} );

	Daemon_Cluster.when_critical( [] ( auto ) -> void { critical_handler(); } );
	ASSERT_STATUS_OR( Daemon_Cluster.init( {
		.iterate_interval_ms = 10000,
		.task_stack_depth    = 8192,
		.task_priority       = 4
	} ) ) critical_handler();
}

extern "C" void app_main( void ) {
	Serial.begin( 115200 );
	vTaskDelay( 1000_pdms2t );

	initArduino();
    init_static();
	
	for(;;) {
		vTaskDelay( 100_pdms2t );

		if( Serial.available() )
		query_serial_for_cli();
	}
}

// =-. Command line interpreter .-=
Fast_cli Cli = {
	{}, {
	{
		.text = "systemctl",
		.opts = {
			{ .sh0rt = 'r', .l0ng = "restart", .arg = Fast_cli::Arg_text },
			{ .sh0rt = 'R', .l0ng = "report" }
		},
		.fnc = [] ( auto& stc_ ) -> status_t {
			char opt;  while( opt = stc_.next() ) {
				switch( opt ) { RGH_FASTCLI_DEFAULT_STENCIL_CASES
					case 'r': { 
						switch( txt_hash( stc_.arg_text() ) ) {
							case txt_hash( "system" ): esp_restart();
						}
					break; }

					case 'R': {
						Serial.print( "OCA//Outdoor-Station//Alpha\n" );
						Serial.print( Daemon_Cluster.report( nullptr ).c_str() );
					break; }
				}
			}
			stc_ += "cli: systemctl: done.\n";
			return RGH_OK;
		}
	}
} };

void query_serial_for_cli( void ) { 
	auto line = Serial.readStringUntil( '\n' );

	string out;
	if( RGH_OK != Cli.execute( line.c_str(), &out ) ) {
		while( Serial.read() != -1 );
	}
	Serial.println( out.c_str() );
}

