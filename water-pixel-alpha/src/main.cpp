#include <Arduino.h>

#include <esp_log.h>
#include <nvs_flash.h>
#include <driver/gpio.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>

#include <a113/gep/text_utils.hpp>
#include <a113/gep/fastcli.hpp>

#include <a113/ucp/core.hpp>
using namespace a113;
using namespace a113::freertos_literals;
#include <a113/ucp/esp32-5x/IO_i2c.hpp>
#include <a113/ucp/sns-drv/bmp280.hpp>
#include <a113/ucp/sns-drv/aht21.hpp>
#include <a113/ucp/compound.hpp>

#include <Preferences.h>

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Arduino_MQTT_Client.h>
#include <Server_Side_RPC.h>
#include <Attribute_Request.h>
#include <Shared_Attribute_Update.h>
#include <ThingsBoard.h>

// ====== Structs ======
struct storage_t : public Preferences {
protected:
	std::mutex   _mtx;

public:
	A113_inline bool begin( const char* sector_, bool ro_ ) { 
		_mtx.lock();
		return this->Preferences::begin( sector_, ro_ ); 
	}

	A113_inline void end( void ) {
		this->Preferences::end();
		_mtx.unlock();
	}

public:
	template< typename _T_ >
	status_t write( const char* sector_, const char* key_, const _T_& val_ ) {
		this->begin( sector_, false );

		bool wres = false;
		if constexpr( std::is_same_v< std::string, _T_ > ) {
			wres = val_.length() == this->putString( key_, val_.c_str() );
		} else if constexpr( std::is_same_v< int, _T_ > ) {
			wres = sizeof( int ) == this->putInt( key_, val_ );
		}

		this->end();
		return wres ? A113_OK : A113_ERR_PLATFORMCALL;
	}
};

struct config_t {
	bool   meas_led              = false;
	int    adc_soil_period_ms    = 20;
	int    adc_soil_meas_count   = 20;
};

// ====== Fields ====== 
const char* const           Tag           = "oca/wp-a";

config_t                    Config        = {};
CompoundCluster_FreeRTOS    CmpdCluster   = {};

WiFiClientSecure            WifiClient    = {};
Arduino_MQTT_Client         MqttClient    = { WifiClient };

adc_oneshot_unit_handle_t   Adc_1         = NULL;
adc_cali_handle_t           Adc_1_Cal     = NULL;

i2c_master_bus_handle_t     I2C_Bus0      = NULL;
esp32::io::I2C_m2s          I2C_AHT21     = {};
esp32::io::I2C_m2s          I2C_BMP280    = {};

snsd::AHT21                 Sns_AHT21     = {};
snsd::BMP280                Sns_BMP280    = {};

storage_t                   Storage       = {};

// ====== Configs ======
constexpr int                 SERIAL_BAUD         = 115200;
constexpr int                 SERIAL_FAST_MS      = 100;
constexpr int                 SERIAL_IDLE_MS      = 3000;
constexpr int                 SERIAL_ACT_TO_US    = 20000000;

constexpr int                 MEAS_INTERVAL_MS    = 30000;

constexpr const char* const   STORAGE_WIFI_SSID   = "wifi-ssid";
constexpr const char* const   STORAGE_WIFI_PWRD   = "wifi-pwrd";
constexpr const char* const   STORAGE_TB_SERVER   = "tb-server";
constexpr const char* const   STORAGE_TB_PORT     = "tb-port";
constexpr const char* const   STORAGE_TB_TOKEN    = "tb-token";

constexpr int                 UPPER_LIM_ATTR      = 3;
constexpr int                 ATTR_REQ_TO_US      = 10000000;
constexpr gpio_num_t          GPIO_MEAS_LED_NUM   = GPIO_NUM_7;
constexpr const char* const   ATTR_SH_MEAS_LED    = "meas-led";

constexpr adc_oneshot_unit_init_cfg_t ADC_UNIT_1_CONFIG = {
	.unit_id  = ADC_UNIT_1,
	.clk_src  = ADC_DIGI_CLK_SRC_DEFAULT,
	.ulp_mode = ADC_ULP_MODE_DISABLE
};

constexpr adc_oneshot_chan_cfg_t ADC_SOIL_MOISTURE_CHANNEL_CONFIG = {
	.atten    = ADC_ATTEN_DB_12,
	.bitwidth = ADC_BITWIDTH_12
};

constexpr adc_cali_curve_fitting_config_t ADC_SOIL_MOISTURE_CAL_CONFIG = {
	.unit_id  = ADC_UNIT_1,
	.chan     = ADC_CHANNEL_2,
	.atten    = ADC_ATTEN_DB_12,
	.bitwidth = ADC_BITWIDTH_12
};

constexpr gpio_config_t GPIO_MEAS_LED_CONFIG = {
	.pin_bit_mask = 0x1ULL << GPIO_MEAS_LED_NUM,
	.mode         = GPIO_MODE_OUTPUT,
	.pull_up_en   = GPIO_PULLUP_DISABLE,
	.pull_down_en = GPIO_PULLDOWN_DISABLE,
	.intr_type    = GPIO_INTR_DISABLE
};

constexpr i2c_master_bus_config_t I2C_BUS_0_CONFIG = {
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

constexpr i2c_device_config_t I2C_DEV_AHT21_CONFIG = {
	.dev_addr_length = I2C_ADDR_BIT_LEN_7,
	.device_address  = snsd::AHT21::I2C_ADDRESS,
	.scl_speed_hz    = 100000,
};

constexpr i2c_device_config_t I2C_DEV_BMP280_CONFIG = {
	.dev_addr_length = I2C_ADDR_BIT_LEN_7,
	.device_address  = snsd::BMP280::I2C_ADDRESS_SDO_VCC,
	.scl_speed_hz    = 100000,
};

// ====== Compounds ======
struct _inet_t : public Compound {
public:
	virtual std::string_view compound_name( void ) const { return "INet"; }

protected:
	virtual status_t _compound_start( [[maybe_unused]]void* ) override {
		ESP_LOGI( Tag, "inet: starting..." );

		Storage.begin( Tag, true );
		auto ssid = Storage.getString( STORAGE_WIFI_SSID, "" );
		auto pwrd = Storage.getString( STORAGE_WIFI_PWRD, "" );
		Storage.end();

		A113_ASSERT_OR( not ssid.isEmpty() and not pwrd.isEmpty() ) {
			ESP_LOGE( Tag, "inet: wifi credentials not set." );
			return A113_ERR_NOT_FOUND;
		}

		WiFi.mode( WIFI_STA );
		WiFi.setAutoReconnect( true );
		WiFi.begin( ssid.c_str(), pwrd.c_str() );
		int wifi_tries = 0;
		do {
			vTaskDelay( 1000_pdms2t );
			A113_ASSERT_OR( ++wifi_tries < 5 ) {
				ESP_LOGE( Tag, "inet: could not connect to wifi: %s", ssid.c_str() );
				return A113_ERR_PLATFORMCALL;
			}
		} while( WiFi.status() != WL_CONNECTED );
		WifiClient.setInsecure();
		ESP_LOGI( Tag, "inet: connected to wifi: %s.", ssid.c_str() );

		ESP_LOGI( Tag, "inet: started." );
		return A113_OK;
	}

	virtual status_t _compound_stop( [[maybe_unused]]void* ) override {
		WiFi.disconnect( true );
		return A113_OK;
	}

} INet;

struct _thingsboard_t : public Compound {
public:
	virtual std::string_view compound_name( void ) const { return "Thingsboard"; }

protected:
	static void _rpc_meas_now( const JsonVariantConst& arg_, JsonDocument& resp_ );
	
	static void _attr_sh_update( const JsonObjectConst& arg_ );

protected:
	inline static Server_Side_RPC< 3, 5 >                        _rpc             = {};
	inline static Attribute_Request< 3, UPPER_LIM_ATTR >         _attr_request    = {};
	inline static Shared_Attribute_Update< 3, UPPER_LIM_ATTR >   _shared_update   = {};

	inline static const std::array< IAPI_Implementation*, 3 >    _APIs   = {
		&_rpc, &_attr_request, &_shared_update
	};
	inline static const std::array< RPC_Callback, 1 >            _RPC_Callbacks   = {
		RPC_Callback{ "meas-now", _rpc_meas_now }
	};
	inline static const std::array< const char*, 1 >             _Attr_Shared     = {
		ATTR_SH_MEAS_LED
	};

protected:
	status_t _compound_start( [[maybe_unused]]void* ) {
		ESP_LOGI( Tag, "thingsboard: starting..." );

		Storage.begin( Tag, true );
		auto server = Storage.getString( STORAGE_TB_SERVER, "" );
		auto port   = Storage.getInt( STORAGE_TB_PORT, 0x0 );
		auto token  = Storage.getString( STORAGE_TB_TOKEN, "" );
		Storage.end();
		
		int cred_bits = ( ( not server.isEmpty() ) << 2 ) |
						( ( port != 0x0 ) << 1 ) |
						( not token.isEmpty() );
		A113_ASSERT_OR( cred_bits == 0b111 ) { 
			ESP_LOGE( Tag, "thingsboard: credentials not set: %d.", cred_bits );
			return A113_ERR_NOT_FOUND;
		}
		
		A113_ASSERT_OR( _dev.connect( server.c_str(), token.c_str(), port ) ) {
			ESP_LOGE( Tag, "thingsboard: bad connection." );
			return A113_ERR_EXCOMCALL;
		}

		A113_ASSERT_OR( _rpc.RPC_Subscribe( _RPC_Callbacks.cbegin(), _RPC_Callbacks.cend() ) ) {
			ESP_LOGE( Tag, "thingsboard: bad rpc subscribe." );
			return A113_ERR_EXCOMCALL;
		}
		
		A113_ASSERT_OR( _shared_update.Shared_Attributes_Subscribe(
			Shared_Attribute_Callback< UPPER_LIM_ATTR >{ &_attr_sh_update, _Attr_Shared.cbegin(), _Attr_Shared.cend() }
		) ) {
			ESP_LOGE( Tag, "thingsboard: bad shared subscribe." );
			return A113_ERR_EXCOMCALL;
		}

		A113_ASSERT_OR( _attr_request.Shared_Attributes_Request(  
			Attribute_Request_Callback< UPPER_LIM_ATTR >{ &_attr_sh_update, ATTR_REQ_TO_US, [](){}, _Attr_Shared }
		) ) {
			ESP_LOGE( Tag, "thingsboard: bad shared request." );
			return A113_ERR_EXCOMCALL;
		}

		A113_ASSERT_OR( pdPASS == xTaskCreate(
			&_thingsboard_t::_main, std::format( "{}/thingsboard/main", Tag ).c_str(),
			8192, this, 4, &_tsk_main
		) ) {
			ESP_LOGE( Tag, "thingsboard: bad main task create." );
			return A113_ERR_SYSCALL;
		}

		ESP_LOGI( Tag, "compound: thinsgboard: started." );
		return A113_OK;
	}

	status_t _compound_stop( [[maybe_unused]]void* ) {
		while( _tsk_main ) vTaskDelay( 100_pdms2t );
		_dev.disconnect();
		return A113_OK;
	}

protected:
	ThingsBoard    _dev         = { MqttClient, 1024, 8192, _APIs };
	TaskHandle_t   _tsk_main    = NULL;
	int64_t        _prev_env    = 0x0;

protected:
	static void _main( void* self_ ) {
		auto*   self   = ( _thingsboard_t* )self_;
	
	for(; self->compound_is_up();) {
		int64_t t_now = esp_timer_get_time() / 1000;

		if( t_now - self->_prev_env > MEAS_INTERVAL_MS ) {
			self->_prev_env = t_now;

			if( Config.meas_led ) {
				gpio_set_level( GPIO_MEAS_LED_NUM, HIGH );
			}

			struct {
				float   temp_a   = 0.0;
				float   temp_b   = 0.0;
				float   press    = 0.0;
				float   hum_air  = 0.0;
				float   hum_sol  = 0.0;
			} env;

			for( int n = 1; n <= Config.adc_soil_meas_count; ++n ) {
				int voltage = 0; 
				adc_oneshot_read( Adc_1, ADC_CHANNEL_2, &voltage );
				adc_cali_raw_to_voltage( Adc_1_Cal, voltage, &voltage );
				env.hum_sol += ( float )voltage;

				vTaskDelay( pdMS_TO_TICKS( Config.adc_soil_period_ms ) );
			}
			env.hum_sol /= 1e3 * Config.adc_soil_meas_count;

			self->_dev.sendTelemetryData( "humidity_soil", env.hum_sol );
			
			Sns_BMP280.store_ctrl_meas( snsd::BMP280::CtrlMeas_TemperatureSampling_1x | snsd::BMP280::CtrlMeas_PressureSampling_1x | snsd::BMP280::CtrlMeas_Power_OneShot );
			vTaskDelay( 20_pdms2t );
			Sns_BMP280.load_data( &env.temp_a, &env.press );
			Sns_BMP280.store_ctrl_meas( snsd::BMP280::CtrlMeas_Power_Low );

			self->_dev.sendTelemetryData( "temperature_a", env.temp_a );
			self->_dev.sendTelemetryData( "pressure", env.press );

			Sns_AHT21.one_shot();
			vTaskDelay( 70_pdms2t );
			Sns_AHT21.load_data( &env.temp_b, &env.hum_air );

			self->_dev.sendTelemetryData( "temperature_b", env.temp_b );
			self->_dev.sendTelemetryData( "humidity_air", env.hum_air );

			gpio_set_level( GPIO_MEAS_LED_NUM, LOW );

			self->_dev.sendAttributeData( "wifi_rssi", WiFi.RSSI() ); 
		}

		self->_dev.loop();
		vTaskDelay( 100_pdms2t );
	}
		vTaskDelete( self->_tsk_main = NULL );
	}

public:
	A113_inline bool connected( void ) { return _dev.connected(); }

public:
	void force_env( void ) { _prev_env = 0x0; }
	
} Thingsboard;

void _thingsboard_t::_rpc_meas_now( const JsonVariantConst& arg_, JsonDocument& resp_ ) {
	ESP_LOGI( Tag, "thingsboard: executing rpc: meas-now..." );
	Thingsboard.force_env();
	ESP_LOGI( Tag, "thingsboard: executed rpc: meas-now..." );
}

void _thingsboard_t::_attr_sh_update( const JsonObjectConst& arg_ ) {
	ESP_LOGI( Tag, "thingsboard: refreshing shared attribs..." );

	for( auto itr : arg_ ) {
		switch( text::hash( itr.key().c_str() ) ) {
			case text::hash( ATTR_SH_MEAS_LED ): {
				Config.meas_led = itr.value().as< uint16_t >();
			break; }
		}
	}

	ESP_LOGI( Tag, "thingsboard: refreshed shared attribs." );
}


// ====== Main ======
status_t init_static( void ) {
	adc_oneshot_new_unit( &ADC_UNIT_1_CONFIG, &Adc_1 );
	adc_oneshot_config_channel( Adc_1, ADC_CHANNEL_2, &ADC_SOIL_MOISTURE_CHANNEL_CONFIG );
	adc_cali_create_scheme_curve_fitting( &ADC_SOIL_MOISTURE_CAL_CONFIG, &Adc_1_Cal );

	gpio_config( &GPIO_MEAS_LED_CONFIG );

	i2c_new_master_bus( &I2C_BUS_0_CONFIG, &I2C_Bus0 );
	I2C_AHT21.bind( I2C_Bus0, I2C_DEV_AHT21_CONFIG, 50 );
	I2C_BMP280.bind( I2C_Bus0, I2C_DEV_BMP280_CONFIG, 50 );
	
	Sns_AHT21.bind_i2c( &I2C_AHT21 ); 
	Sns_AHT21.calib();
	
	Sns_BMP280.bind_i2c( &I2C_BMP280 );
	Sns_BMP280.load_calibs();

	return A113_OK;
}

void query_serial( void );

extern "C" void app_main( void ) {
	auto last_serial_act  = esp_timer_get_time();
	int  eff_serial_delay = SERIAL_FAST_MS;

	vTaskPrioritySet( NULL, 5 ); 

	Serial.begin( SERIAL_BAUD );
	vTaskDelay( 1000_pdms2t );

	initArduino();
	init_static();

	INet.compound_start( nullptr );
	Thingsboard.compound_start( nullptr );

	CmpdCluster.push( { .ref = INet, .restart_if = [] ( auto&, auto& args_ ) -> status_t { 
		A113_ASSERT_OR( WiFi.status() == WL_CONNECTED ) {
			ESP_LOGW( Tag, "compound cluster: restarting INet." );
			return A113_ERR_OPEN;
		}
		return A113_OK;
	} } );
	CmpdCluster.push( { .ref = Thingsboard, .restart_if = [] ( auto&, auto& args_ ) -> status_t { 
		A113_ASSERT_OR( Thingsboard.connected() ) {
			if( args_.attempt >= 5 && eff_serial_delay != SERIAL_FAST_MS ) esp_restart();

			ESP_LOGW( Tag, "compound cluster: restarting Thingsboard." );
			return A113_ERR_OPEN;
		}
		return A113_OK;
	} } );
	CmpdCluster.init( {
		.iterate_interval_ms = 5000,
		.task_stack_depth    = CONFIG_ESP_MAIN_TASK_STACK_SIZE
	} );
	
	for(;;) {
		vTaskDelay( pdMS_TO_TICKS( eff_serial_delay ) );

		if( not Serial.available() ) {
			if( esp_timer_get_time() - last_serial_act > SERIAL_ACT_TO_US ) eff_serial_delay = SERIAL_IDLE_MS;
			continue;
		}
		last_serial_act  = esp_timer_get_time();
		eff_serial_delay = SERIAL_FAST_MS;
		
		query_serial();
	}
}

// ====== Cli ======
text::Fastcli Cli{
	{}, {
	{
		.text = "systemctl",
		.opts = {
			{ .sh0rt = 'r', .l0ng = "restart", .arg = text::Fastcli::Arg_text }
		},
		.fnc = [] ( auto& stencil_ ) -> status_t {
			char opt;  while( opt = stencil_.next() ) {
				switch( opt ) { A113_TEXT_FASTCLI_DEFAULT_STENCIL_CASES
					case 'r': { 
						switch( text::hash( stencil_.arg_text() ) ) {
							case text::hash( "system" ): esp_restart();
							case text::hash( "storage" ): nvs_flash_erase(); break;
						}
					break; }
				}
			}
			stencil_ += "cli: systemctl: done.\n";
			return A113_OK;
		}
	}, {
		.text = "set",
		.opts = {
			{ .sh0rt = 's', .l0ng = STORAGE_WIFI_SSID, .arg = text::Fastcli::Arg_text },
			{ .sh0rt = 'p', .l0ng = STORAGE_WIFI_PWRD, .arg = text::Fastcli::Arg_text },
			{ .sh0rt = 'S', .l0ng = STORAGE_TB_SERVER, .arg = text::Fastcli::Arg_text },
			{ .sh0rt = 'P', .l0ng = STORAGE_TB_PORT,   .arg = text::Fastcli::Arg_i32 },
			{ .sh0rt = 't', .l0ng = STORAGE_TB_TOKEN,  .arg = text::Fastcli::Arg_text }
		},
		.fnc = [] ( auto& stencil_ ) -> status_t {
			char opt;  while( opt = stencil_.next() ) {
				switch( opt ) { A113_TEXT_FASTCLI_DEFAULT_STENCIL_CASES
					case 's': {
						A113_ASSERT_OR( A113_OK == Storage.write< std::string >( Tag, STORAGE_WIFI_SSID, stencil_.arg_text() ) )
							stencil_ += "cli: set wifi ssid: bad storage.\n";
						else
							stencil_ += "cli: set wifi ssid: ok.\n";
					break; }
					case 'p': {
						A113_ASSERT_OR( A113_OK == Storage.write< std::string >( Tag, STORAGE_WIFI_PWRD, stencil_.arg_text() ) )
							stencil_ += "cli: set wifi password: bad storage.\n";
						else
							stencil_ += "cli: set wifi ssid: ok.\n";
					break; }
					case 'S': {
						A113_ASSERT_OR( A113_OK == Storage.write< std::string >( Tag, STORAGE_TB_SERVER, stencil_.arg_text() ) )
							stencil_ += "cli: set tb server: bad storage.\n";
						else
							stencil_ += "cli: set tb server: ok.\n";
					break; }
					case 'P': {
						A113_ASSERT_OR( A113_OK == Storage.write< int >( Tag, STORAGE_TB_PORT, stencil_.arg_i32() ) )
							stencil_ += "cli: set tb port: bad storage.\n";
						else
							stencil_ += "cli: set tb port: ok.\n";
					break; }
					case 't': { 
						A113_ASSERT_OR( A113_OK == Storage.write< std::string >( Tag, STORAGE_TB_TOKEN, stencil_.arg_text() ) )
							stencil_ += "cli: set tb token: bad storage.\n";
						else
							stencil_ += "cli: set tb token: ok.\n";
					break; }

				}
			}
			stencil_ += "cli: set: done.\n";
			return A113_OK;
		}
	} 
} };

void query_serial( void ) { 
	auto line = Serial.readStringUntil( '\n' );

	std::string out;
	if( A113_OK != Cli.execute( line.c_str(), &out ) ) {
		while( Serial.read() != -1 );
	}
	Serial.println( out.c_str() );
}

