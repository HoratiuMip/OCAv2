#include <Arduino.h>

#include <esp_log.h>
#include <nvs_flash.h>

#include <a113/gep/text_utils.hpp>
#include <a113/gep/fastcli.hpp>

#include <a113/ucp/core.hpp>
using namespace a113;
using namespace a113::freertos_literals;
#include <a113/ucp/esp32-5x/IO_i2c.hpp>
#include <a113/ucp/sns-drv/bmp280.hpp>
#include <a113/ucp/sns-drv/aht21.hpp>

#include <Preferences.h>

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


// ====== Fields ====== 
const char* const         Tag           = "oca/wp-a";

WiFiClientSecure          WifiClient    = {};
Arduino_MQTT_Client       MqttClient    = { WifiClient };

i2c_master_bus_handle_t   I2C_Bus0      = NULL;
esp32::io::I2C_m2s        I2C_AHT21     = {};
esp32::io::I2C_m2s        I2C_BMP280    = {};

snsd::AHT21               Sns_AHT21     = {};
snsd::BMP280              Sns_BMP280    = {};

storage_t                 Storage             = {};
const char* const         Storage_WIFI_SSID   = "wifi-ssid";
const char* const         Storage_WIFI_PWRD   = "wifi-pwrd";
const char* const         Storage_TB_SERVER   = "tb-server";
const char* const         Storage_TB_PORT     = "tb-port";
const char* const         Storage_TB_TOKEN    = "tb-token";

// ====== Configs ======
constexpr int   SERIAL_BAUD        = 115200;
constexpr int   SERIAL_FAST_MS     = 100;
constexpr int   SERIAL_IDLE_MS     = 3000;
constexpr int   SERIAL_ACT_TO_US   = 20000000;

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

#include <WiFi.h>
#include <WiFiClientSecure.h>

// Maximum amount of attributs we can request or subscribe, has to be set both in the ThingsBoard template list and Attribute_Request_Callback template list
// and should be the same as the amount of variables in the passed array. If it is less not all variables will be requested or subscribed
constexpr size_t MAX_ATTRIBUTES = 3U;

constexpr uint64_t REQUEST_TIMEOUT_MICROSECONDS = 5000U * 1000U;

// Attribute names for attribute request and attribute updates functionality

constexpr const char BLINKING_INTERVAL_ATTR[] = "blinkingInterval";
constexpr const char LED_MODE_ATTR[] = "ledMode";
constexpr const char LED_STATE_ATTR[] = "ledState";


// handle led state and mode changes
volatile bool attributesChanged = false;

// LED modes: 0 - continious state, 1 - blinking
volatile int ledMode = 0;

// Current led state
volatile bool ledState = false;

// Settings for interval in blinking mode
constexpr uint16_t BLINKING_INTERVAL_MS_MIN = 10U;
constexpr uint16_t BLINKING_INTERVAL_MS_MAX = 60000U;
volatile uint16_t blinkingInterval = 1000U;

uint32_t previousStateChange;

// For telemetry
constexpr int16_t telemetrySendInterval = 2000U;
uint32_t previousDataSend;


/// @brief Processes function for RPC call "setLedMode"
/// RPC_Data is a JSON variant, that can be queried using operator[]
/// See https://arduinojson.org/v5/api/jsonvariant/subscript/ for more details
/// @param data Data containing the rpc data that was called and its current value
void processSetLedMode(const JsonVariantConst &data, JsonDocument &response) {
	Serial.println("Received the set led state RPC method");
	
	// Process data
	int new_mode = data;
	
	Serial.print("Mode to change: ");
	Serial.println(new_mode);
	StaticJsonDocument<1> response_doc;
	
	if (new_mode != 0 && new_mode != 1) {
		response_doc["error"] = "Unknown mode!";
		response.set(response_doc);
		return;
	}
	
	ledMode = new_mode;
	
	attributesChanged = true;
	
	// Returning current mode
	response_doc["newMode"] = (int)ledMode;
	response.set(response_doc);
}

/// @brief Update callback that will be called as soon as one of the provided shared attributes changes value,
/// if none are provided we subscribe to any shared attribute change instead
/// @param data Data containing the shared attributes that were changed and their current value
void processSharedAttributes(const JsonObjectConst &data) {
	for (auto it = data.begin(); it != data.end(); ++it) {
		if (strcmp(it->key().c_str(), BLINKING_INTERVAL_ATTR) == 0) {
			const uint16_t new_interval = it->value().as<uint16_t>();
			if (new_interval >= BLINKING_INTERVAL_MS_MIN && new_interval <= BLINKING_INTERVAL_MS_MAX) {
				blinkingInterval = new_interval;
				Serial.print("Blinking interval is set to: ");
				Serial.println(new_interval);
			}
		} else if (strcmp(it->key().c_str(), LED_STATE_ATTR) == 0) {
			ledState = it->value().as<bool>();
			if (LED_BUILTIN != 99) {
				digitalWrite(LED_BUILTIN, ledState);
			}
			Serial.print("LED state is set to: ");
			Serial.println(ledState);
		}
	}
	attributesChanged = true;
}

void processClientAttributes(const JsonObjectConst &data) {
	for (auto it = data.begin(); it != data.end(); ++it) {
		if (strcmp(it->key().c_str(), LED_MODE_ATTR) == 0) {
			const uint16_t new_mode = it->value().as<uint16_t>();
			ledMode = new_mode;
		}
	}
}

// Attribute request did not receive a response in the expected amount of microseconds 
void requestTimedOut() {
	Serial.printf("Attribute request timed out did not receive a response in (%llu) microseconds. Ensure client is connected to the MQTT broker and that the keys actually exist on the target device\n", REQUEST_TIMEOUT_MICROSECONDS);
}

struct _inet_t : public Compound {
protected:
	virtual status_t _compound_start( [[maybe_unused]]void* ) override {
		ESP_LOGI( Tag, "compound: inet: starting..." );

		Storage.begin( Tag, true );
		auto ssid = Storage.getString( Storage_WIFI_SSID, "" );
		auto pwrd = Storage.getString( Storage_WIFI_PWRD, "" );
		Storage.end();

		A113_ASSERT_OR( not ssid.isEmpty() and not pwrd.isEmpty() ) {
			ESP_LOGE( Tag, "compound: inet: wifi credentials not set." );
			return A113_ERR_NOT_FOUND;
		}

		WiFi.begin( ssid.c_str(), pwrd.c_str() );
		int wifi_tries = 0;
		do {
			vTaskDelay( 1000_pdms2t );
			A113_ASSERT_OR( ++wifi_tries < 5 ) {
				ESP_LOGE( Tag, "compound: inet: could not connect to wifi: %s", ssid.c_str() );
				return A113_ERR_PLATFORMCALL;
			}
		} while( WiFi.status() != WL_CONNECTED );
		WifiClient.setInsecure();
		ESP_LOGI( Tag, "compound: inet: connected to wifi: %s.", ssid.c_str() );

		ESP_LOGI( Tag, "compound: inet: started." );
		return A113_OK;
	}

	virtual status_t _compound_stop( [[maybe_unused]]void* ) override {
		WiFi.disconnect();
		return A113_OK;
	}

} INet;

namespace _tb_cfg {
	Server_Side_RPC< 3, 5 >                        rpc;
	Attribute_Request< 2, MAX_ATTRIBUTES >         attr_request;
	Shared_Attribute_Update< 3, MAX_ATTRIBUTES >   shared_update;

	const std::array<IAPI_Implementation*, 3 >   apis   = {
		&rpc,
		&attr_request,
		&shared_update
	};

	inline static constexpr std::array< const char*, 2 > SHARED_ATTRIBUTES_LIST = {
		LED_STATE_ATTR,
		BLINKING_INTERVAL_ATTR
	};

	inline static constexpr std::array< const char*, 1 > CLIENT_ATTRIBUTES_LIST = {
		LED_MODE_ATTR
	};

	const std::array< RPC_Callback, 1 > callbacks = {
		RPC_Callback{ "setLedMode", processSetLedMode }
	};

	const Shared_Attribute_Callback< MAX_ATTRIBUTES > attributes_callback(&processSharedAttributes, SHARED_ATTRIBUTES_LIST.cbegin(), SHARED_ATTRIBUTES_LIST.cend());
	const Attribute_Request_Callback< MAX_ATTRIBUTES > attribute_shared_request_callback(&processSharedAttributes, REQUEST_TIMEOUT_MICROSECONDS, &requestTimedOut, SHARED_ATTRIBUTES_LIST);
	const Attribute_Request_Callback< MAX_ATTRIBUTES > attribute_client_request_callback(&processClientAttributes, REQUEST_TIMEOUT_MICROSECONDS, &requestTimedOut, CLIENT_ATTRIBUTES_LIST);
};
struct _thingsboard_t : public Compound {
protected:
	status_t _compound_start( [[maybe_unused]]void* ) {
		ESP_LOGI( Tag, "compound: thingsboard: starting..." );

		Storage.begin( Tag, true );
		auto server = Storage.getString( Storage_TB_SERVER, "" );
		auto port   = Storage.getInt( Storage_TB_PORT, 0x0 );
		auto token  = Storage.getString( Storage_TB_TOKEN, "" );
		Storage.end();
		
		int cred_bits = ( ( not server.isEmpty() ) << 2 ) |
						( ( port != 0x0 ) << 1 ) |
						( not token.isEmpty() );
		A113_ASSERT_OR( cred_bits == 0b111 ) { 
			ESP_LOGE( Tag, "compound: thingsboard: credentials not set: %d.", cred_bits );
			return A113_ERR_NOT_FOUND;
		}
		
		A113_ASSERT_OR( _dev.connect( server.c_str(), token.c_str(), port ) ) {
			ESP_LOGE( Tag, "compound: thingsboard: bad connection." );
			return A113_ERR_EXCOMCALL;
		}

		A113_ASSERT_OR( _tb_cfg::rpc.RPC_Subscribe( _tb_cfg::callbacks.cbegin(), _tb_cfg::callbacks.cend() ) ) {
			ESP_LOGE( Tag, "compound: thingsboard: bad rpc subscribe." );
			return A113_ERR_EXCOMCALL;
		}
		
		A113_ASSERT_OR( _tb_cfg::shared_update.Shared_Attributes_Subscribe( _tb_cfg::attributes_callback ) ) {
			ESP_LOGE( Tag, "compound: thingsboard: bad shared subscribe." );
			return A113_ERR_EXCOMCALL;
		}

		A113_ASSERT_OR( pdPASS == xTaskCreate(
			&_thingsboard_t::_main, std::format( "{}/thingsboard/main", Tag ).c_str(),
			8192, this, 4, &_tsk_main
		) ) {
			ESP_LOGE( Tag, "compound: thingsboard: bad main task create." );
			return A113_ERR_SYSCALL;
		}

		ESP_LOGI( Tag, "compound: thinsgboard: started." );
		return A113_OK;
	}

	status_t _compound_stop( [[maybe_unused]]void* ) {
		return A113_OK;
	}

protected:
	ThingsBoard    _dev        = { MqttClient, 1024, 8192, _tb_cfg::apis };
	TaskHandle_t   _tsk_main   = NULL;

protected:
	static void _main( void* self_ ) {
		auto* self = ( _thingsboard_t* )self_;
	for( ;; ) {
		float temp = 0.0;
		
		Sns_BMP280.store_ctrl_meas( snsd::BMP280::CtrlMeas_TemperatureSampling_1x | snsd::BMP280::CtrlMeas_PressureSampling_1x | snsd::BMP280::CtrlMeas_Power_OneShot );
		vTaskDelay( pdMS_TO_TICKS( 10 ) );
		Sns_BMP280.load_data( &temp, nullptr );
		Sns_BMP280.store_ctrl_meas( snsd::BMP280::CtrlMeas_Power_Low );
		self->_dev.sendTelemetryData("temperature", temp);
		self->_dev.loop();

		vTaskDelay( 3000_pdms2t );
	}
	}
	
} Thingsboard;

status_t init_static( void ) {
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
	vTaskPrioritySet( NULL, 5 ); 

	Serial.begin( SERIAL_BAUD );
	vTaskDelay( 1000_pdms2t );

	initArduino();
	init_static();

	INet.compound_start( nullptr );
	Thingsboard.compound_start( nullptr );
	
	auto last_serial_act  = esp_timer_get_time();
	int  eff_serial_delay = SERIAL_FAST_MS;
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
							case text::hash( "system" ): esp_restart(); *(volatile int*)0x0=*(int*)0x0; return A113_ERR_TERMINATED;
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
			{ .sh0rt = 's', .l0ng = Storage_WIFI_SSID, .arg = text::Fastcli::Arg_text },
			{ .sh0rt = 'p', .l0ng = Storage_WIFI_PWRD, .arg = text::Fastcli::Arg_text },
			{ .sh0rt = 'S', .l0ng = Storage_TB_SERVER, .arg = text::Fastcli::Arg_text },
			{ .sh0rt = 'P', .l0ng = Storage_TB_PORT,   .arg = text::Fastcli::Arg_i32 },
			{ .sh0rt = 't', .l0ng = Storage_TB_TOKEN,  .arg = text::Fastcli::Arg_text }
		},
		.fnc = [] ( auto& stencil_ ) -> status_t {
			char opt;  while( opt = stencil_.next() ) {
				switch( opt ) { A113_TEXT_FASTCLI_DEFAULT_STENCIL_CASES
					case 's': {
						A113_ASSERT_OR( A113_OK == Storage.write< std::string >( Tag, Storage_WIFI_SSID, stencil_.arg_text() ) )
							stencil_ += "cli: set wifi ssid: bad storage.\n";
						else
							stencil_ += "cli: set wifi ssid: ok.\n";
					break; }
					case 'p': {
						A113_ASSERT_OR( A113_OK == Storage.write< std::string >( Tag, Storage_WIFI_PWRD, stencil_.arg_text() ) )
							stencil_ += "cli: set wifi password: bad storage.\n";
						else
							stencil_ += "cli: set wifi ssid: ok.\n";
					break; }
					case 'S': {
						A113_ASSERT_OR( A113_OK == Storage.write< std::string >( Tag, Storage_TB_SERVER, stencil_.arg_text() ) )
							stencil_ += "cli: set tb server: bad storage.\n";
						else
							stencil_ += "cli: set tb server: ok.\n";
					break; }
					case 'P': {
						A113_ASSERT_OR( A113_OK == Storage.write< int >( Tag, Storage_TB_PORT, stencil_.arg_i32() ) )
							stencil_ += "cli: set tb port: bad storage.\n";
						else
							stencil_ += "cli: set tb port: ok.\n";
					break; }
					case 't': { 
						A113_ASSERT_OR( A113_OK == Storage.write< std::string >( Tag, Storage_TB_TOKEN, stencil_.arg_text() ) )
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

