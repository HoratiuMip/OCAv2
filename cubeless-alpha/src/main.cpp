#include <Arduino.h>

#include <esp_log.h>
#include <nvs_flash.h>
#include <driver/gpio.h>

#include <rgh/gep/text_utils.hpp>
#include <rgh/gep/fastcli.hpp>

#include <rgh/ucp/core.hpp>
using namespace rgh;
using namespace rgh::freertos_literals;
#include <rgh/ucp/compound.hpp>

#include <Preferences.h>

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Arduino_MQTT_Client.h>
#include <Server_Side_RPC.h>
#include <Attribute_Request.h>
#include <Shared_Attribute_Update.h>
#include <ThingsBoard.h>


template<


	int SERIAL_BAUD_RATE_,
	int SERIAL_FAST_MS_,
	int SERIAL_IDLE_MS_,
	int SERIAL_ACT_TO_MS_
> class Thingsboard_clockwork {
public:
	Thingsboard_clockwork( void ) {
		Serial.begin( SERIAL_BAUD_RATE_ );
		vTaskDelay()
	}

public:
	Fast_cli   Cli   = {
		{}, {
		{
			.text = "systemctl",
			.opts = {
				{ .sh0rt = 'r', .l0ng = "restart", .arg = Fast_cli::Arg_text }
			},
			.fnc = [] ( auto& stencil_ ) -> status_t {
				char opt;  while( opt = stencil_.next() ) {
					switch( opt ) { RGH_FASTCLI_DEFAULT_STENCIL_CASES
						case 'r': { 
							switch( txt_hash( stencil_.arg_text() ) ) {
								case txt_hash( "system" ): esp_restart();
								case txt_hash( "storage" ): nvs_flash_erase(); break;
							}
						break; }
					}
				}
				stencil_ += "cli: systemctl: done.\n";
				return RGH_OK;
			}
		}, {
			.text = "set",
			.opts = {
				{ .sh0rt = 's', .l0ng = STORAGE_WIFI_SSID, .arg = Fast_cli::Arg_text },
				{ .sh0rt = 'p', .l0ng = STORAGE_WIFI_PWRD, .arg = Fast_cli::Arg_text },
				{ .sh0rt = 'S', .l0ng = STORAGE_TB_SERVER, .arg = Fast_cli::Arg_text },
				{ .sh0rt = 'P', .l0ng = STORAGE_TB_PORT,   .arg = Fast_cli::Arg_i32 },
				{ .sh0rt = 't', .l0ng = STORAGE_TB_TOKEN,  .arg = Fast_cli::Arg_text }
			},
			.fnc = [] ( auto& stencil_ ) -> status_t {
				char opt;  while( opt = stencil_.next() ) {
					switch( opt ) { RGH_FASTCLI_DEFAULT_STENCIL_CASES
						case 's': {
							RGH_ASSERT_OR( RGH_OK == Storage.write< std::string >( Tag, STORAGE_WIFI_SSID, stencil_.arg_text() ) )
								stencil_ += "cli: set wifi ssid: bad storage.\n";
							else
								stencil_ += "cli: set wifi ssid: ok.\n";
						break; }
						case 'p': {
							RGH_ASSERT_OR( RGH_OK == Storage.write< std::string >( Tag, STORAGE_WIFI_PWRD, stencil_.arg_text() ) )
								stencil_ += "cli: set wifi password: bad storage.\n";
							else
								stencil_ += "cli: set wifi password: ok.\n";
						break; }
						case 'S': {
							RGH_ASSERT_OR( RGH_OK == Storage.write< std::string >( Tag, STORAGE_TB_SERVER, stencil_.arg_text() ) )
								stencil_ += "cli: set tb server: bad storage.\n";
							else
								stencil_ += "cli: set tb server: ok.\n";
						break; }
						case 'P': {
							RGH_ASSERT_OR( RGH_OK == Storage.write< int >( Tag, STORAGE_TB_PORT, stencil_.arg_i32() ) )
								stencil_ += "cli: set tb port: bad storage.\n";
							else
								stencil_ += "cli: set tb port: ok.\n";
						break; }
						case 't': { 
							RGH_ASSERT_OR( RGH_OK == Storage.write< std::string >( Tag, STORAGE_TB_TOKEN, stencil_.arg_text() ) )
								stencil_ += "cli: set tb token: bad storage.\n";
							else
								stencil_ += "cli: set tb token: ok.\n";
						break; }

					}
				}
				stencil_ += "cli: set: done.\n";
				return RGH_OK;
			}
		} 
	} };

};


// ====== Structs ======
struct storage_t : public Preferences {
protected:
	std::mutex   _mtx;

public:
	RGH_inline bool begin( const char* sector_, bool ro_ ) { 
		_mtx.lock();
		return this->Preferences::begin( sector_, ro_ ); 
	}

	RGH_inline void end( void ) {
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
		return wres ? RGH_OK : RGH_ERR_PLATFORMCALL;
	}
};

struct config_t {
	
};

// ====== Fields ====== 
const char* const           Tag           = "oca/cbls-a";

config_t                    Config        = {};
Compound_cluster_FreeRTOS   CmpdCluster   = {};

WiFiClientSecure            WifiClient    = {};
Arduino_MQTT_Client         MqttClient    = { WifiClient };

storage_t                   Storage       = {};

// ====== Configs ======
constexpr UBaseType_t         MAIN_TASK_PRIO      = 5;
constexpr UBaseType_t         CMPD_TASK_PRIO      = 5;
constexpr UBaseType_t         TB_MAIN_TASK_PRIO   = 4;

constexpr int                 SERIAL_BAUD         = 115200;
constexpr int                 SERIAL_FAST_MS      = 100;
constexpr int                 SERIAL_IDLE_MS      = 3000;
constexpr int                 SERIAL_ACT_TO_US    = 20000000;

constexpr const char* const   STORAGE_WIFI_SSID   = "wifi-ssid";
constexpr const char* const   STORAGE_WIFI_PWRD   = "wifi-pwrd";
constexpr const char* const   STORAGE_TB_SERVER   = "tb-server";
constexpr const char* const   STORAGE_TB_PORT     = "tb-port";
constexpr const char* const   STORAGE_TB_TOKEN    = "tb-token";

constexpr int                 UPPER_LIM_ATTR      = 3;
constexpr int                 ATTR_REQ_TO_US      = 10000000;

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

		RGH_ASSERT_OR( not ssid.isEmpty() and not pwrd.isEmpty() ) {
			ESP_LOGE( Tag, "inet: wifi credentials not set." );
			return RGH_ERR_NOT_FOUND;
		}

		WiFi.mode( WIFI_STA );
		WiFi.setAutoReconnect( true );

		WiFi.begin( ssid.c_str(), pwrd.c_str() );

		int wifi_tries = 0;
		do {
			vTaskDelay( 1000_pdms2t );
			RGH_ASSERT_OR( ++wifi_tries < 5 ) {
				ESP_LOGE( Tag, "inet: could not connect to wifi: %s", ssid.c_str() );
				return RGH_ERR_PLATFORMCALL;
			}
		} while( WiFi.status() != WL_CONNECTED );
		WifiClient.setInsecure();
		ESP_LOGI( Tag, "inet: connected to wifi: %s.", ssid.c_str() );

		ESP_LOGI( Tag, "inet: started." );
		return RGH_OK;
	}

	virtual status_t _compound_stop( [[maybe_unused]]void* ) override {
		return RGH_OK;
	}

public:
	RGH_inline bool connected( void ) { return WiFi.status() == WL_CONNECTED; }

} INet;

struct _thingsboard_t : public Compound {
public:
	virtual std::string_view compound_name( void ) const { return "Thingsboard"; }

protected:
	static void _rpc_hello_there( const JsonVariantConst& arg_, JsonDocument& resp_ );

protected:
	static void _attr_sh_update( const JsonObjectConst& arg_ );

protected:
	inline static Server_Side_RPC< 3, 5 >                        _rpc             = {};
	inline static Attribute_Request< 3, UPPER_LIM_ATTR >         _attr_request    = {};
	inline static Shared_Attribute_Update< 3, UPPER_LIM_ATTR >   _shared_update   = {};

	inline static const std::array< IAPI_Implementation*, 3 >    _APIs   = {
		&_rpc, &_attr_request, &_shared_update
	};
	inline static const std::array< RPC_Callback, 1 >            _RPC_Callbacks   = {
		RPC_Callback{ "hello-there", _rpc_hello_there }
	};
	inline static const std::array< const char*, 1 >             _Attr_Shared     = {
		""
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
		RGH_ASSERT_OR( cred_bits == 0b111 ) { 
			ESP_LOGE( Tag, "thingsboard: credentials not set: %d.", cred_bits );
			return RGH_ERR_NOT_FOUND;
		}
		
		RGH_ASSERT_OR( _dev.connect( server.c_str(), token.c_str(), port ) ) {
			ESP_LOGE( Tag, "thingsboard: bad connection." );
			return RGH_ERR_EXCOMCALL;
		}

		RGH_ASSERT_OR( _rpc.RPC_Subscribe( _RPC_Callbacks.cbegin(), _RPC_Callbacks.cend() ) ) {
			ESP_LOGE( Tag, "thingsboard: bad rpc subscribe." );
			return RGH_ERR_EXCOMCALL;
		}
		
		RGH_ASSERT_OR( _shared_update.Shared_Attributes_Subscribe(
			Shared_Attribute_Callback< UPPER_LIM_ATTR >{ &_attr_sh_update, _Attr_Shared.cbegin(), _Attr_Shared.cend() }
		) ) {
			ESP_LOGE( Tag, "thingsboard: bad shared subscribe." );
			return RGH_ERR_EXCOMCALL;
		}

		RGH_ASSERT_OR( _attr_request.Shared_Attributes_Request(  
			Attribute_Request_Callback< UPPER_LIM_ATTR >{ &_attr_sh_update, ATTR_REQ_TO_US, [](){}, _Attr_Shared }
		) ) {
			ESP_LOGE( Tag, "thingsboard: bad shared request." );
			return RGH_ERR_EXCOMCALL;
		}

		RGH_ASSERT_OR( pdPASS == xTaskCreate(
			&_thingsboard_t::_main, std::format( "{}/thingsboard/main", Tag ).c_str(),
			8192, this, TB_MAIN_TASK_PRIO, &_tsk_main
		) ) {
			ESP_LOGE( Tag, "thingsboard: bad main task create." );
			return RGH_ERR_SYSCALL;
		}

		ESP_LOGI( Tag, "compound: thinsgboard: started." );
		return RGH_OK;
	}

	status_t _compound_stop( [[maybe_unused]]void* ) {
		while( _tsk_main ) vTaskDelay( 100_pdms2t );
		return RGH_OK;
	}

protected:
	ThingsBoardSized< 48 >   _dev        = { MqttClient, 1024, 8192, _APIs };
	TaskHandle_t             _tsk_main   = NULL;
	int64_t                  _prev_env   = 0x0;

protected:
	static void _main( void* self_ ) {
		auto*   self   = ( _thingsboard_t* )self_;
	
	for(; self->compound_is_up();) {
		
		self->_dev.loop();
		vTaskDelay( 100_pdms2t );
	}
		vTaskDelete( self->_tsk_main = NULL );
	}

public:
	RGH_inline bool connected( void ) { return _dev.connected(); }
	
} Thingsboard;

void _thingsboard_t::_attr_sh_update( const JsonObjectConst& arg_ ) {
	ESP_LOGI( Tag, "thingsboard: refreshing shared attribs..." );

	for( auto itr : arg_ ) {
		switch( txt_hash( itr.key().c_str() ) ) {
		}
	}

	ESP_LOGI( Tag, "thingsboard: refreshed shared attribs." );
}

void _thingsboard_t::_rpc_hello_there( const JsonVariantConst& arg_, JsonDocument& resp_ ) {
	ESP_LOGI( Tag, "thingsboard: rpc: hello there!" );
}


// ====== Main ======
status_t init_static( void ) {
	return RGH_OK;
}

void query_serial( void );

extern "C" void app_main( void ) {
	auto last_serial_act  = esp_timer_get_time();
	int  eff_serial_delay = SERIAL_FAST_MS;

	vTaskPrioritySet( NULL, MAIN_TASK_PRIO ); 

	Serial.begin( SERIAL_BAUD );
	vTaskDelay( 1000_pdms2t );

	initArduino();
	init_static();

	CmpdCluster.push( { 
		.ref = INet, 
		.keep_alive = true,
		.restart_if = [] ( [[maybe_unused]]auto&, auto& args_ ) -> status_t { 
			RGH_ASSERT_OR( WiFi.status() == WL_CONNECTED ) {
				ESP_LOGW( Tag, "compound cluster: restarting INet." );
				return RGH_ERR_OPEN;
			}
			return RGH_OK;
		}
	} );
	CmpdCluster.push( { 
		.ref = Thingsboard, 
		.deps = { INet },
		.keep_alive = true,
		.restart_if = [] ( [[maybe_unused]]auto&, auto& args_ ) -> status_t { 
			RGH_ASSERT_OR( Thingsboard.connected() ) {
				ESP_LOGW( Tag, "compound cluster: restarting Thingsboard." );
				return RGH_ERR_OPEN;
			}
			return RGH_OK;
		},
		.critical_n_restarts = 3
	} );

	CmpdCluster.when_critical( [] ( auto ) -> void {
		esp_restart();
	} );

	CmpdCluster.init( {
		.iterate_interval_ms = 15000,
		.task_stack_depth    = CONFIG_ESP_MAIN_TASK_STACK_SIZE,
		.task_priority       = CMPD_TASK_PRIO
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

void query_serial( void ) { 
	auto line = Serial.readStringUntil( '\n' );

	std::string out;
	if( RGH_OK != Cli.execute( line.c_str(), &out ) ) {
		while( Serial.read() != -1 );
	}
	Serial.println( out.c_str() );
}

