#include <Arduino.h>
#undef _BV

#include <esp_log.h>
#include <nvs_flash.h>
#include <driver/gpio.h>

#include <rgh/gep/text_utils.hpp>
#include <rgh/gep/fastcli.hpp>
#include <rgh/gep/divergent_ring.hpp>

#include <rgh/ucp/core.hpp>
using namespace rgh;
using namespace rgh::freertos_literals;
#include <rgh/ucp/compound.hpp>

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Preferences.h>
#include <Arduino_MQTT_Client.h>
#include <Server_Side_RPC.h>
#include <Attribute_Request.h>
#include <Shared_Attribute_Update.h>
#include <ThingsBoard.h>

// ====== Configs ======
const char* const             Tag                   = "oca/dwmq-b";

constexpr UBaseType_t         MAIN_TASK_PRIO       = 5;
constexpr UBaseType_t         CMPD_TASK_PRIO       = 5;
constexpr UBaseType_t         TB_MAIN_TASK_PRIO    = 5;

constexpr int                 SERIAL_BAUD_RATE     = 115200;
constexpr int                 SERIAL_FAST_MS       = 100;
constexpr int                 SERIAL_IDLE_MS       = 3000;
constexpr int                 SERIAL_ACT_TO_US     = 20000000;

constexpr gpio_num_t          GPIO_NUM_PWR_OPTO    = GPIO_NUM_32;
constexpr gpio_num_t          GPIO_NUM_PWR_RL      = GPIO_NUM_25;
constexpr gpio_num_t          GPIO_NUM_BLUE_RL     = GPIO_NUM_27;
constexpr gpio_num_t          GPIO_NUM_VUP_RL      = GPIO_NUM_33;
constexpr gpio_num_t          GPIO_NUM_VDWN_RL     = GPIO_NUM_26;
constexpr gpio_num_t          GPIO_NUM_FANS_RL     = GPIO_NUM_19;
constexpr gpio_num_t          GPIO_NUM_LSTR_RL     = GPIO_NUM_18;

constexpr const char* const   STORAGE_WIFI_SSID    = "wifi-ssid";
constexpr const char* const   STORAGE_WIFI_PWRD    = "wifi-pwrd";
constexpr const char* const   STORAGE_TB_SERVER    = "tb-server";
constexpr const char* const   STORAGE_TB_PORT      = "tb-port";
constexpr const char* const   STORAGE_TB_TOKEN     = "tb-token";

constexpr int                 CHECK_COMPOUNDS_MS   = 15000;
constexpr int                 ATTR_UPPER_LIM       = 10;
constexpr int                 ATTR_REQ_TO_US       = 10000000;

constexpr const char* const   ATTR_POWER           = "power";
constexpr const char* const   ATTR_BLUETOOTH       = "bluetooth";
constexpr const char* const   ATTR_FANS            = "fans";
constexpr const char* const   ATTR_LIGHT_STRIP     = "light-strip";
constexpr const char* const   ATTR_GAUGE           = "gauge";

// ====== Fields ====== 
struct storage_t : public Preferences {
_RGH_PROTECTED:
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
} Storage;

class System_ctl {
public:
	inline static constexpr gpio_config_t GPIO_PWR_OPTO_CONFIG = {
		.pin_bit_mask = _BV( GPIO_NUM_PWR_OPTO ),
		.mode         = GPIO_MODE_INPUT,
		.pull_up_en   = GPIO_PULLUP_ENABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type    = GPIO_INTR_DISABLE
	};

	inline static constexpr gpio_config_t GPIO_RELAY_CONFIG = {
		.mode         = GPIO_MODE_OUTPUT_OD,
		.pull_up_en   = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type    = GPIO_INTR_DISABLE
	};

public:
	System_ctl( void ) {
		gpio_config( &GPIO_PWR_OPTO_CONFIG );

		auto gpio_rl_cfg = GPIO_RELAY_CONFIG;
		gpio_rl_cfg.pin_bit_mask = _BV( GPIO_NUM_PWR_RL )  |
								   _BV( GPIO_NUM_BLUE_RL ) |
								   _BV( GPIO_NUM_VUP_RL )  |
								   _BV( GPIO_NUM_VDWN_RL ) |
								   _BV( GPIO_NUM_FANS_RL ) |
								   _BV( GPIO_NUM_LSTR_RL );
		this->open_relays();
		gpio_config( &gpio_rl_cfg );
	}

_RGH_PROTECTED:
	template< typename ...GPIO_NUMS >
	static void _open_relay( GPIO_NUMS... gpio_num_ ) {
		( gpio_set_level( gpio_num_, SET ), ... );
	}

	template< typename ...GPIO_NUMS >
	static void _close_relay( GPIO_NUMS... gpio_num_ ) {
		( gpio_set_level( gpio_num_, RESET ), ... );
	}

public:
	RGH_inline void open_relays( void ) {
		this->_open_relay( 
			GPIO_NUM_PWR_RL, GPIO_NUM_BLUE_RL, GPIO_NUM_VUP_RL, GPIO_NUM_VDWN_RL, GPIO_NUM_FANS_RL, GPIO_NUM_LSTR_RL
		);
	}

_RGH_PROTECTED:
	RGH_inline void _toggle_power( int hold_ms_ = 300 ) {
		this->_close_relay( GPIO_NUM_PWR_RL );
		vTaskDelay( pdMS_TO_TICKS( hold_ms_ ) );
		this->_open_relay( GPIO_NUM_PWR_RL );
	}

public:
	RGH_inline void power_on( void ) {
		if( gpio_get_level( GPIO_NUM_PWR_OPTO ) == RESET ) return;
		this->_toggle_power();
	}
	RGH_inline void power_off( void ) {
		if( gpio_get_level( GPIO_NUM_PWR_OPTO ) == SET ) return;
		this->_toggle_power();
	}

	RGH_inline void bluetooth_on( void ) {
		this->_close_relay( GPIO_NUM_BLUE_RL );
	}
	RGH_inline void bluetooth_off( void ) {
		this->_open_relay( GPIO_NUM_BLUE_RL );
	}

	RGH_inline void fans_on( void ) {
		this->_close_relay( GPIO_NUM_FANS_RL );
	}
	RGH_inline void fans_off( void ) {
		this->_open_relay( GPIO_NUM_FANS_RL );
	}

	RGH_inline void light_strip_on( void ) {
		this->_close_relay( GPIO_NUM_LSTR_RL );
	}
	RGH_inline void light_strip_off( void ) {
		this->_open_relay( GPIO_NUM_LSTR_RL );
	}

} Systemctl;

Compound_cluster_FreeRTOS   CmpdCluster   = {};

class TB_on_WiFi {
_RGH_PROTECTED:
	struct _submodule_t {
		_submodule_t( TB_on_WiFi& hyper_ ) : _hyper{ hyper_ } {}
		TB_on_WiFi&   _hyper;
	};

public:
	TB_on_WiFi( void ) {
		CmpdCluster.push( { 
			.ref        = _INet, 
			.keep_alive = true,
			.restart_if = [] ( [[maybe_unused]]auto&, auto& args_ ) -> status_t { 
				RGH_ASSERT_OR( WiFi.status() == WL_CONNECTED ) {
					ESP_LOGW( Tag, "clkwrk-tb-on-wifi: restarting INet." );
					return RGH_ERR_OPEN;
				}
				return RGH_OK;
			}
		} );
		CmpdCluster.push( { 
			.ref        = _Thingsboard, 
			.deps       = { _INet },
			.keep_alive = true,
			.restart_if = [ this ] ( [[maybe_unused]]auto&, auto& args_ ) -> status_t { 
				RGH_ASSERT_OR( _Thingsboard.connected() ) {
					ESP_LOGW( Tag, "clkwrk-tb-on-wifi: restarting Thingsboard." );
					return RGH_ERR_OPEN;
				}
				return RGH_OK;
			}
		} );
	}

_RGH_PROTECTED:
	WiFiClientSecure      _WifiClient    = {};
	Arduino_MQTT_Client   _MqttClient    = { _WifiClient };

_RGH_PROTECTED:
	struct _inet_t : public _submodule_t, public Compound {
	public:
		virtual std::string_view compound_name( void ) const { return "INet"; }

	_inet_t( auto& hyper_ ) : _submodule_t( hyper_ ) {}

	_RGH_PROTECTED:
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
			_hyper._WifiClient.setInsecure();
			ESP_LOGI( Tag, "inet: connected to wifi: %s.", ssid.c_str() );

			ESP_LOGI( Tag, "inet: started." );
			return RGH_OK;
		}

		virtual status_t _compound_stop( [[maybe_unused]]void* ) override {
			return RGH_OK;
		}

	public:
		RGH_inline bool connected( void ) { return WiFi.status() == WL_CONNECTED; }

	} _INet{ *this };

	struct _thingsboard_t : public _submodule_t, public Compound {
	public:
		virtual std::string_view compound_name( void ) const { return "Thingsboard"; }

	public:
		_thingsboard_t( auto& hyper_ ) : _submodule_t( hyper_ ), _dev{ _hyper._MqttClient, 1024, 8192, _APIs } {}

	_RGH_PROTECTED:
		static void _rpc_hello_there( const JsonVariantConst& arg_, JsonDocument& resp_ );

	_RGH_PROTECTED:
		static void _attr_sh_update( const JsonObjectConst& arg_ );

	_RGH_PROTECTED:
		Server_Side_RPC< 3, 5 >                        _rpc             = {};
		Attribute_Request< 3, ATTR_UPPER_LIM >         _attr_request    = {};
		Shared_Attribute_Update< 3, ATTR_UPPER_LIM >   _shared_update   = {};

		const std::array< IAPI_Implementation*, 3 >    _APIs   = {
			&_rpc, &_attr_request, &_shared_update
		};
		const std::array< RPC_Callback, 1 >            _RPC_Callbacks   = {
			RPC_Callback{ "hello-there", _rpc_hello_there }
		};
		const std::array< const char*, 4 >             _Attr_Shared     = {
			ATTR_POWER,
			ATTR_BLUETOOTH,
			ATTR_FANS,
			ATTR_LIGHT_STRIP
		};

	_RGH_PROTECTED:
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
				Shared_Attribute_Callback< ATTR_UPPER_LIM >{ &_attr_sh_update, _Attr_Shared.cbegin(), _Attr_Shared.cend() }
			) ) {
				ESP_LOGE( Tag, "thingsboard: bad shared subscribe." );
				return RGH_ERR_EXCOMCALL;
			}

			RGH_ASSERT_OR( _attr_request.Shared_Attributes_Request(  
				Attribute_Request_Callback< ATTR_UPPER_LIM >{ &_attr_sh_update, ATTR_REQ_TO_US, [](){}, _Attr_Shared }
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

	_RGH_PROTECTED:
		ThingsBoardSized< 48 >   _dev;
		TaskHandle_t             _tsk_main   = NULL;
		int64_t                  _prev_env   = 0x0;

	_RGH_PROTECTED:
		static void _main( void* self_ ) {
			auto*   self   = ( _thingsboard_t* )self_;
		
		for(; self->compound_is_up();) {
			self->_dev.loop();
			vTaskDelay( 250_pdms2t );
		}
			vTaskDelete( self->_tsk_main = NULL );
		}

	public:
		RGH_inline bool connected( void ) { return _dev.connected(); }
		
	} _Thingsboard{ *this };

} TBonWiFi;

struct config_t {
	
} Config;

// ====== Main ======
void critical_handler( void ) {
	esp_restart();
}

status_t init_static( void ) {
	CmpdCluster.when_critical( [] ( [[maybe_unused]]auto ) -> void { critical_handler(); } );
	CmpdCluster.init( {
		.iterate_interval_ms = CHECK_COMPOUNDS_MS,
		.task_stack_depth    = CONFIG_ESP_MAIN_TASK_STACK_SIZE,
		.task_priority       = CMPD_TASK_PRIO
	} );

	return RGH_OK;
}

void query_serial( void );

extern "C" void app_main( void ) {
	auto last_serial_act  = esp_timer_get_time();
	int  eff_serial_delay = SERIAL_FAST_MS;

	vTaskPrioritySet( NULL, MAIN_TASK_PRIO ); 

	Serial.begin( SERIAL_BAUD_RATE );
	vTaskDelay( 1000_pdms2t );

	initArduino();
	init_static();
	
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

// ====== Thingsboard ======
void TB_on_WiFi::_thingsboard_t::_attr_sh_update( const JsonObjectConst& arg_ ) {
	ESP_LOGI( Tag, "thingsboard: refreshing shared attribs..." );

	for( auto itr : arg_ ) { switch( txt_hash( itr.key().c_str() ) ) {
		case txt_hash( ATTR_GAUGE ): {
			auto pwr = ( int )itr.value().as< float >();

		break; }
		
	#define _ATTR_ON_OFF( attr, func ) \
		case txt_hash( attr ): { \
			if( itr.value().as< uint16_t >() ) Systemctl.func##_on(); else Systemctl.func##_off(); \
		break; }
		_ATTR_ON_OFF( ATTR_POWER, power )
		_ATTR_ON_OFF( ATTR_BLUETOOTH, bluetooth )
		_ATTR_ON_OFF( ATTR_FANS, fans )
		_ATTR_ON_OFF( ATTR_LIGHT_STRIP, light_strip )
	#undef _ATTR_ON_OFF
	} }

	ESP_LOGI( Tag, "thingsboard: refreshed shared attribs." );
}

void TB_on_WiFi::_thingsboard_t::_rpc_hello_there( const JsonVariantConst& arg_, JsonDocument& resp_ ) {
	ESP_LOGI( Tag, "thingsboard: rpc: hello there!" );
}

// ====== Cli ======
Fast_cli Cli = {
	{}, {
	{
		.text = "systemctl",
		.opts = {
			{ .sh0rt = 'r', .l0ng = "restart", .arg = Fast_cli::Arg_text },

			{ .sh0rt = 'b', .l0ng = "blue-off" },
			{ .sh0rt = 'B', .l0ng = "blue-on" },
			{ .sh0rt = 'f', .l0ng = "fans-off" },
			{ .sh0rt = 'F', .l0ng = "fans-on" },
			{ .sh0rt = 'l', .l0ng = "lights-off" },
			{ .sh0rt = 'L', .l0ng = "lights-on" }
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
					
					case 'b': Systemctl.bluetooth_off(); break;
					case 'B': Systemctl.bluetooth_on(); break;
					case 'f': Systemctl.fans_off(); break;
					case 'F': Systemctl.fans_on(); break;
					case 'l': Systemctl.light_strip_off(); break;
					case 'L': Systemctl.light_strip_on(); break;
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

void query_serial( void ) { 
	auto line = Serial.readStringUntil( '\n' );

	std::string out;
	if( RGH_OK != Cli.execute( line.c_str(), &out ) ) {
		while( Serial.read() != -1 );
	}
	Serial.println( out.c_str() );
}

