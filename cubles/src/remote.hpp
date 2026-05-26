#pragma once

#include "config.hpp"
#include "storage.hpp"

#include <rgh/ucp/daemon.hpp>

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Arduino_MQTT_Client.h>
#include <Server_Side_RPC.h>
#include <Attribute_Request.h>
#include <Shared_Attribute_Update.h>
#include <ThingsBoard.h>

typedef void( *remote_sattr_cb_t )( const JsonObjectConst& );
typedef function< void( void ) > remote_loop_cb_t;

struct remote_init_args_t {
    HVec< Daemon_cluster_FreeRTOS >   dmon_clst    = nullptr;
	vector< RPC_Callback >            rpc_list     = {};
	vector< const char* >             sattr_list   = {};
	remote_sattr_cb_t                 sattr_cb     = nullptr;
	remote_loop_cb_t                  loop_cb      = nullptr;
};

class _Remote {
protected:
    struct _subdaemon_t { _subdaemon_t( _Remote& hyper_ ) : _hyper{ hyper_ } {} _Remote& _hyper; };

public:
	status_t init( const remote_init_args_t& args_ ) {
        ASSERT_OR( _dmon_clst = move( args_.dmon_clst ) ) return ERR_BADARG;
		ASSERT_OR( _loop_cb = move( args_.loop_cb ) ) return ERR_BADARG;

		_rpc_list   = move( args_.rpc_list );
		_sattr_list = move( args_.sattr_list );
		_sattr_cb   = move( args_.sattr_cb );

		_dmon_clst->push( { 
			.ref        = _WiFi, 
			.keep_alive = true,
			.restart_if = [ this ] ( [[maybe_unused]]auto&, auto& args_ ) -> status_t { 
				ASSERT_OR( _WiFi.connected() ) { ESP_LOGW( TAG, "remote: restarting WiFi." ); return ERR_OPEN; }
				return OK;
			}
		} );
		_dmon_clst->push( { 
			.ref        = _Thingsboard, 
			.deps       = { _WiFi },
			.keep_alive = true,
			.restart_if = [ this ] ( [[maybe_unused]]auto&, auto& args_ ) -> status_t { 
				ASSERT_OR( _Thingsboard.connected() ) {
					ESP_LOGW( TAG, "remote: restarting Thingsboard." );
					return ERR_OPEN;
				}
				return OK;
			}
		} );

        return OK;
	}

protected:
    HVec< Daemon_cluster_FreeRTOS >   _dmon_clst     = nullptr;

	WiFiClientSecure                  _wifi_client   = {};
	Arduino_MQTT_Client               _mqtt_client   = { _wifi_client };

	vector< RPC_Callback >            _rpc_list      = {};
	vector< const char* >             _sattr_list    = {};
	remote_sattr_cb_t                 _sattr_cb      = nullptr;
	remote_loop_cb_t                  _loop_cb       = nullptr;        

protected:
	struct _wifi_t : public _subdaemon_t, public Daemon { friend class _Remote;
	public:
		virtual std::string_view daemon_name( void ) const { return "WiFi"; }

    public:
	    _wifi_t( auto& hyper_ ) : _subdaemon_t( hyper_ ) {}

	protected:
		virtual status_t _daemon_start( [[maybe_unused]]void* ) override {
			ESP_LOGI( TAG, "remote: wifi: starting..." );

			auto ssid = Storage.get_wifi_ssid();
			auto pwrd = Storage.get_wifi_pwrd();

			ASSERT_OR( not ssid.isEmpty() and not pwrd.isEmpty() ) {
				ESP_LOGE( TAG, "remote: wifi: credentials empty." );
				return ERR_NOT_FOUND;
			}

			WiFi.mode( WIFI_STA );
			WiFi.setAutoReconnect( true );

			WiFi.begin( ssid.c_str(), pwrd.c_str() );

			uint32_t wifi_tries = 0;
			do {
				vTaskDelay( pdMS_TO_TICKS( REMOTE_WIFI_CONNECT_DELAY_MS ) );
				ASSERT_OR( ++wifi_tries < REMOTE_WIFI_CONNECT_TRIES ) {
					ESP_LOGE( TAG, "remote: wifi: could not connect: %s", ssid.c_str() );
					return ERR_PLATFORMCALL;
				}
			} while( not this->connected() );

			_hyper._wifi_client.setInsecure();

			ESP_LOGI( TAG, "remote: wifi: started, connected: %s.", ssid.c_str() );
			return OK;
		}

		virtual status_t _daemon_stop( [[maybe_unused]]void* ) override {
			_hyper._mqtt_client.disconnect();
			_hyper._wifi_client.stop();
			WiFi.disconnect();

			return OK;
		}

	public:
		inline bool connected( void ) { return WiFi.status() == WL_CONNECTED; }

	} _WiFi{ *this };

	struct _thingsboard_t : public _subdaemon_t, public Daemon { friend class _Remote;
	public:
		virtual std::string_view daemon_name( void ) const { return "Thingsboard"; }

	public:
		_thingsboard_t( auto& hyper_ ) : _subdaemon_t( hyper_ ), _dev{ _hyper._mqtt_client, 1024, 8192, _APIs } {}

	protected:
		ThingsBoardSized< 48 >                                          _dev             ;
		TaskHandle_t                                                    _tsk_main        = nullptr;

		Server_Side_RPC< 1, 5 >                                         _rpc             = {};
		Attribute_Request< 1, REMOTE_TB_MAX_SHARED_ATTRIBUTES >         _sattr_request   = {};
		Shared_Attribute_Update< 1, REMOTE_TB_MAX_SHARED_ATTRIBUTES >   _sattr_update    = {};

		const std::array< IAPI_Implementation*, 3 >                     _APIs            = {
			&_rpc, &_sattr_request, &_sattr_update
		};
		
	protected:
		status_t _daemon_start( [[maybe_unused]]void* ) override {
			ESP_LOGI( TAG, "remote: tb: starting..." );

			auto server = Storage.get_tb_server();
			auto port   = Storage.get_tb_port();
			auto token  = Storage.get_tb_token();
			
			uint8_t cred_bits = ( ( not server.isEmpty() ) << 2 ) |
							    ( ( port != 0x0 ) << 1 ) |
							    ( not token.isEmpty() );
			ASSERT_OR( cred_bits == 0b111 ) { 
				ESP_LOGE( TAG, "remote: tb: credentials invalid: %d.", cred_bits );
				return ERR_NOT_FOUND;
			}
			
			ASSERT_OR( _dev.connect( server.c_str(), token.c_str(), port ) ) {
				ESP_LOGE( TAG, "remote: tb: bad connection." );
				return ERR_EXCOMCALL;
			}

			if( not _hyper._rpc_list.empty() ) {
				ASSERT_OR( _rpc.RPC_Subscribe( _hyper._rpc_list.cbegin(), _hyper._rpc_list.cend() ) ) {
					ESP_LOGE( TAG, "remote: tb: bad rpc subscribe." );
					return ERR_EXCOMCALL;
				}
			}
			
			if( not _hyper._sattr_list.empty() ) {
				ASSERT_OR( _sattr_update.Shared_Attributes_Subscribe(
					Shared_Attribute_Callback< REMOTE_TB_MAX_SHARED_ATTRIBUTES >{ _hyper._sattr_cb, _hyper._sattr_list.cbegin(), _hyper._sattr_list.cend() }
				) ) {
					ESP_LOGE( TAG, "remote: tb: bad shared subscribe." );
					return ERR_EXCOMCALL;
				}

				ASSERT_OR( _sattr_request.Shared_Attributes_Request(  
					Attribute_Request_Callback< REMOTE_TB_MAX_SHARED_ATTRIBUTES >{ _hyper._sattr_cb, REMOTE_TB_SATTR_REQ_TIMEOUT_MS*1000, [](){}, _hyper._sattr_list }
				) ) {
					ESP_LOGE( TAG, "remote: tb: bad shared request." );
					return ERR_EXCOMCALL;
				}
			}

			ASSERT_OR( pdPASS == xTaskCreate(
				&_thingsboard_t::_main, std::format( "{}/remote-tb/main", TAG ).c_str(),
				8192, this, TaskPriority_Default, &_tsk_main
			) ) {
				ESP_LOGE( TAG, "remote: tb: bad main task create." );
				return ERR_SYSCALL;
			}

			ESP_LOGI( TAG, "remote: tb: started." );
			return OK;
		}

		status_t _daemon_stop( [[maybe_unused]]void* ) override {
			while( _tsk_main ) vTaskDelay( 100_pdms2t );

			_sattr_update.Shared_Attributes_Unsubscribe();
			_rpc.RPC_Unsubscribe();
			_dev.disconnect();

			return OK;
		}

	protected:
		static void _main( void* arg_ ) {
			auto*  self = ( _thingsboard_t* )arg_;

		for(; self->daemon_is_up();) {
			vTaskDelay( pdMS_TO_TICKS( REMOTE_TB_MAIN_TASK_DELAY_MS ) );

			ASSERT_AND( self->connected() ) {
				self->_hyper._loop_cb();
				self->_dev.loop();
			}
		}
			vTaskDelete( self->_tsk_main = NULL );
		}

	public:
		inline bool connected( void ) { return _dev.connected(); }
		
	} _Thingsboard{ *this };

public:
	inline bool send_attr( const char* key_, const auto& val_ ) {
		return _Thingsboard._dev.sendAttributeData( key_, val_ );
	} 

	inline auto wifi_rssi( void ) { return WiFi.RSSI(); }

};
inline _Remote Remote;