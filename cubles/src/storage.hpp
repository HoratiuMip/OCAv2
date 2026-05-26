#pragma once

#include "config.hpp"

#include <nvs_flash.h>
#include <Preferences.h>

/**
 * @brief Flash storage master.
 */
class _Storage : protected Preferences {
protected:
	mutex   _mtx   = {};

protected:
	bool _begin( const char* sector_, bool ro_ ) { 
		_mtx.lock();
		return this->Preferences::begin( sector_, ro_ ); 
	}

	void _end( void ) {
		this->Preferences::end();
		_mtx.unlock();
	}

protected:
	template< typename _T_ >
	status_t _write( const char* key_, const _T_& val_ ) {
		this->_begin( STORAGE_SECTOR, false );

		bool wres = false;
		if constexpr( is_same_v< string, _T_ > ) {
			wres = val_.length() == this->putString( key_, val_.c_str() );
		} else if constexpr( is_same_v< int, _T_ > ) {
			wres = sizeof( int ) == this->putInt( key_, val_ );
		}

		this->_end();
		return wres ? OK : ERR_PLATFORMCALL;
	}

public:
    void erase( void ) { nvs_flash_erase(); }

public:
    inline status_t set_wifi_ssid( const string& ssid_ )  { return this->_write< string >( STORAGE_WIFI_SSID, ssid_ ); }
    inline status_t set_wifi_pwrd( const string& pwrd_ )  { return this->_write< string >( STORAGE_WIFI_PWRD, pwrd_ ); }
    inline status_t set_tb_server( const string& srvip_ ) { return this->_write< string >( STORAGE_TB_SERVER, srvip_ ); }
    inline status_t set_tb_port  ( const int32_t port_ )  { return this->_write< int >   ( STORAGE_TB_PORT,   port_ ); }
    inline status_t set_tb_token ( const string& tok_ )   { return this->_write< string >( STORAGE_TB_TOKEN,  tok_ ); }

public:
    inline auto get_wifi_ssid( void ) { this->_begin( STORAGE_SECTOR, true ); auto ret = this->Preferences::getString( STORAGE_WIFI_SSID, "" ); this->_end(); return ret; }
    inline auto get_wifi_pwrd( void ) { this->_begin( STORAGE_SECTOR, true ); auto ret = this->Preferences::getString( STORAGE_WIFI_PWRD, "" ); this->_end(); return ret; }
    inline auto get_tb_server( void ) { this->_begin( STORAGE_SECTOR, true ); auto ret = this->Preferences::getString( STORAGE_TB_SERVER, "" ); this->_end(); return ret; }
    inline auto get_tb_port( void )   { this->_begin( STORAGE_SECTOR, true ); auto ret = this->Preferences::getInt   ( STORAGE_TB_PORT, 0x0 );  this->_end(); return ret; }
    inline auto get_tb_token( void )  { this->_begin( STORAGE_SECTOR, true ); auto ret = this->Preferences::getString( STORAGE_TB_TOKEN, "" );  this->_end(); return ret; }

};
inline _Storage Storage;