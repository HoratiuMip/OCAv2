/**
 * @brief: OCA-Cubles version Alpha.
 * @details: Garden automation cabinet.
 * @authors: Vatca "Mipsan" Tudor-Horatiu
 */

#include "config.hpp"

/* =-. RGH .-= */
#include <rgh/gep/text_utils.hpp>
#include <rgh/gep/fastcli.hpp>

#include "hidropump.hpp"
#include "remote.hpp"
#include "storage.hpp"

Daemon_cluster_FreeRTOS   Daemon_Cluster;

void init_static( void ) {
    Hidro_Pump.init();

    Daemon_Cluster.when_critical( [] ( [[maybe_unused]]auto ) -> void { critical_handler(); } );
	Daemon_Cluster.init( {
		.iterate_interval_ms = DAEMON_CLUSTER_ITERATE_INTERVAL_MS,
		.task_stack_depth    = CONFIG_ESP_MAIN_TASK_STACK_SIZE,
		.task_priority       = TaskPriority_High
	} );

    Remote.init( remote_init_args_t{
        .dmon_clst = Daemon_Cluster,
		.loop_cb   = [
			prev_class_I_sc = esp_timer_get_time()
		] ( void ) mutable -> void {
			int64_t now_sc = esp_timer_get_time();
		
			ASSERT_AND( now_sc - prev_class_I_sc >= REMOTE_CLASS_I_INTERVAL_MS*1000 ) {
				prev_class_I_sc = now_sc;

				Remote.send_attr( "wifi-rssi", Remote.wifi_rssi() );
			}
		} 
    } );
}

void query_serial_for_cli( void );

extern "C" void app_main( void ) {
    vTaskPrioritySet( nullptr, TaskPriority_Default );

	auto last_serial_act  = esp_timer_get_time();
	int  eff_serial_delay = USB_SERIAL_FAST_MS;

	Serial.begin( USB_SERIAL_BAUD_RATE );
	vTaskDelay( 1000_pdms2t );

	initArduino();
    init_static();
	
	for(;;) {
		vTaskDelay( pdMS_TO_TICKS( eff_serial_delay ) );

		if( not Serial.available() ) {
			if( esp_timer_get_time() - last_serial_act > USB_SERIAL_TIMEOUT_MS*1000 ) eff_serial_delay = USB_SERIAL_SLOW_MS;
			continue;
		}
		last_serial_act  = esp_timer_get_time();
		eff_serial_delay = USB_SERIAL_FAST_MS;
		
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

            { .sh0rt = 'h', .l0ng = "hidropump-off" },
            { .sh0rt = 'H', .l0ng = "hidropump-on", .arg = Fast_cli::Arg_i32 }
		},
		.fnc = [] ( auto& stc_ ) -> status_t {
			char opt;  while( opt = stc_.next() ) {
				switch( opt ) { RGH_FASTCLI_DEFAULT_STENCIL_CASES
					case 'r': { 
						switch( txt_hash( stc_.arg_text() ) ) {
							case txt_hash( "system" ): esp_restart();
							case txt_hash( "storage" ): Storage.erase(); break;
						}
					break; }

                    case 'h': Hidro_Pump.disengage(); break;
                    case 'H': Hidro_Pump.engage( stc_.arg_i32() ); break;
				}
			}
			stc_ += "cli: systemctl: done.\n";
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
		.fnc = [] ( auto& stc_ ) -> status_t {
			char opt;  while( opt = stc_.next() ) {
				switch( opt ) { RGH_FASTCLI_DEFAULT_STENCIL_CASES
					case 's': {
						ASSERT_OR( 0x0 == Storage.set_wifi_ssid( stc_.arg_text() ) )
						stc_ += "cli: set wifi ssid: bad storage.\n"; else stc_ += "cli: set wifi ssid: ok.\n";
					break; }
					case 'p': {
						ASSERT_OR( 0x0 == Storage.set_wifi_pwrd( stc_.arg_text() ) )
						stc_ += "cli: set wifi pwrd: bad storage.\n"; else stc_ += "cli: set wifi pwrd: ok.\n";
					break; }
					case 'S': {
						ASSERT_OR( 0x0 == Storage.set_tb_server( stc_.arg_text() ) )
						stc_ += "cli: set tb server: bad storage.\n"; else stc_ += "cli: set tb server: ok.\n";
					break; }
					case 'P': {
						ASSERT_OR( 0x0 == Storage.set_tb_port( stc_.arg_i32() ) )
						stc_ += "cli: set tb port: bad storage.\n"; else stc_ += "cli: set tb port: ok.\n";
					break; }
					case 't': { 
						ASSERT_OR( 0x0 == Storage.set_tb_token( stc_.arg_text() ) )
						stc_ += "cli: set tb token: bad storage.\n"; else stc_ += "cli: set tb token: ok.\n";
					break; }

				}
			}
			stc_ += "cli: set: done.\n";
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

