#include <Arduino.h>

#include <esp_log.h>

#include <a113/ucp/core.hpp>
using namespace a113;
using namespace a113::freertos_literals;
#include <a113/ucp/esp32-5x/IO_i2c.hpp>
#include <a113/ucp/sns-drv/bmp280.hpp>
#include <a113/ucp/sns-drv/aht21.hpp>

#include <Preferences.h>

// ====== Fields ====== 
const char* const         Tag          = "oca/wp-a";

i2c_master_bus_handle_t   I2C_Bus0     = NULL;
esp32::io::I2C_m2s        I2C_AHT21    = {};
esp32::io::I2C_m2s        I2C_BMP280   = {};

snsd::AHT21               Sns_AHT21    = {};
snsd::BMP280              Sns_BMP280   = {};

Preferences               Psettings;

// ====== Configs ======
constexpr int SERIAL_BAUD = 115200;

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

#define LED_BUILTIN 7

#include <Arduino_MQTT_Client.h>
#include <Server_Side_RPC.h>
#include <Attribute_Request.h>
#include <Shared_Attribute_Update.h>
#include <ThingsBoard.h>

constexpr char WIFI_SSID[] = "";
constexpr char WIFI_PASSWORD[] = "";

// See https://thingsboard.io/docs/getting-started-guides/helloworld/
// to understand how to obtain an access token
constexpr char TOKEN[] =
#include "_dev_token"
;

// Thingsboard we want to establish a connection too
constexpr char THINGSBOARD_SERVER[] = "";
// MQTT port used to communicate with the server, 1883 is the default unencrypted MQTT port.
constexpr uint16_t THINGSBOARD_PORT = 20001U;

// Maximum size packets will ever be sent or received by the underlying MQTT client,
// if the size is to small messages might not be sent or received messages will be discarded
constexpr uint32_t MAX_MESSAGE_SIZE = 1024U;

// Maximum amount of attributs we can request or subscribe, has to be set both in the ThingsBoard template list and Attribute_Request_Callback template list
// and should be the same as the amount of variables in the passed array. If it is less not all variables will be requested or subscribed
constexpr size_t MAX_ATTRIBUTES = 3U;

constexpr uint64_t REQUEST_TIMEOUT_MICROSECONDS = 5000U * 1000U;

// Attribute names for attribute request and attribute updates functionality

constexpr const char BLINKING_INTERVAL_ATTR[] = "blinkingInterval";
constexpr const char LED_MODE_ATTR[] = "ledMode";
constexpr const char LED_STATE_ATTR[] = "ledState";

// Initialize underlying client, used to establish a connection
WiFiClientSecure wifiClient;

// Initalize the Mqtt client instance
Arduino_MQTT_Client mqttClient(wifiClient);

// Initialize used apis
Server_Side_RPC<3U, 5U> rpc;
Attribute_Request<2U, MAX_ATTRIBUTES> attr_request;
Shared_Attribute_Update<3U, MAX_ATTRIBUTES> shared_update;

const std::array<IAPI_Implementation*, 3U> apis = {
	&rpc,
	&attr_request,
	&shared_update
};

// Initialize ThingsBoard instance with the maximum needed buffer size, stack size and the apis we want to use
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE, Default_Max_Stack_Size, apis);

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

// List of shared attributes for subscribing to their updates
constexpr std::array<const char *, 2U> SHARED_ATTRIBUTES_LIST = {
	LED_STATE_ATTR,
	BLINKING_INTERVAL_ATTR
};

// List of client attributes for requesting them (Using to initialize device states)
constexpr std::array<const char *, 1U> CLIENT_ATTRIBUTES_LIST = {
	LED_MODE_ATTR
};


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


// Optional, keep subscribed shared attributes empty instead,
// and the callback will be called for every shared attribute changed on the device,
// instead of only the one that were entered instead
const std::array<RPC_Callback, 1U> callbacks = {
	RPC_Callback{ "setLedMode", processSetLedMode }
};


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

const Shared_Attribute_Callback<MAX_ATTRIBUTES> attributes_callback(&processSharedAttributes, SHARED_ATTRIBUTES_LIST.cbegin(), SHARED_ATTRIBUTES_LIST.cend());
const Attribute_Request_Callback<MAX_ATTRIBUTES> attribute_shared_request_callback(&processSharedAttributes, REQUEST_TIMEOUT_MICROSECONDS, &requestTimedOut, SHARED_ATTRIBUTES_LIST);
const Attribute_Request_Callback<MAX_ATTRIBUTES> attribute_client_request_callback(&processClientAttributes, REQUEST_TIMEOUT_MICROSECONDS, &requestTimedOut, CLIENT_ATTRIBUTES_LIST);

void loop() {
	delay(10);
	
	if (!reconnect()) {
		vTaskDelay( pdMS_TO_TICKS( 500 ) );
		return;
	}
	
	if (!tb.connected()) {
		// Connect to the ThingsBoard
		Serial.print("Connecting to: ");
		Serial.print(THINGSBOARD_SERVER);
		Serial.print(" with token ");
		Serial.println(TOKEN);
		if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
			Serial.println("Failed to connect");
			return;
		}
		Serial.println( "OK" );
		
		// Sending a MAC address as an attribute
		tb.sendAttributeData("macAddress", WiFi.macAddress().c_str());
		
		Serial.println("Subscribing for RPC...");
		// Perform a subscription. All consequent data processing will happen in
		// processSetLedState() and processSetLedMode() functions,
		// as denoted by callbacks array.
		if (!rpc.RPC_Subscribe(callbacks.cbegin(), callbacks.cend())) {
			Serial.println("Failed to subscribe for RPC");
			return;
		}
		
		if (!shared_update.Shared_Attributes_Subscribe(attributes_callback)) {
			Serial.println("Failed to subscribe for shared attribute updates");
			return;
		}
		
		Serial.println("Subscribe done");
		
		// Request current states of shared attributes
		if (!attr_request.Shared_Attributes_Request(attribute_shared_request_callback)) {
			Serial.println("Failed to request for shared attributes");
			return;
		}
		
		// Request current states of client attributes
		if (!attr_request.Client_Attributes_Request(attribute_client_request_callback)) {
			Serial.println("Failed to request for client attributes");
			return;
		}
	}
	
	if (attributesChanged) {
		attributesChanged = false;
		if (ledMode == 0) {
			previousStateChange = millis();
		}
		tb.sendTelemetryData(LED_MODE_ATTR, ledMode);
		tb.sendTelemetryData(LED_STATE_ATTR, ledState);
		tb.sendAttributeData(LED_MODE_ATTR, ledMode);
		tb.sendAttributeData(LED_STATE_ATTR, ledState);
	}
	
	if (ledMode == 1 && millis() - previousStateChange > blinkingInterval) {
		previousStateChange = millis();
		ledState = 1;
		tb.sendTelemetryData(LED_STATE_ATTR, ledState);
		tb.sendAttributeData(LED_STATE_ATTR, ledState);
		if (LED_BUILTIN == 99) {
			Serial.print("LED state changed to: ");
			Serial.println(ledState);
		} else {
			digitalWrite(LED_BUILTIN, ledState);
		}
	}
	
	// Sending telemetry every telemetrySendInterval time
	if (millis() - previousDataSend > telemetrySendInterval) {
		previousDataSend = millis();
		
		float temp = 0.0; 
		
		
		Sns_BMP280.store_ctrl_meas( snsd::BMP280::CtrlMeas_TemperatureSampling_1x | snsd::BMP280::CtrlMeas_PressureSampling_1x | snsd::BMP280::CtrlMeas_Power_OneShot )
		;
		vTaskDelay( pdMS_TO_TICKS( 10 ) );
		Sns_BMP280.load_data( &temp, nullptr );
		Sns_BMP280.store_ctrl_meas( snsd::BMP280::CtrlMeas_Power_Low );
		tb.sendTelemetryData("temperature", temp);
		tb.sendAttributeData("rssi", WiFi.RSSI());
		tb.sendAttributeData("channel", WiFi.channel());
		tb.sendAttributeData("bssid", WiFi.BSSIDstr().c_str());
		tb.sendAttributeData("localIp", WiFi.localIP().toString().c_str());
		tb.sendAttributeData("ssid", WiFi.SSID().c_str());
	}
	
	tb.loop();
}

struct _inet_t : public Service {
protected:
	virtual status_t _service_start( void* ctx_ ) override {
		ESP_LOGI( Tag, "service: inet: starting..." );

		Psettings.begin( "wifi_creds", true );
		auto ssid = Psettings.getString( "ssid", "" );
		auto pwrd = Psettings.getString( "pwrd", "" );
		Psettings.end();

		A113_ASSERT_OR( not ssid.isEmpty() and not pwrd.isEmpty() ) {
			ESP_LOGE( Tag, "service: inet: wifi credentials not set." );
			//return A113_ERR_LOGIC;
		}

		WiFi.begin( WIFI_SSID, WIFI_PASSWORD );
		while( WiFi.status() != WL_CONNECTED ) {
			vTaskDelay( 1000_pdms2t );
		}
		ESP_LOGI( Tag, "service: inet: connected to wifi: %s.", ssid.c_str() );

		ESP_LOGI( Tag, "service: inet: started." );
		return A113_OK;
	}

	virtual status_t _service_stop( void* ctx_ ) override {
		return A113_OK;
	}

} INet;

struct _thingsboard_t {
public:
	status_t start( void ) {

	}
	
} Thingsboard;

extern "C" void app_main( void ) {
	vTaskDelay( 1000_pdms2t );

	initArduino();

	i2c_new_master_bus( &I2C_BUS_0_CONFIG, &I2C_Bus0 );
	I2C_AHT21.bind( I2C_Bus0, I2C_DEV_AHT21_CONFIG, 50 );
	I2C_BMP280.bind( I2C_Bus0, I2C_DEV_BMP280_CONFIG, 50 );
	
	Sns_AHT21.bind_i2c( &I2C_AHT21 ); 
	Sns_AHT21.calib();
	
	Sns_BMP280.bind_i2c( &I2C_BMP280 );
	Sns_BMP280.load_calibs();

	INet.service_start( nullptr );
}

