#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include "nvs_flash.h"
#include <esp_http_server.h>
#include <esp_vfs_fat.h>
#include <math.h>
#include "i2c_funtions.h"
#include "ssd1306.h"
#include "ultrasonic.h"
#include "esp_http_client.h"


#include <bme680.h>

#define FILE_PATH "../index.html"

esp_err_t post_led_toggle ( httpd_req_t * req );
esp_err_t get_root_handler (httpd_req_t * req);
esp_err_t get_sensor_value ( httpd_req_t * req );
esp_err_t get_burglary_value ( httpd_req_t * req );
esp_err_t reset_burglary ( httpd_req_t * req );
httpd_handle_t start_webserver ( void);

#define ESP_WIFI_SSID ""
#define ESP_WIFI_PASS ""

bool wifi_established;
static const char *TAG = "wifi station";
#define GPIO_LED 5 // port pin of on - board LED

#define SDA 21
#define SCL 22
#define I2C_MASTER_PORT 0
#define ADDR BME680_I2C_ADDR_0

// Farben der Wetterdateneinträge
#define GREEN "#9bd317"
#define light_green "#b2ff8e"
#define ORANGE "#a74c25"


char * html_response;
bme680_values_float_t data;
double tsl_lux_values;
uint8_t burglary_happenning = 0u;

// HTTP Responses der api.callmebot api auswerten
esp_err_t http_event_handler (esp_http_client_event_t *evt) {
    switch (evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGI ( TAG , " HTTP_EVENT_ERROR " ) ;
            break ;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGI ( TAG , "HTTP EVENT_ON_CONNECTED" ) ;
            break ;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGI(TAG, "HTTP EVENT HEADER SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGI (TAG, "HTTP_EVENT_ON HEADER");
            printf ( "%.*s" , evt->data_len , ( char *) evt->data ) ;
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGI (TAG, "HTTP EVENT ON DATA, len=%d", evt->data_len);
            if (!esp_http_client_is_chunked_response(evt->client)) {
                printf("sending whatsapp msg was successfull");
            }
            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGI (TAG, "HTTP_EVENT_ON_FINISH");
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
            break;
    }
    return ESP_OK;
}

// HTTP Client initialisieren und sofort Nachricht schicken -> danach Client Struktur wieder freigeben
// nur einmal initialisiert da Client möglicherweise nicht benötigt wenn Einbruchüberwachung nicht nötig
static void init_client( char * msg) {
    char * url_buffer[140];
    snprintf(url_buffer, sizeof(url_buffer), "http://api.callmebot.com/whatsapp.php?phone=491782112365&text=%s&apikey=8805349", msg);
    esp_http_client_config_t cfg = { // CLient Konfiguration 
        .url = url_buffer,
        .event_handler = http_event_handler, // obiger Event Handler
    };
    esp_http_client_handle_t client; // Handle des Clients
    client = esp_http_client_init(&cfg); // Initialisierung des Clients
    esp_err_t err = esp_http_client_perform(client); // Aufbau der Verbindung
    if ( err == ESP_OK ) {
        // Status Code bei erfolgreicher Kommunikation mit Client ausgeben
        ESP_LOGI(TAG,"Status = %d length = %d", esp_http_client_get_status_code (client), esp_http_client_get_content_length(client)) ; 
    }
    // Speicherplatz der Clientstruktur freigeben
    esp_http_client_cleanup(client); 
}



void bme680_task(void *pvParamters)
{
    bme680_t sensor; // Sensorinitialisierungsdatenstruktur
    memset(&sensor, 0, sizeof(bme680_t)); // Struktur komplett auf 0 setzen

    sensor.i2c_dev.timeout_ticks=1000; // I2C Bus timeout
    
    //Sensorinitialisierungsparameter festlegen (wir nutzen I2C)
    ESP_ERROR_CHECK(bme680_init_desc(&sensor, BME680_I2C_ADDR_0, I2C_MASTER_PORT, SDA, SCL));

    // Sensor mit oben festgelegten Parametern initialisieren
    ESP_ERROR_CHECK(bme680_init_sensor(&sensor));

    // festlegen um wie viel abgetastet werden soll (im vergleich zur normalen Datenrate)
    bme680_set_oversampling_rates(&sensor, BME680_OSR_2X, BME680_OSR_4X, BME680_OSR_8X);


    bme680_set_filter_size(&sensor, BME680_IIR_SIZE_3);

    // Gaswiederstandsmessung aktivieren
    bme680_use_heater_profile(&sensor, 0);

    // Temp in Wiederstandsmessung einbeziehen -> fehlerhaft wenn Temp. messung fehlerhaft
    bme680_set_ambient_temperature(&sensor, data.temperature);

    
    uint32_t duration;
    // geschätzte Messdauer ermitteln
    bme680_get_measurement_duration(&sensor, &duration);

    while (1) {
        // Messen
        if (bme680_force_measurement(&sensor) == ESP_OK) {
            vTaskDelay(duration); // warten bis gemessen wurde
            if (bme680_get_results_float(&sensor, &data) != ESP_OK) { // Resultate setzen
                printf("bme sensor meassuring not successful");
            }
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

// Lux Wert Berechnung aus tsl Kanälen vornehmen
// CH0 sensitiv für Infrarot- und sichtbaren Licht
// CH1 misst nur Infrarot Wert -> Korrektur da erste Photodiode ansonsten Infrarot zu stark gewichtet
// Formel ist Dokumentation entnommen
float calc_lux(uint16_t CH0, uint16_t CH1) {
    float divident = 0.0;
    if (CH0 !=0) {
        divident = CH1/CH0;
    } else {
        return 0;
    } 
    double lux = 0.0;
    if (divident <= 0.5) {
        lux = 0.0304 * CH0 - 0.062 * CH0 * powf((CH1/CH0), 1.4);
    } else if (divident <= 0.61) {
        lux = 0.0224 * CH0 - 0.031 * CH1;
    } else if (divident <= 0.80) {
        lux = 0.0128 * CH0 - 0.0153 * CH1;
    } else if (divident <= 1.30) {
        lux = 0.00146 * CH0 - 0.00112 * CH1;
    } else {
        lux = 0.0;
    }
    return lux;
}

uint8_t lux_read_sensor_id() {
    uint8_t tsl_i2c_address = 0b00101001;
    uint8_t reg_addr = 0b11001010; // ID Register
    return i2c_register_read(I2C_MASTER_PORT, tsl_i2c_address, reg_addr);
}

void tsl_task(void *pvParamters) {
    uint8_t tls_adc_data[4] = {0};
    uint8_t tsl_i2c_address = 0b00101001; 
    uint8_t control_reg_cmds = 0b11000000; 
    uint8_t adc_channel_cmds = 0b11011100;
    // Sensor anschalten
    if (i2c_register_write(I2C_MASTER_PORT, tsl_i2c_address, control_reg_cmds, (uint8_t)0x03) != ESP_OK){
            printf("Setting Control Byte was not succesfull\n");
    } 
    // Sensor ID zum prüfen ausgeben
    printf("Tsl ID: %X\n", (uint16_t)lux_read_sensor_id());
    while(1) {
        // adc Daten lesen
        i2c_multi_register_read(I2C_MASTER_PORT, tsl_i2c_address, adc_channel_cmds, tls_adc_data, 2);
        i2c_multi_register_read(I2C_MASTER_PORT, tsl_i2c_address, (adc_channel_cmds+2), (tls_adc_data+2), 2);
        printf("Channel values: %08X\n", (uint32_t)*((uint32_t*)tls_adc_data));
        printf("Channel output values CH0: %d CH1: %d \n", (uint16_t) *(uint16_t *)tls_adc_data, (uint16_t)*(uint16_t *)(tls_adc_data+2));
        tsl_lux_values = calc_lux((uint16_t) *(uint16_t *)tls_adc_data, (uint16_t)*(uint16_t *)(tls_adc_data+2));
        printf("The light intensity is at: %.2f lux\n", tsl_lux_values);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    
}
// Distanz messen
void ultrasonic_task(void *pvParamters) {
    // Initialisierungsparam. des Bewegungsmelders festlegen
    ultrasonic_sensor_t dev = {
        .trigger_pin = GPIO_NUM_18,
        .echo_pin = GPIO_NUM_18
    };
    
    while (1) {
        ultrasonic_init(&dev); 
        uint32_t distance = 0u;
        esp_err_t res = ESP_OK;
        if ((res = (ultrasonic_measure_cm(&dev, (uint32_t)250, &distance)) != ESP_OK)) {
            switch (res)
            {
                case ESP_ERR_ULTRASONIC_PING:
                    printf("Cannot ping\n");
                    break;
                case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
                    printf("Ping timeout (no device found)\n");
                    break;
                case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
                    // kann ignoriert werden wenn Distanz zur Wand größer als 2,5m
                    printf("Echo timeout (distance to big)\n");
                    break;
                default:
                    printf("%s\n", esp_err_to_name(res));
            }
        };
        // wenn Distanz zu Gegenstand zu groß -> Einbruch :)
        if (distance < 150u && !burglary_happenning) {
            burglary_happenning = 1; 
            if (wifi_established) { 
                init_client("Einbruch!!!");//whatsapp Nachricht senden
            }
        }
        // alle 150 msec messen
        printf("Measured Distance: %u\n", distance);
        vTaskDelay(150 / portTICK_PERIOD_MS);
    }
}



// Wifi Event Handler
static void event_handler(void* handler_arg, esp_event_base_t base, int32_t id, void* event_data)
{
    if ( base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        wifi_established = false;
        ESP_LOGI(TAG, "connect to the Access Point");
        esp_wifi_connect();
    } else if ( base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_established = false;
        ESP_LOGI(TAG, "retry connect to the Access Point");
        esp_wifi_connect ();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t* ) event_data;
        ESP_LOGI(TAG, "got ip: %s ", ip4addr_ntoa(((in_addr_t*)&(event->ip_info.ip))));
        start_webserver();
        wifi_established = true;
    } else {
        ESP_LOGI(TAG, "unhandled event (%s) with ID %d!" , base , id);
    }
    // Event handler logic
}

// Initialisierung des Wifis
static void wifi_init() {
    tcpip_adapter_init(); // TCP/IP Stack initialisieren
    ESP_ERROR_CHECK(esp_event_loop_create_default()); // default Event Loop kreiren
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT(); // Default Konfiguration erstellen 
    cfg.nvs_enable = false;
    ESP_ERROR_CHECK (esp_wifi_init(&cfg)); // Default Konfiguration übergeben
    // Event Handler registrieren
    ESP_ERROR_CHECK( esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, event_handler, NULL)); 
    ESP_ERROR_CHECK( esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, event_handler, NULL));
    ESP_ERROR_CHECK( esp_event_handler_register(IP_EVENT, IP_EVENT_STA_LOST_IP, event_handler, NULL));
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA)); // Station Modus setzen
    wifi_config_t wifi_config = { // SSID und Password festlegen
        .sta = {
        .ssid = ESP_WIFI_SSID ,
        .password = ESP_WIFI_PASS ,
        } ,
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA ,&wifi_config)); // Konfiguration setzen
    ESP_ERROR_CHECK ( esp_wifi_start ()) ; // Verbindung aufbauen
    ESP_LOGI(TAG, "wifi_init_sta finished.");
}

// Farbe des Luftfeuchtigkeitswert in html Tabelle ermitteln 
char * evaluate_humidity() {
    if (data.humidity < 30.0 || data.humidity > 70.0) {
        if (wifi_established) { 
                char message[60]= {0};
                snprintf(message, sizeof(message),"Fenster aufmachen!!! %.2f  %% Luftfeuchtigkeit", data.humidity);
                init_client(message);
        }
        
        return ORANGE;
    } else if (data.humidity > 40.0 || data.humidity < 60.0) {
        return GREEN;
    } else {
        return light_green;
    }
} 

/* URI handler structure for GET / uri */
httpd_uri_t uri_get_root = {
    .uri = "/" ,
    .method = HTTP_GET ,
    .handler = get_root_handler ,
    .user_ctx = NULL
};

/* URI handler structure for GET / uri */
httpd_uri_t uri_led_toggle = {
    .uri = "/onbut" ,
    .method = HTTP_GET ,
    .handler = post_led_toggle ,
    .user_ctx = NULL
};

/* URI handler structure for GET / uri */
httpd_uri_t uri_sensor_values = {
    .uri = "/sensor_val" ,
    .method = HTTP_GET ,
    .handler = get_sensor_value ,
    .user_ctx = NULL
};


/* URI handler structure for GET / uri */
httpd_uri_t uri_burglary = {
    .uri = "/check_burglary" ,
    .method = HTTP_GET ,
    .handler = get_burglary_value ,
    .user_ctx = NULL
};

/* URI handler structure for GET / uri */
httpd_uri_t uri_reset_burglary = {
    .uri = "/reset_burglary" ,
    .method = HTTP_GET ,
    .handler = reset_burglary ,
    .user_ctx = NULL
};

esp_err_t get_burglary_value ( httpd_req_t * req ) {
    if (burglary_happenning) {
        httpd_resp_send ( req, "1", HTTPD_RESP_USE_STRLEN);
    } else {
        httpd_resp_send ( req, "0", HTTPD_RESP_USE_STRLEN);
    }
    return ESP_OK;
}

esp_err_t reset_burglary ( httpd_req_t * req ) {
    burglary_happenning = 0;
    httpd_resp_send ( req, "burglary happend value was reset", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t get_root_handler ( httpd_req_t * req ) {
    char* html_end_response= (char *)malloc(4000);
    memset(html_end_response,0,4000);
    
    snprintf(html_end_response, 4000,"<!DOCTYPE html><html><head><title>ESP32WetterStation</title><style>body{background-color:#86b9e0;color:white;width:95%%;margin:auto;}h1{text-align:center;margin-bottom:40px;}p{display:inline-block;} .key{font-weight:bolder;padding-right:4px;}table{margin:auto;}.value{padding:7px 7px 7px 9px;width:200px;margin:0;background-color:#9bd317;border-radius:10px;}td{padding-right:30px;padding-bottom:10px;} .main-content{font-size:18px;} .button-container{display:flex;justify-content:center;margin-top:20px;}button{background-color:#cb9be8;margin-right:10px;padding:10px;border-radius:15px;border:1px solid transparent;}</style></head><body>\
        <script type=\"text/javascript\" src=\"//ajax.googleapis.com/ajax/libs/jquery/1.9.1/jquery.js\"></script>\
<script>\n\
  function loadLatestResults() {\n\
    $.ajax({\n\
      url: './sensor_val',\n\
      cache: false,\n\
      success: function(data) {\n\
        const sensor_values = data.split(',');\
        $('#temperature').html(sensor_values[0]);\n\
        $('#air-pressure').html(sensor_values[1]);\n\
        $('#humidity').html(sensor_values[2]);\n\
        $('#light-intensity').html(sensor_values[3]);\n\
        $('#gas_resistance').html(sensor_values[4]);\n\
      }\n\
    });\n\
    $.ajax({\n\
      url: './check_burglary',\n\
      cache: false,\n\
      success: function(data) {\n\
        console.log(data);\
      }\n\
    });\n\
  }\n\
  \n\
  $(document).ready(function() {\n\
    window.setInterval(loadLatestResults, 5000);\n\
  });\n//\
    </script><h1>Die ESP-WetterStation</h1><div class=\"main-content\"><div><table><tr><td class=\"key\">Temperatur:</td><td><p class=\"value\" style=\"background-color:%s\" id=\"temperature\">%.2f&#176;C</p></td><td class=\"key\">Luftdruck:</td><td><p style=\"background-color:%s\" class=\"value\" id=\"air-pressure\">%.2fPa</p></td></tr><tr><td class=\"key\">Luftfeuchtigkeit:</td><td><p class=\"value\" style=\"background-color:%s\" id=\"humidity\">%.2f%%</p></td><td class=\"key\">Lichtintensit&auml;t:</td><td><p style=\"background-color:%s\" class=\"value\" id=\"light-intensity\">%.2f Lux</p></td></tr>\
    <tr><td class=\"key\">Gaswiederstand:</td><td><p class=\"value\" style=\"background-color:#9bd317\" id=\"gas_resistance\">%.2fOhm</p></td></tr></table>\
    </div><div class=\"button-container\"><a href=\"/onbut\"><button>LED an</button></a><a href=\"/onrel\"><button>Relais ON</button><a href=\"/reset_burglary\"><button>Einbruchsüberwachung zurücksetzen</button></a></a></div></div>\
    </body></html>", GREEN,
                     data.temperature, GREEN, data.pressure, evaluate_humidity(), data.humidity, GREEN, tsl_lux_values, data.gas_resistance);
    httpd_resp_send ( req, html_end_response, HTTPD_RESP_USE_STRLEN) ;
    free(html_end_response);
    return ESP_OK;
}

int led_is_on = 0;

esp_err_t post_led_toggle ( httpd_req_t * req ) {
    httpd_resp_set_status(req, HTTPD_204);
    if (led_is_on) {
        led_is_on = 0;
    } else {
        led_is_on = 1;
    }
    gpio_set_level ( GPIO_LED , led_is_on);
    return ESP_OK;
}


esp_err_t get_sensor_value ( httpd_req_t * req ) {
    char *sensor_values_resp = "%.2f&#176;C,%.2fPa,%.2f%%,%.2fLux,%.2fOhm";
    char *values_resp = (char*)malloc(120);
    snprintf(values_resp, 120,sensor_values_resp, data.temperature, data.pressure, data.humidity, lux_read_sensor_id, data.gas_resistance);
    httpd_resp_send ( req, values_resp, HTTPD_RESP_USE_STRLEN) ;
    return ESP_OK;
}

httpd_handle_t start_webserver ( void ) {
    /* Server Configuration auf default Config setzen */
    httpd_config_t config = HTTPD_DEFAULT_CONFIG () ;
    /* Leeres handle esp_http_server */
    httpd_handle_t server = NULL ;
    /* Start des httpd servers mit gegebenen Ressourcen */
    if ( httpd_start (&server , &config ) == ESP_OK ) {
        /* Register URI handlers */
        httpd_register_uri_handler(server,&uri_get_root);
        httpd_register_uri_handler(server,&uri_led_toggle);
        httpd_register_uri_handler(server,&uri_sensor_values);
        httpd_register_uri_handler(server,&uri_burglary);
        httpd_register_uri_handler(server,&uri_reset_burglary);
    }
    return server;
}

/* Function for stopping the webserver */
void stop_webserver(httpd_handle_t server)
{
    if (server) {
        /* Stop the httpd server */
        httpd_stop(server);
    }
}




void app_main() {
    nvs_flash_init();
    html_response= (char*)malloc(3000);
    memset(html_response, 0, 3000);
    
    
      
    data = (bme680_values_float_t){0};
    tsl_lux_values = 0.0;
   
    gpio_set_direction ( GPIO_LED , GPIO_MODE_OUTPUT ); // set GPIO of led pint as output
    wifi_init();
    xTaskCreatePinnedToCore(ultrasonic_task, "ultrasonic_task", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
    ESP_ERROR_CHECK(i2cdev_init());
    vTaskDelay(2000/portTICK_PERIOD_MS);
    xTaskCreatePinnedToCore(bme680_task, "bme680_task", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
    vTaskDelay(2000/portTICK_PERIOD_MS);
    init_i2c();
    xTaskCreatePinnedToCore(tsl_task, "tsl_task", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
    
}