#include <WiFi.h>
#include <PubSubClient.h>
#include <esp_camera.h>
#include <driver/i2s.h>

// WiFi credentials
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// MQTT Broker settings
const char* mqtt_server = "YOUR_MQTT_SERVER_IP";
const int mqtt_port = 1883;
const char* mqtt_topic_video = "esp32/camera";
const char* mqtt_topic_audio = "esp32/audio";

// WiFi client
WiFiClient espClient;
PubSubClient client(espClient);

// Camera pins for XIAO ESP32 S3 Sense
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     10
#define SIOD_GPIO_NUM     40
#define SIOC_GPIO_NUM     39
#define Y9_GPIO_NUM       48
#define Y8_GPIO_NUM       46
#define Y7_GPIO_NUM       9
#define Y6_GPIO_NUM       8
#define Y5_GPIO_NUM       7
#define Y4_GPIO_NUM       6
#define Y3_GPIO_NUM       5
#define Y2_GPIO_NUM       4
#define VSYNC_GPIO_NUM    47
#define HREF_GPIO_NUM     45
#define PCLK_GPIO_NUM     21

// Audio settings
#define I2S_WS 42
#define I2S_SD 41
#define I2S_SCK 43
#define I2S_PORT I2S_NUM_0
#define I2S_SAMPLE_RATE   16000
#define I2S_SAMPLE_BITS   16
#define I2S_READ_LEN      16000

// Buffer for audio samples
int16_t audio_buffer[I2S_READ_LEN];

void setup_wifi() {
    delay(10);
    Serial.println("Connecting to WiFi...");
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

void setup_camera() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.frame_size = FRAMESIZE_VGA;
    config.pixel_format = PIXFORMAT_JPEG;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.jpeg_quality = 12;
    config.fb_count = 1;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x", err);
        return;
    }
}

void setup_i2s() {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = I2S_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 1024,
        .use_apll = false,
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SCK,
        .ws_io_num = I2S_WS,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_SD
    };

    esp_err_t err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    if (err != ESP_OK) {
        Serial.printf("Failed installing I2S driver: %d\n", err);
        return;
    }

    err = i2s_set_pin(I2S_PORT, &pin_config);
    if (err != ESP_OK) {
        Serial.printf("Failed setting I2S pins: %d\n", err);
        return;
    }
}

void reconnect() {
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (client.connect("ESP32Client")) {
            Serial.println("connected");
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" retrying in 5 seconds");
            delay(5000);
        }
    }
}

void setup() {
    Serial.begin(115200);
    
    // Initialize WiFi
    setup_wifi();
    
    // Initialize MQTT client
    client.setServer(mqtt_server, mqtt_port);
    
    // Initialize camera
    setup_camera();
    
    // Initialize I2S for microphone
    setup_i2s();
}

void loop() {
    if (!client.connected()) {
        reconnect();
    }
    client.loop();

    // Capture and send camera frame
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) {
        client.beginPublish(mqtt_topic_video, fb->len, false);
        client.write(fb->buf, fb->len);
        client.endPublish();
        esp_camera_fb_return(fb);
    }

    // Capture and send audio
    size_t bytes_read = 0;
    i2s_read(I2S_PORT, audio_buffer, sizeof(audio_buffer), &bytes_read, portMAX_DELAY);
    if (bytes_read > 0) {
        client.beginPublish(mqtt_topic_audio, bytes_read, false);
        client.write((const uint8_t*)audio_buffer, bytes_read);
        client.endPublish();
    }

    // Small delay to prevent overwhelming the network
    delay(100);
}