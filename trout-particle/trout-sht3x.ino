// A trout is an animal that's very sensitive to changes in motion and temperature.
//
// This firmware combines a motion sensor and a temperature sensor

#include "MQTT.h"

/////////////////////////////////////////////////////////////////////////////////////////////
// MQTT
/////////////////////////////////////////////////////////////////////////////////////////////

char myIpString[24];
byte server[] = { 10, 5, 23, 34 };

MQTT mqttClient(server, 1883, mqttCallback);
int mqtt_status = 0;

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    char p[length + 1];
    memcpy(p, payload, length);
    p[length] = '\0';
    String message(p);

    //Do something with String message?
}

bool setupMqtt() {
    Particle.variable("mqttstatus", mqtt_status);

    // connect to the server
    if (mqttClient.connect("trout")) {
        return true;
    }
    return false;
}

void loopMqtt() {
    mqtt_status = mqttClient.isConnected() ? 1 : 0;

    if (!mqttClient.loop()) {
        // Not connected, try to reconnect
        if (!setupMqtt()) {
            // Reconnect failed, wait a few seconds
            delay(5000);
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////
// MOTION:
/////////////////////////////////////////////////////////////////////////////////////////////
//
// Monitor the "motion" variable, where "1" = motion within the last {TTL} seconds, "0" = no motion within the last {TTL} seconds
// curl -H "Authorization: Bearer <token>" https://api.spark.io/v1/devices/<deviceid>/motion
//
// Publishes an event (and blinks D7) whenever a motion sensor (hooked up to A0) senses motion.
// Check for "motion" events on this Server Sent Events (SSE) stream. "data" will match up with the value of the "motion" variable
// curl -H "Authorization: Bearer <token>" https://api.spark.io/v1/devices/<deviceid>/events/
// curl -H "Authorization: Bearer <token>" https://api.spark.io/v1/devices/events

#define TTL 5
#define LED D7
#define MOTION_SENSOR A1

#define MOTION_DEBOUNCE_SECONDS 1

int state = LOW;
int reset_time = 0;
int motion_debounce_time = 0;
int motion = 0;

void setupMotion() {
    pinMode(LED, OUTPUT);
    pinMode(MOTION_SENSOR, INPUT);
    digitalWrite(LED, state);
    Particle.variable("motion", motion);
}

void loopMotion() {
	int now = Time.now();

	if (now > motion_debounce_time) {
		int newstate = digitalRead(MOTION_SENSOR);
		if (newstate != state) {
			if (newstate == HIGH) {
				reset_time = now + TTL;
				if (motion == 0) {
					Particle.publish("motion", "active", TTL, PRIVATE);
					mqttClient.publish("devices/trout/motion", "active");
					motion = 1;
				}
				digitalWrite(LED, HIGH);
			}

			state = newstate;
			motion_debounce_time = now + MOTION_DEBOUNCE_SECONDS;
		}
	}

    if (now > reset_time) {
        if (motion == 1) {
            Particle.publish("motion", "inactive", TTL, PRIVATE);
			mqttClient.publish("devices/trout/motion", "inactive");
            motion = 0;
        }
        digitalWrite(LED, LOW);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////
// TEMPERATURE:
/////////////////////////////////////////////////////////////////////////////////////////////
//
// Tests a SHT3x temperature/humidity sensor connected to I2C on pins D0/D1
// curl -H "Authorization: Bearer <token>" https://api.spark.io/v1/devices/<deviceid>/temperature
// curl -H "Authorization: Bearer <token>" https://api.spark.io/v1/devices/<deviceid>/humidity

#include "adafruit-sht31.h"

Adafruit_SHT31 sht31 = Adafruit_SHT31();

#define SHT_DEBOUNCE_SECONDS 60

char temperature[16];
char humidity[16];

int temperature10 = 0;
int humidity10 = 0;
int temperature_debounce_time = 0;
int humidity_debounce_time = 0;

void setupTemperature() {
    Particle.variable("temperature", temperature);
    Particle.variable("humidity", humidity);
    sht31.begin(0x44);
}

// Rounds to the nearest tenth and stringifies a float into *buffer
void round10(char* buffer, int size, float value) {
    int v = (int) round(value * 10);
    char *p = buffer + size;
    *--p = '\0';
    *--p = '0' + (v % 10);
    *--p = '.';
    do {
        v /= 10;
        *--p = '0' + (v % 10);
    } while (v >= 10 && p > buffer);

    while (*p != 0) {
        *buffer++ = *p++;
    }
}

void loopTemperature() {
    int now = Time.now();

    if (now > temperature_debounce_time) {
        float tC = sht31.readTemperature();
        float tF = (tC * 9) / 5 + 32;

        if (!isnan(tF)) {
            int tf10 = (int) round(tF * 10);
            if (temperature10 != tf10) {
                temperature10 = tf10;
                round10(temperature, 16, tF);
                Particle.publish("temperature", temperature, 0, PRIVATE);
                mqttClient.publish("devices/trout/temperature", temperature);
                temperature_debounce_time = now + SHT_DEBOUNCE_SECONDS;
            }
        }
    }

    if (now > humidity_debounce_time) {
        float h = sht31.readHumidity();

        if (!isnan(h)) {
            int h10 = (int) round(h * 10);
            if (humidity10 != h10) {
                humidity10 = h10;
                round10(humidity, 16, h);
                Particle.publish("humidity", humidity, 0, PRIVATE);
                mqttClient.publish("devices/trout/humidity", humidity);
                humidity_debounce_time = now + SHT_DEBOUNCE_SECONDS;
            }
        }
    }

}

/////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
	setupMqtt();
    setupMotion();
    setupTemperature();
}

void loop() {
	loopMqtt();
    loopMotion();
    loopTemperature();
}
