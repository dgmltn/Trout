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
    p[length] = NULL;
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

int LED = D0;
int MOTION_SENSOR = A0;
int state = LOW;
int reset_time = 0;
int debounce_time = 0;
int motion = 0;

void setupMotion() {
    pinMode(LED, OUTPUT);
    pinMode(MOTION_SENSOR, INPUT);
    digitalWrite(LED, state);
    Particle.variable("motion", &motion, INT);
}

void loopMotion() {
	int now = Time.now();

	if (now > debounce_time) {
		int newstate = digitalRead(MOTION_SENSOR);
		if (newstate != state) {
			if (newstate == HIGH) {
				reset_time = now + TTL;
				if (motion == 0) {
					Particle.publish("motion", "1", TTL, PRIVATE);
					mqttClient.publish("devices/trout/motion", "1");
					motion = 1;
				}
				digitalWrite(LED, HIGH);
			}

			state = newstate;
			debounce_time = now + 1;
		}
	}

    if (now > reset_time) {
        if (motion == 1) {
            Spark.publish("motion", "0", TTL, PRIVATE);
			mqttClient.publish("devices/trout/motion", "0");
            motion = 0;
        }
        digitalWrite(LED, LOW);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////
// TEMPERATURE:
/////////////////////////////////////////////////////////////////////////////////////////////
//
// Tests a TMP36 temperature sensor connected to A4
// curl -H "Authorization: Bearer <token>" https://api.spark.io/v1/devices/<deviceid>/tempc
// curl -H "Authorization: Bearer <token>" https://api.spark.io/v1/devices/<deviceid>/tempf

int TMP36_SENSOR = A4;
double tempc = 0.0;
double tempf = 0.0;
int tempraw = 0;
int temp_debounce_time = 0;

void setupTemperature() {
    Particle.variable("tempraw", &tempraw, INT);
    Particle.variable("tempc", &tempc, DOUBLE);
    Particle.variable("tempf", &tempf, DOUBLE);
    pinMode(TMP36_SENSOR, INPUT);
}

void loopTemperature() {
	int now = Time.now();
	if (now > temp_debounce_time) {
		tempraw = analogRead(TMP36_SENSOR);
		// The returned value from the Core is going to be in the range from 0 to 4095
		// Calculate the voltage from the sensor reading
		double c = (((tempraw * 3.3) / 4095) - 0.5) * 100;
		// Convert C -> F
		double f = (c * 9 / 5) + 32;

		// Round to 1 decimal place
		tempc = round(c*10)/10.0;
		tempf = round(f*10)/10.0;

		temp_debounce_time = now + 5;
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
