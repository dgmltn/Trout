// A trout is an animal that's very sensitive to changes in motion and temperature.
//
// This firmware combines a motion sensor and a temperature sensor

/////////////////////////////////////////////////////////////////////////////////////////////
// MOTION:
//
// Monitor the "motion" variable, where "1" = motion within the last {TTL} seconds, "0" = no motion within the last {TTL} seconds
// curl -H "Authorization: Bearer <token>" https://api.spark.io/v1/devices/<deviceid>/motion
//
// Publishes an event (and blinks D7) whenever a motion sensor (hooked up to A0) senses motion.
// Check for "motion" events on this Server Sent Events (SSE) stream. "data" will match up with the value of the "motion" variable
// curl -H "Authorization: Bearer <token>" https://api.spark.io/v1/devices/<deviceid>/events/
// curl -H "Authorization: Bearer <token>" https://api.spark.io/v1/devices/events

int LED = D0;
int MOTION_SENSOR = A0;
int state = LOW;
int reset_time = 0;
int TTL = 5;
int motion = 0;

void setupMotion() {
    pinMode(LED, OUTPUT);
    pinMode(MOTION_SENSOR, INPUT);
    digitalWrite(LED, state);
    Particle.variable("motion", &motion, INT);
}

void loopMotion() {
    int newstate = digitalRead(MOTION_SENSOR);
    if (newstate != state) {
        if (newstate == HIGH) {
            reset_time = Time.now() + TTL;
            if (motion == 0) {
                Particle.publish("motion", "1", TTL, PRIVATE);
                motion = 1;
            }
            digitalWrite(LED, HIGH);
        }

        state = newstate;
        delay(500); // Debounce
    }
    
    if (Time.now() > reset_time) {
        if (motion == 1) {
            Spark.publish("motion", "0", TTL, PRIVATE);
            motion = 0;
        }
        digitalWrite(LED, LOW);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////
// TEMPERATURE:
// Tests a TMP36 temperature sensor connected to A4
// curl -H "Authorization: Bearer <token>" https://api.spark.io/v1/devices/<deviceid>/tempc
// curl -H "Authorization: Bearer <token>" https://api.spark.io/v1/devices/<deviceid>/tempf

int TMP36_SENSOR = A4;
double tempc = 0.0;
double tempf = 0.0;
int tempRaw = 0;

void setupTemperature() {
    Particle.variable("tempraw", &tempRaw, INT);
    Particle.variable("tempc", &tempc, DOUBLE);
    Particle.variable("tempf", &tempf, DOUBLE);
    pinMode(TMP36_SENSOR, INPUT);
}

void loopTemperature() {
    tempRaw = analogRead(TMP36_SENSOR);
    // The returned value from the Core is going to be in the range from 0 to 4095
    // Calculate the voltage from the sensor reading
    double c = (((tempRaw * 3.3) / 4095) - 0.5) * 100;
    // Convert C -> F
    double f = (c * 9 / 5) + 32;
    
    // Round to 1 decimal place
    tempc = round(c*10)/10.0;
    tempf = round(f*10)/10.0;
}

/////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
    setupMotion();
    setupTemperature();
}

void loop() {
    loopMotion();
    loopTemperature();
}

