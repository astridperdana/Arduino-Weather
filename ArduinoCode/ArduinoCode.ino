#include "DHT.h"
#include "MQ135.h"
#include <SPI.h>
#include <Ethernet.h>
#include <ArduinoJson.h>

// <============> START OF ETHERNET GLOBAL VARIABLE
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(10, 126, 11, 207);
EthernetServer server(80);
// <============> END OF ETHERNET GLOBAL VARIABLE





/* Photocell simple testing sketch.

Connect one end of the photocell to 5V, the other end to Analog 0.
Then connect one end of a 10K resistor from Analog 0 to ground
Connect LED from pin 11 through a resistor to ground
For more information see http://learn.adafruit.com/photocells */

/* Vibration sensor connected to Arduino pins as follows:
Arduino            Vibration Sensor
D9                    DOut
GND                   GND
+5V                   VCC

D13                Indication LED
*/

#define DHTPIN 4     // what digital pin we're connected to

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

// Initialize DHT sensor.
// Note that older versions of this library took an optional third parameter to
// tweak the timings for faster processors.  This parameter is no longer needed
// as the current DHT reading algorithm adjusts itself to work on faster procs.
DHT dht(DHTPIN, DHTTYPE);

// <============> START OF WATERFLOW GLOBAL VARIABLE
	byte statusLed = 13;
	byte sensorInterrupt = 0;  // 0 = digital pin 2
	byte sensorPin = 2;
	// The hall-effect flow sensor outputs approximately 4.5 pulses per second per
	// litre/minute of flow.
	float calibrationFactor = 4.5;
	volatile byte pulseCount;
	float flowRate;
	unsigned int flowMilliLitres;
	unsigned long totalMilliLitres;
	unsigned long oldTime;
// <============> END OF WATERFLOW GLOBAL VARIABLE

// <============> START OF WINDFLOW GLOBAL VARIABLE
	// Pin definitions
	# define windPin 3 // Receive the data from sensor

	// Constants definitions
	const float pi = 3.14159265; // pi number
	int period = 10000; // Measurement period (miliseconds)
	int delaytime = 10000; // Time between samples (miliseconds)
	int radio = 80; // Distance from center windmill to outer cup (mm)
	int jml_celah = 22; // jumlah celah sensor

						// Variable definitions
	unsigned int Sample = 0; // Sample number
	unsigned int counter = 0; // B/W counter for sensor
	unsigned int RPM = 0; // Revolutions per minute
	float speedwind = 0; // Wind speed (m/s)
// <============> END OF WINDFLOW GLOBAL VARIABLE




// <JsonServer> START OF JSON

bool readRequest(EthernetClient& client) {
	bool currentLineIsBlank = true;
	while (client.connected()) {
		if (client.available()) {
			char c = client.read();
			if (c == '\n' && currentLineIsBlank) {
				return true;
			}
			else if (c == '\n') {
				currentLineIsBlank = true;
			}
			else if (c != '\r') {
				currentLineIsBlank = false;
			}
		}
	}
	return false;
}

JsonObject& prepareResponse(JsonBuffer& jsonBuffer) {
	JsonObject& root = jsonBuffer.createObject();

	// <>=============<> START TEMPHUMID CODE HERE <>=============<>
	float h = dht.readHumidity();
	// Read temperature as Celsius (the default)
	float t = dht.readTemperature();
	// Read temperature as Fahrenheit (isFahrenheit = true)
	float f = dht.readTemperature(true);

	// Check if any reads failed and exit early (to try again).
	if (isnan(h) || isnan(t) || isnan(f)) {
		Serial.println("Failed to read from DHT sensor!");
		return;
	}
	// Compute heat index in Fahrenheit (the default)
	float hif = dht.computeHeatIndex(f, h);
	// Compute heat index in Celsius (isFahreheit = false)
	float hic = dht.computeHeatIndex(t, h, false);

	JsonArray& humid = root.createNestedArray("Humidity");
	humid.add(h);

	JsonArray& ctemp = root.createNestedArray("CelciusTemperature");
	ctemp.add(t);

	JsonArray& ftemp = root.createNestedArray("FahrenheitTemperature");
	ftemp.add(f);

	JsonArray& cheat = root.createNestedArray("CelciusHeat");
	cheat.add(hic);

	JsonArray& fheat = root.createNestedArray("FahrenheitHeat");
	fheat.add(hif);
	// <>=============<> END TEMPHUMID CODE HERE <>=============<>

	// <>=============<> START AIRQUALITY CODE HERE <>=============<>
	MQ135 gasSensor = MQ135(0);
	float rzero = gasSensor.getRZero();
	float ppm = gasSensor.getPPM();
	int sensorValue = analogRead(0);       // read analog input pin 0

	JsonArray& airQanalog = root.createNestedArray("AirQualityAnalog");
	airQanalog.add(sensorValue);
	JsonArray& airQrZero = root.createNestedArray("AirQualityRZero");
	airQrZero.add(rzero);
	JsonArray& airQppm = root.createNestedArray("AirQualityPPM");
	airQppm.add(ppm);
	// <>=============<> END AIRQUALITY CODE HERE <>=============<>

	// <>=============<> START PHOTOCELL CODE HERE <>=============<>
	int photocellPin = 1;     // the cell and 10K pulldown are connected to a0
	int photocellReading;     // the analog reading from the sensor divider
							  //int LEDpin = 13;          // connect Red LED to pin 11 (PWM pin)
							  //int LEDbrightness;

	photocellReading = analogRead(photocellPin);

	JsonArray& photo = root.createNestedArray("Photocell");
	photo.add(photocellReading);
	// <>=============<> END PHOTOCELL CODE HERE <>=============<>

	// <>=============<> START VIBRATION CODE HERE <>=============<>
	long measurement = TP_init();
	delay(50);
	JsonArray& vibrate = root.createNestedArray("Vibration");
	vibrate.add(measurement);
	// <>=============<> END VIBRATION CODE HERE <>=============<>

	// <>=============<> START WATERFLOW CODE HERE <>=============<>
	if ((millis() - oldTime) > 1000) {
		detachInterrupt(sensorInterrupt);
		flowRate = ((1000.0 / (millis() - oldTime)) * pulseCount) / calibrationFactor;
		oldTime = millis();
		flowMilliLitres = (flowRate / 60) * 1000;
		totalMilliLitres += flowMilliLitres;
		unsigned int frac;
		JsonArray& fFlowRate = root.createNestedArray("FlowRate"); // L/min
		fFlowRate.add(flowRate);

		JsonArray& fFlowing = root.createNestedArray("CurrentLiquidFlowing"); // mL/sec
		fFlowing.add(flowMilliLitres);

		JsonArray& fFlow = root.createNestedArray("OutputLiquidQuantity"); // mL
		fFlow.add(totalMilliLitres);

		pulseCount = 0;

		attachInterrupt(sensorInterrupt, pulseCounter, FALLING);
	}
	// <>=============<> END WATERFLOW CODE HERE <>=============<>

	// <>=============<> START WINDFLOW CODE HERE <>=============<>
	Sample++;
	
	Serial.print(": Start measurement");
	windvelocity();
	Serial.println(" finished.");
	Serial.print("Counter: ");
	Serial.print(counter);
	Serial.print("; RPM: ");
	RPMcalc();
	Serial.print(RPM);
	Serial.print("; Wind speed: ");
	WindSpeed();
	Serial.print(speedwind);
	Serial.print(" [m/s]");
	Serial.println();
	delay(5000);
	JsonArray& jcount = root.createNestedArray("Counter"); // How Much Count of rotation on sensor
	jcount.add(counter);
	JsonArray& jrpm = root.createNestedArray("RPM"); // How Much Count of rotation / minutes
	jrpm.add(RPM);
	JsonArray& jwspeed = root.createNestedArray("WindSpeed"); // How Much Count of rotation / minutes
	jwspeed.add(speedwind);
	JsonArray& reqcount = root.createNestedArray("RequestCount"); // counter
	reqcount.add(Sample);
	// <>=============<> END WINDFLOW CODE HERE <>=============<>
	return root;
}

void writeResponse(EthernetClient& client, JsonObject& json) {
	client.println("HTTP/1.1 200 OK");
	client.println("Content-Type: application/json");
	client.println("Connection: close");
	client.println();

	json.prettyPrintTo(client);
}

// <JsonServer> END OF JSON



/*
// <************> START OF READ SENSOR
void temphumid() {
	// Reading temperature or humidity takes about 250 milliseconds!
	// Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
	float h = dht.readHumidity();
	// Read temperature as Celsius (the default)
	float t = dht.readTemperature();
	// Read temperature as Fahrenheit (isFahrenheit = true)
	float f = dht.readTemperature(true);

	// Check if any reads failed and exit early (to try again).
	if (isnan(h) || isnan(t) || isnan(f)) {
		Serial.println("Failed to read from DHT sensor!");
		return;
	}
	// Compute heat index in Fahrenheit (the default)
	float hif = dht.computeHeatIndex(f, h);
	// Compute heat index in Celsius (isFahreheit = false)
	float hic = dht.computeHeatIndex(t, h, false);

	Serial.print("Humidity: ");
	Serial.print(h);
	Serial.print(" %\t");
	Serial.print("Temperature: ");
	Serial.print(t);
	Serial.print(" *C ");
	Serial.print(f);
	Serial.print(" *F\t");
	Serial.print("Heat index: ");
	Serial.print(hic);
	Serial.print(" *C ");
	Serial.print(hif);
	Serial.println(" *F");
}

void airquality() {
	MQ135 gasSensor = MQ135(0);
	float rzero = gasSensor.getRZero();
	float ppm = gasSensor.getPPM();
	int sensorValue = analogRead(0);       // read analog input pin 0

	Serial.print("Analog Read : \t");  // prints the value read
	Serial.print(sensorValue, DEC);  // prints the value read
	Serial.print("\trZero Read : \t");  // prints the value read
	Serial.print(rzero, DEC);  // prints the value read
	Serial.print("\tPPM Read : \t");  // prints the value read
	Serial.println(ppm, DEC);  // prints the value read
}

void photocell() {
	int photocellPin = 1;     // the cell and 10K pulldown are connected to a0
	int photocellReading;     // the analog reading from the sensor divider
							  //int LEDpin = 13;          // connect Red LED to pin 11 (PWM pin)
							  //int LEDbrightness;

	photocellReading = analogRead(photocellPin);

	Serial.print("Photocell reading = \t");
	Serial.println(photocellReading);     // the raw analog reading
										  // LED gets brighter the darker it is at the sensor
										  // that means we have to -invert- the reading from 0-1023 back to 1023-0
	//photocellReading = 1023 - photocellReading;
	//now we have to map 0-1023 to 0-255 since thats the range analogWrite uses
	//LEDbrightness = map(photocellReading, 0, 1023, 0, 255);
	//analogWrite(LEDpin, LEDbrightness);
}

void vibration() {
	//  int ledPin = 13;

	long measurement = TP_init();
	delay(50);
	Serial.print("Vibration reading = \t");
	Serial.println(measurement);
	//	if (measurement > 1000) {
	//		digitalWrite(ledPin, HIGH);
	//	}
	//	else {
	//		digitalWrite(ledPin, LOW);
	//	}
}

void waterflow() {
	if ((millis() - oldTime) > 1000)    // Only process counters once per second
	{
		// Disable the interrupt while calculating flow rate and sending the value to
		// the host
		detachInterrupt(sensorInterrupt);

		// Because this loop may not complete in exactly 1 second intervals we calculate
		// the number of milliseconds that have passed since the last execution and use
		// that to scale the output. We also apply the calibrationFactor to scale the output
		// based on the number of pulses per second per units of measure (litres/minute in
		// this case) coming from the sensor.
		flowRate = ((1000.0 / (millis() - oldTime)) * pulseCount) / calibrationFactor;

		// Note the time this processing pass was executed. Note that because we've
		// disabled interrupts the millis() function won't actually be incrementing right
		// at this point, but it will still return the value it was set to just before
		// interrupts went away.
		oldTime = millis();

		// Divide the flow rate in litres/minute by 60 to determine how many litres have
		// passed through the sensor in this 1 second interval, then multiply by 1000 to
		// convert to millilitres.
		flowMilliLitres = (flowRate / 60) * 1000;

		// Add the millilitres passed in this second to the cumulative total
		totalMilliLitres += flowMilliLitres;

		unsigned int frac;

		// Print the flow rate for this second in litres / minute
		Serial.print("Flow rate: \t");
		Serial.print(int(flowRate));  // Print the integer part of the variable
		Serial.print(".");             // Print the decimal point
									   // Determine the fractional part. The 10 multiplier gives us 1 decimal place.
		frac = (flowRate - int(flowRate)) * 10;
		Serial.print(frac, DEC);      // Print the fractional part of the variable
		Serial.print("L/min");
		// Print the number of litres flowed in this second
		Serial.print("\tCurrent Liquid Flowing: ");             // Output separator
		Serial.print(flowMilliLitres);
		Serial.print("mL/Sec");

		// Print the cumulative total of litres flowed since starting
		Serial.print("\tOutput Liquid Quantity: ");             // Output separator
		Serial.print(totalMilliLitres);
		Serial.println("mL");

		// Reset the pulse counter so we can start incrementing again
		pulseCount = 0;

		// Enable the interrupt again now that we've finished sending output
		attachInterrupt(sensorInterrupt, pulseCounter, FALLING);
	}
}
// <************> END OF READ SENSOR
*/

// <====> start of depedency method by read sensor
long TP_init() {
	int EP = 5;

	delay(10);
	long measurement = pulseIn(EP, HIGH);  //wait for the pin to get HIGH and returns measurement
	return measurement;
}

void pulseCounter()
{
	// Increment the pulse counter
	pulseCount++;
}

// Measure wind speed
void windvelocity()
{
	speedwind = 0;
	counter = 0;
	attachInterrupt(1, addcount, CHANGE);
	unsigned long millis();
	long startTime = millis();
	while (millis() < startTime + period) {}

	detachInterrupt(1);
}

void RPMcalc()
{
	RPM = ((counter / jml_celah) * 60) / (period / 1000); // Calculate revolutions per minute (RPM)
}

void WindSpeed()
{
	speedwind = ((2 * pi * radio * RPM) / 60) / 1000; // Calculate wind speed on m/s
}

void addcount()
{
	counter++;
}
// <====> end of depedency method by read sensor




void setup()
{
	//Serial.begin(9600);
	dht.begin();
	attachInterrupt(sensorInterrupt, pulseCounter, FALLING);
	Ethernet.begin(mac, ip);
	server.begin();
}

void loop() {
	// Wait a few seconds between measurements.
	/*
	delay(2000);
	vibration();
	photocell();
	temphumid();
	airquality();
	waterflow();
	Serial.println("");
	*/
	EthernetClient client = server.available();
	if (client) {
		bool success = readRequest(client);
		if (success) {
			StaticJsonBuffer<500> jsonBuffer;
			JsonObject& json = prepareResponse(jsonBuffer);
			writeResponse(client, json);
		}
		delay(1);
		client.stop();
	}
}
