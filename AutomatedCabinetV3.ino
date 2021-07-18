/*
 Name:		AutomatedCabinetV3.ino
 Created:	2/21/2021 8:53:31 PM
 Author:	Liam Price
 Purpose: Runs on ESP32 Espressif chip to automate a timber cabinet to be opened and closed over the internet.
 GitHub: https://github.com/LeehamElectronics/Automated-Cabinet
*/

/*

██╗░░░░░███████╗███████╗██╗░░██╗░█████╗░███╗░░░███╗██╗░██████╗      ░██████╗░█████╗░███████╗████████╗░██╗░░░░░░░██╗░█████╗░██████╗░███████╗
██║░░░░░██╔════╝██╔════╝██║░░██║██╔══██╗████╗░████║╚█║██╔════╝      ██╔════╝██╔══██╗██╔════╝╚══██╔══╝░██║░░██╗░░██║██╔══██╗██╔══██╗██╔════╝
██║░░░░░█████╗░░█████╗░░███████║███████║██╔████╔██║░╚╝╚█████╗░      ╚█████╗░██║░░██║█████╗░░░░░██║░░░░╚██╗████╗██╔╝███████║██████╔╝█████╗░░
██║░░░░░██╔══╝░░██╔══╝░░██╔══██║██╔══██║██║╚██╔╝██║░░░░╚═══██╗      ░╚═══██╗██║░░██║██╔══╝░░░░░██║░░░░░████╔═████║░██╔══██║██╔══██╗██╔══╝░░
███████╗███████╗███████╗██║░░██║██║░░██║██║░╚═╝░██║░░░██████╔╝      ██████╔╝╚█████╔╝██║░░░░░░░░██║░░░░░╚██╔╝░╚██╔╝░██║░░██║██║░░██║███████╗
╚══════╝╚══════╝╚══════╝╚═╝░░╚═╝╚═╝░░╚═╝╚═╝░░░░░╚═╝░░░╚═════╝░     ╚═════╝░░╚════╝░╚═╝░░░░░░░░╚═╝░░░░░░╚═╝░░░╚═╝░░╚═╝░░╚═╝╚═╝░░╚═╝╚══════╝

*/

/* ---------------------------------------------------------------------------------------------------------------------------
--------------------------------------------------    Include Libraries   ----------------------------------------------------
--------------------------------------------------------------------------------------------------------------------------- */
#pragma region
#include "include/mqttConfiguration.h"  /* mqtt config file, see wiki for details on how to create it: https://github.com/LeehamElectronics/Automated-Cabinet/wiki */
#include "include/configuration.h" /* general config file to make life easier for you */

/* WiFiManager Development builds seem to work better and are more stable: https://github.com/tzapu/WiFiManager/tree/development */
/* If WiFi Manager has HTTP Header issues use the following lib instead: https://github.com/Brunez3BD/WIFIMANAGER-ESP32 */
#include <WiFiManager.h> 
#include <WiFi.h>

/*
 * This 'PubSubClient' is actually the MQTT Arduino Library that we are using to make the Arduino
 * board communicate with the Control Panel I made, you might be wondering, why is it called
 * 'PubSubClient' and not ArduinoMQTT? Well there is a more 'official' Arduino MQTT library available,
 * however it is very new at the time of writing and I prefer using these more 'established' libraries
 * for stability.
 */
#include <PubSubClient.h> 

/* this was required for ESP32 chips to use analogWrite for some reason, look it up https://github.com/ERROPiX/ESP32_AnalogWrite */
#include <analogWrite.h> 

/*
 * Used to store information even when the robot is turned off, dont abuse it (stay away from for loops lol), only has a set
 * amount of read / write cycles. If you need to use EEPROM A LOT try using a SD Card instead!
 */
#include <EEPROM.h> 

/* We use ArduinoJSON so we can send, recieve and decode JSON data via MQTT */
#include <ArduinoJson.h> 
#pragma endregion

/* ---------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------    Create Main Objects   ---------------------------------------------------
--------------------------------------------------------------------------------------------------------------------------- */
#pragma region
WiFiClient espClient;
PubSubClient client(espClient);
WiFiManager wifiManager;
#pragma endregion


/* ---------------------------------------------------------------------------------------------------------------------------
---------------------------------------------------    GPIO Pinout Menu   ----------------------------------------------------
--------------------------------------------------------------------------------------------------------------------------- */
#pragma region
/* See Wiki for a schematic and more detailed pinout menu: https://github.com/LeehamElectronics/Automated-Cabinet/wiki */
const int limitSwitchPin0 = 17;	           /* Front limit switch (closed when cabinet is in open position) */
const int limitSwitchPin1 = 18;            /* Rear limit switch (closed when cabinet is in closed position) */ 
const int manualToggleButtonPin = 16;      /* This button is hidden inside the cabinet, can be used for testing */
const int hBridgeMotorPin0 = 19;           /* the number of the h bridge pin 1 */
const int hBridgeMotorPin1 = 21;           /* the number of the h bridge pin 2 */

int limitSwitchState0 = 0;                 /* variable to keep track of the state of limitSwitchPin0 */
int limitSwitchState1 = 0;                 /* variable to keep track of the state of limitSwitchPin1 */
int manualToggleButtonState = 0;           /* variable to keep track of the state of manualToggleButtonPin */

String drawPosition = "closed";            /* Allows the program to keep track of the last known position of the draw*/
boolean drawIsMoving = false;              /* Allows the program to keep track of weather the draw is currently moving or not */
boolean awaitingIntervention = false;      /* This becomes true when a special event occurs such as someone manually breaking into the safe, or auto-close-assist. */
String drawMovingDirection = "null";       /* Keep track of what direction the draw is moving, can one of the following:  null | closing | opening */

unsigned long previousMillis = millis();   /* This keeps track of the current time in miliseconds, we use it for detecting issues with motor or cabinet draw. */
int failSafeTimout = 6500;                 /* Specify how much time we should allocate to the cabinet open / close before we intervene and shut down motor (prevents house from burning down) */
#pragma endregion


/* ---------------------------------------------------------------------------------------------------------------------------
---------------------------------------------------    Setup Function   ----------------------------------------------------
--------------------------------------------------------------------------------------------------------------------------- */
#pragma region
void setup() {
	Serial.println(String(map()));
	pinMode(hBridgeMotorPin0, OUTPUT);
	pinMode(hBridgeMotorPin1, OUTPUT);
	pinMode(limitSwitchPin0, INPUT_PULLUP);  /* Although we told the ESP32 to pullup this pin, some chips don't support it so you need to manually pull this up with a resistor. */
	pinMode(limitSwitchPin1, INPUT_PULLUP);
	pinMode(manualToggleButtonPin, INPUT_PULLUP);
	Serial.begin(9600); /* Start the serial monitor so we can see what the ESP32 is thinking via serial USB connection, this is good for diagnosing issues. */
	Serial.println("Automated Cabinet Systems Starting, Built and Designed By Liam Price 2017");

	Serial.println("Connecting to WiFi...");
	/* Set callback function that gets called when connecting to previous WiFi fails, and then enters Access Point mode ready for configuration via your phone or computer. */
	wifiManager.setAPCallback(configModeCallback);

	/* If our ESP32 can not find or connect to a known WiFi network, this code will force it to enter AP mode where we can configure it. */
	/* Here you can specify the AP SSID and password */
	if (!wifiManager.autoConnect(PORTAL_SSID, PORTAL_PWD)) {
		Serial.println("failed to connect to WiFi and hit timeout");
		/* Restart ESP32 Chip */
		ESP.restart();
		delay(1000);
	}

	/* If program gets to this point that means we connected to WiFi successfully! */
	Serial.println("Connected to WiFi successfully :)");

	/* Now we can try connecitng to our MQTT server */
	client.setServer(MQTT_SERVER, MQTT_PORT); // configure these in mqttConfiguration.h file
	client.setCallback(callback);

	Serial.println("Setup function completed");
	delay(100); /* Give our ESP32 some time to breath right? */

}
#pragma endregion


/* ---------------------------------------------------------------------------------------------------------------------------
--------------------------------------------------    Main Loop Function   ---------------------------------------------------
--------------------------------------------------------------------------------------------------------------------------- */
#pragma region
void loop() {

	/* The next 3 lines check what the state of each button is (open or closed...) */
	limitSwitchState0 = digitalRead(limitSwitchPin0); /* Front limit switch (closed when cabinet is in open position) */
	limitSwitchState1 = digitalRead(limitSwitchPin1);  /* Rear limit switch (closed when cabinet is in closed position) */
	manualToggleButtonState = digitalRead(manualToggleButtonPin);  /* This button is hidden inside the cabinet, can be used for testing... */

    if (drawIsMoving == true) {

		if (drawMovingDirection == "opening") {

			if (limitSwitchState0 == LOW) { 
				drawPosition = "open";
				stopDrawFromMoving();
				client.publish("acs_output", (char*)"OPEN");
			}
			else {
				/* Fail safe timout check routine */
				unsigned long currentMillis = millis();
				if (currentMillis - previousMillis >= failSafeTimout) {
					// save the last time we checked the time in memory
					previousMillis = currentMillis;
					awaitingIntervention = true;
					stopDrawFromMoving();
					client.publish("acs_output", (char*)"ERROR26");
					Serial.println("ERROR! Motors have stopped due to timeout, are the limit switches broken? Is there to much weight in the Draw?");
				}
			}
		}

		else {

			if (limitSwitchState1 == LOW) { // when limit switch 1 is hit, the draw is CLOSED
				drawPosition = "closed";
				stopDrawFromMoving();
				client.publish("acs_output", (char*)"CLOSED");
			}
			else {
				/* Fail safe timout check routine */
				unsigned long currentMillis = millis();
				if (currentMillis - previousMillis >= failSafeTimout) {
					// save the last time we checked the time in memory
					previousMillis = currentMillis;
					awaitingIntervention = true;
					stopDrawFromMoving();
					client.publish("acs_output", (char*)"ERROR26");
					Serial.println("ERROR! Motors have stopped due to timeout, are the limit switches broken? Is there to much weight in the Draw?");
				}
			}		
		}
    }
	else {
		if (awaitingIntervention == false)
		{

			if (drawPosition == "closed") {
				if (limitSwitchState1 == HIGH) { // when limit switch 1 is released during no operation in the closed position, this means somone has MANUALLY OPENED the draw!!
					drawPosition = "open";
					stopDrawFromMoving(); // safety measure
					awaitingIntervention = true; // This prevents the program from getting stuck into a loop, this remains true untill the draw is activated via button / MQTT
					client.publish("acs_output", (char*)"ERROR11");
				}
			}
			else if (drawPosition == "open") {
				if (limitSwitchState0 == HIGH) { // when limit switches 1 is released during no operation in the open position, someone is trying to manually push the draw back in, so we will assist them!
					drawPosition = "open";
					awaitingIntervention = true; // This prevents the program from getting stuck into a loop, this remains true untill the draw is activated via button / MQTT
					client.publish("acs_output", (char*)"C_ASIST"); 
					close_draw();
				}
			}
		}
	}

    if (manualToggleButtonState == LOW) {    // if internal button is manually hit, activate draw
		toggle_draw();
		}


	if (!client.connected()) {
		Serial.println("Not connected to MQTT, trying to connect to broker...");
		stopDrawFromMoving();  // Must stop motors before exiting loop to search for MQTT server
		reconnect();
	}
	client.loop();

}				
#pragma endregion


/* ---------------------------------------------------------------------------------------------------------------------------
------------------------------------------------    Toggle Motor Functions   -------------------------------------------------
--------------------------------------------------------------------------------------------------------------------------- */
#pragma region
void toggle_draw() {
    Serial.println("Draw Activated");
	previousMillis = millis(); // Reset Fail Safe Timer
    if (drawPosition == "open") {
        drawBackward();
        drawPosition = "closed";
    }
    else if (drawPosition == "closed") {
        drawForward();
        drawPosition = "open";
    }
}

void open_draw() {
	Serial.println("Draw Opening Function");
	previousMillis = millis(); // Reset Fail Safe Timer
	awaitingIntervention = false;
	if (drawPosition == "closed") {
		drawForward();
		drawPosition = "open";
	}
	else
	{
		Serial.println("Draw is already Open!");
		client.publish("acs_output", (char*)"A_OPEN");
	}
}

void close_draw() {
	Serial.println("Draw Closing Function");
	previousMillis = millis(); // Reset Fail Safe Timer
	awaitingIntervention = false;
	if (drawPosition == "open") {
		drawBackward();
		drawPosition = "closed";
	}
	else
	{
		Serial.println("Draw is already Closed!");
		client.publish("acs_output", (char*)"A_CLOSED");
	}
}
#pragma endregion


/* ---------------------------------------------------------------------------------------------------------------------------
-------------------------------------------    H Bridge Motor Control Functions   --------------------------------------------
--------------------------------------------------------------------------------------------------------------------------- */
#pragma region
void drawForward() {
    /* Turn H Bridge motor driver on in the foward polarity */
    digitalWrite(hBridgeMotorPin0, LOW);
    digitalWrite(hBridgeMotorPin1, HIGH);
    Serial.println("motor-foward-func");
	drawIsMoving = true;
	drawMovingDirection = "opening";
}

/* Inverse function of function above */
void drawBackward() {
    // turn H Bridge on with the REVERSED polarity
    digitalWrite(hBridgeMotorPin0, HIGH);
    digitalWrite(hBridgeMotorPin1, LOW);
    Serial.println("motor-back-func");
	drawIsMoving = true;
	drawMovingDirection = "closing";
}

/* Stop H Bridge motor driver */
void stopDrawFromMoving() {
    //turn H Bridge motor driver OFF by turning both pins to LOW
    digitalWrite(hBridgeMotorPin0, LOW);
    digitalWrite(hBridgeMotorPin1, LOW);
    Serial.println("motor-stop-func");
	drawIsMoving = false;
	drawMovingDirection = "null";
}
#pragma endregion


/* ---------------------------------------------------------------------------------------------------------------------------
--------------------------------------    Networking Callback and Reconnect Functions   --------------------------------------
--------------------------------------------------------------------------------------------------------------------------- */
#pragma region
/* Function gets called when WiFiManager enters configuration mode */
void configModeCallback(WiFiManager* myWiFiManager) {
	Serial.println("Entered config mode");
	stopDrawFromMoving();  // Must stop motors before exiting loop
	Serial.println(WiFi.softAPIP());
	Serial.println(myWiFiManager->getConfigPortalSSID());  // print SSID
}


/* ---------------------------------------------------------------------------------
------------------------    Callback Function for MQTT  ----------------------------
------------- This function runs when data is recieved from mqtt server ------------
--------------------------------------------------------------------------------- */
void callback(char* topic, byte* payload, unsigned int length) {
	Serial.print("Topic is: ");
	Serial.println(topic);
	if (strcmp(topic, "acs_input") == 0) {
		Serial.println("acs_input");
		String payload_formatted = String((char*)payload);
		Serial.println("Payload formatted is: " + payload_formatted);
		Serial.println("Payload NOT formatted is: " + String(payload[0]));
		if (payload[0] == 't') {
			Serial.println("Toggling Safe");
			toggle_draw();
		}
		else if (payload[0] == 'o') {
			Serial.println("Opening Safe");
			open_draw();
		}
		else if (payload[0] == 'c') {
			Serial.println("Closing Safe");
			close_draw();
		}
	}

	else {
		Serial.print("Topic not relevant!");
	}
}


/* Function gets called when we lose connection to MQTT server */
void reconnect() {
	/* Loop until we're reconnected */
	if (!client.connected()) {
		Serial.print("Attempting MQTT connection...");
		/* Attempt to connect */
		if (client.connect("AutomatedCabinet", MQTT_USERNAME, MQTT_PASSWORD)) {
			Serial.println("connected");
			/* General input to tank from control panel: */
			client.subscribe("acs_input");
		}
		else {
			Serial.print("failed, rc="); /* Print error code, here is your reference: https://ldprice.com/public_images/chrome_LaZ4xARWcT.png */
			Serial.print(client.state());
			Serial.println(" try again in 2 seconds");
			/* Wait 2 seconds before retrying (don't wanna get flagged as a DOS attack...) */
			delay(2000);

		}
	}
}
#pragma endregion