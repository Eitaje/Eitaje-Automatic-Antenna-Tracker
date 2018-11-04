int connectionId;

void establish_comm()
{
	//*
	String SSID = "AntennaTracker";
	String PASS = "Hello2018";

	String cmd = "AT+CWSAP=\"";
	cmd += SSID;
	cmd += "\",\"";
	cmd += PASS;
	cmd += "\",";
	cmd += "11";
	cmd += ",";
	cmd += "4";
	cmd += "\r\n";
	

	//DEBUG
	if(DEBUG_WIFI)
		port_Debug.println(cmd);

	//*/
	leds[AIRPLANE_GPS] = CRGB::AliceBlue;
	FastLED.show();
	digitalWrite(13, HIGH);
	sendData("AT+RST\r\n", 250, DEBUG); // reset module
	
	leds[AIRPLANE_GPS] = CRGB::Black;
	FastLED.show();
	digitalWrite(13, LOW);
	sendData("AT+CWMODE=2\r\n", 250, DEBUG); // configure as access point
	
	leds[AIRPLANE_GPS] = CRGB::Aqua;
	FastLED.show();
	digitalWrite(13, HIGH);
	sendData(cmd, 250, DEBUG); // configure security
	
	leds[AIRPLANE_GPS] = CRGB::Black;
	FastLED.show();
	digitalWrite(13, LOW);
	sendData("AT+RST\r\n", 250, DEBUG); // reset module
	
	leds[AIRPLANE_GPS] = CRGB::Aquamarine;
	FastLED.show();
	digitalWrite(13, HIGH);
	sendData("AT+CIFSR\r\n", 250, DEBUG); // get ip address //192.168.4.1
	
	leds[AIRPLANE_GPS] = CRGB::Black;
	FastLED.show();
	digitalWrite(13, LOW);
	sendData("AT+CIPMUX=1\r\n", 250, DEBUG); // configure for multiple connections

	
	leds[AIRPLANE_GPS] = CRGB::Beige;
	FastLED.show();
	digitalWrite(13, HIGH);
	sendData("AT+CIPSERVER=1,5776\r\n", 250, DEBUG); // turn on server on port 5775
	digitalWrite(13, HIGH);
	
	leds[AIRPLANE_GPS] = CRGB::Black;
	FastLED.show();
	digitalWrite(13, LOW);
	//*/

	
}


/*
Name: sendData
Description: Function used to send data to ESP8266.
Params: command - the data/command to send; timeout - the time to wait for a response; debug - print to Serial window?(true = yes, false = no)
Returns: The response from the esp8266 (if there is a reponse)
*/

String sendData(String command, const int timeout, boolean debug)
{
	String response = "";
	port_Wifi.print(command); // send the read character to the esp8266
	long int time = millis();
	while ((time + timeout) > millis())
	{
		while (port_Wifi.available())
		{
			// The esp has data so display its output to the serial window
			char c = port_Wifi.read(); // read the next character.
			response += c;
		}
	}

	if (debug)
	{
		port_Debug.print(response);
	}

	return response;
}


/*
Name: prepareReturnMsg
*/

void prepareReturnMsg(String ret_msg)
{
	// generate web page
	String cipSend = "AT+CIPSEND=";
	cipSend += connectionId;
	cipSend += ",";
	cipSend += ret_msg.length();
	cipSend += "\r\n";
	sendData(cipSend, 50, DEBUG);
	sendData(ret_msg, 50, DEBUG);

	// make close command
	String closeCommand = "AT+CIPCLOSE=";
	closeCommand += connectionId; // append connection id
	closeCommand += "\r\n";
	sendData(closeCommand, 50, DEBUG); // close connection

}
/*
SerialEvent occurs whenever a new data comes in the hardware serial RX. This
routine is run between each time loop() runs, so using delay inside loop can
delay response. Multiple bytes of data may be available.
*/
/*
void serialEvent2() {
	delay(100);
	if (port_Wifi.find("+IPD,"))
	{
		//dEBUG
		port_Debug.println("*** IPD ***");

		delay(100); // wait for the serial buffer to fill up (read all the serial data)
					// get the connection id so that we can then disconnect
		int connectionId = port_Wifi.read() - 48; // subtract 48 because the read() function returns
												// the ASCII decimal value and 0 (the first decimal number) starts at 48
		
		port_Wifi.find(":");//find command's start
		
		while (port_Wifi.available()) {

			// get the new byte:
			char inChar = (char)port_Wifi.read();
			// add it to the inputString:
			inputString += inChar;
			// if the incoming character is a newline, set a flag so the main loop can
			// do something about it:
			if (inChar == '\n') {
				stringComplete = true;
			}
		}

		if (stringComplete) {


			for (int i = 0; i < inputString.length(); i++) {
				myParser.processByte(inputString.charAt(i));
			}

			inputString = "";
			stringComplete = false;
		}
		
		prepareReturnMsg("Done command :-)");

	}
	
}
*/
