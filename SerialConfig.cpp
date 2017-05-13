#include <Log.h>
#include <Config.h>

#define TIMEOUT_ENTER 50
#define ENTER 13            // byte value of the Enter key (LF)

void readLine(String& str, int timeout) {
	int index = 0;
	if (timeout == 0)
		timeout = 9999999;
	unsigned long end = millis() + timeout;

	while (true) {
		if (millis() > end) {
			Serial.println("\r\n TIMEOUT ! ");
			return;
		}
		if (Serial.available()) {

			char c = Serial.read();
			switch (c) {
			case '\n':
			case '\r':
				Serial.print("\r\n");
				return;
				break;

			case '\b':
				// backspace
				if (str.length() > 0) {
					str.remove(str.length() - 1);
					Serial.print(' ');
				}
				break;

			default:
				// normal character entered. add it to the buffer
				Serial.print(c);
				str.concat(c);
				break;
			}
		}
	}
}

void ask(String question, String& input, int timeout) {
	Serial.print(question);
	readLine(input, timeout);
}

void showFullConfig(){
    Serial.println(config.summary());
}

void configMenu() {
	while (true) {
		showFullConfig();
//		object.printTo(show);
		Serial.println(
				"\r\n__________________ Config JSON ________________________________________");
		showFullConfig();
		Serial.println(
				"\r\n_______________________________________________________________________");
		Serial.print("\r\n Create, Update, Delete, Save, eXit [cudsx] : ");
		String line;
		readLine(line, 0);
		Serial.println("");
		if (line.length() == 0)
			return;
		else if (line.equals("c") || line.equals("u")) {
			String key, value;
			ask("\r key   : ", key, 0);
			ask("\r value : ", value, 0);
			config.set(key.c_str(),value.c_str());
		} else if (line.equals("s")) {
			Serial.println("\r\n Saving.. ");
			showFullConfig();
			Serial.println("done.\r\n ");
		} else if (line.equals("d")) {
			String key;
			ask("\r key   : ", key, 0);
			config.remove(key.c_str());
		} else if (line.equals("x")) {
			Serial.println("\r\n Exit config");
			return;
		}
	}
}

void waitConfig() {
	String line;
	unsigned long end = millis() + 5000;
	while (true) {
		if (millis() > end) {
			Serial.println(" input timed out, starting...");
			break;
		}
		Serial.printf(" Hit 'x' + Enter for menu within 5 seconds : ");
		readLine(line, 10000);
		if (line.equals("x")) {
			configMenu();
			return;
		} else {
			Serial.printf(" invalid input >>%s<< , try again.\n", line.c_str());
		}
		line = "";
	}
}
