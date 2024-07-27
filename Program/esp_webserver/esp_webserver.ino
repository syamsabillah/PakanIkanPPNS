#include <WiFi.h>
#include <WebServer.h>
#include <EEPROM.h>

#define EEPROM_SIZE 64

const char *apSSID = "Pakan Lele - IoT";
const char *apPassword = "12345678";

WebServer server(80);

String ssid = "";
String password = "";

// HTML page to configure WiFi
const char *htmlPage = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>WiFi Login</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            background-color: #f4f4f4;
        }

        .container {
            width: 300px;
            margin: 0 auto;
            padding: 20px;
            background-color: #fff;
            border-radius: 5px;
            box-shadow: 0 0 10px rgba(0, 0, 0, 0.1);
            margin-top: 100px;
        }

        h2 {
            text-align: center;
        }

        .form-group {
            margin-bottom: 20px;
        }

        label {
            display: block;
            font-weight: bold;
        }

        input[type="text"],
        input[type="password"] {
            width: 90%;
            padding: 10px;
            border: 1px solid #ccc;
            border-radius: 5px;
        }

        input[type="submit"] {
            background-color: #007BFF;
            color: #fff;
            padding: 10px 20px;
            border: none;
            border-radius: 5px;
            cursor: pointer;
        }

        input[type="submit"]:hover {
            background-color: #0056b3;
        }
    </style>
</head>
<body>
    <div class="container">
        <h2>WiFi Login</h2>
        <form action="/submit" method="post">
            <div class="form-group">
                <label for="SSID">SSID:</label>
                <input type="text" id="SSID" name="SSID" required>
            </div>
            <div class="form-group">
                <label for="password">Password:</label>
                <input type="password" id="password" name="password">
                <input type="checkbox" id="showPassword"> Show Password<br>
            </div>
            <div class="form-group">
                <input type="submit" value="Save and Restart">
            </div>
        </form>
        <script>
        const passwordInput = document.getElementById('password');
        const showPasswordCheckbox = document.getElementById('showPassword');

        showPasswordCheckbox.addEventListener('change', function () {
            if (showPasswordCheckbox.checked) {
                passwordInput.type = 'text';
            } else {
                passwordInput.type = 'password';
            }
        });
    </script>
    </div>
</body>
</html>
)rawliteral";

void handleRoot() {
  server.send(200, "text/html", htmlPage);
}

void handleSubmit() {
  ssid = server.arg("SSID");
  password = server.arg("password");
  
  // Save WiFi credentials to EEPROM
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.writeString(0, ssid);
  EEPROM.writeString(32, password);
  EEPROM.commit();
  
  server.send(200, "text/html", "Configuration Saved. ESP will restart and try to connect to the specified network.");
  
  // Restart the ESP to apply the new WiFi settings
  delay(1000);
  ESP.restart();
}

void setup() {
  Serial.begin(115200);

  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);

  // Start access point
  WiFi.softAP(apSSID, apPassword);
  Serial.println("Access Point Started");
  Serial.print("AP IP Address: ");
  Serial.println(WiFi.softAPIP());

  // Start web server
  server.on("/", handleRoot);
  server.on("/submit", handleSubmit);
  server.begin();
  Serial.println("HTTP server started");

  // Read WiFi credentials from EEPROM
  ssid = EEPROM.readString(0);
  password = EEPROM.readString(32);

  // Try to connect to WiFi with stored credentials
  if (ssid.length() > 0 && password.length() > 0) {
    WiFi.begin(ssid.c_str(), password.c_str());
    Serial.print("Connecting to WiFi: ");
    Serial.println(ssid);

    unsigned long startAttemptTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 15000) {
      Serial.print(".");
      delay(100);
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("Connected!");
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
      
      // Close access point
      WiFi.softAPdisconnect(true);
      Serial.println("Access Point Closed");
    } else {
      Serial.println("Failed to connect, clearing EEPROM and restarting...");
      EEPROM.begin(EEPROM_SIZE);
      for (int i = 0; i < EEPROM_SIZE; i++) {
        EEPROM.write(i, 0);
      }
      EEPROM.commit();
      delay(1000);
      ESP.restart();
    }
  }
}

void loop() {
  server.handleClient();
}
