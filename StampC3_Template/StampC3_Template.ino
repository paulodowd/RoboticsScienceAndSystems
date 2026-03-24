
#include <Wire.h>
#include <WiFi.h>
// Set these to your desired credentials.
const char *ssid = "yourAP";
const char *password = "yourPassword";

WiFiServer server(80);
WiFiClient client;

unsigned long wifi_ts;
unsigned long update_ts;
#include "Robot.h"
Robot_c robot;

void setup() {

  Serial.begin(115200);
  Serial.println("Starting");

  WiFi.mode( WIFI_AP );

  // You can remove the password parameter if you want the AP to be open.
  // a valid password must have more than 7 characters
  if (!WiFi.softAP(ssid, password)) {
    log_e("Soft AP creation failed.");
    while (1);
  }
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  server.begin();
  server.setNoDelay(true);

  Serial.println("Server started");

  //  Serial.setDebugOutput(true);

  robot.initialise();

  wifi_ts = millis();
  update_ts = millis();



}

void serviceWiFiClient() {
  if (!client || !client.connected()) {
    if (client) {
      client.stop();
    }

    WiFiClient newClient = server.accept();
    if (newClient) {
      client = newClient;
      Serial.println("Client connected");
    }
  }
}

static unsigned long dt;
void loop() {
  
  unsigned long start = micros();
  if ( millis() - update_ts > 25 ) {
    update_ts = millis();


    robot.getSurfaceSensors();
    robot.getPose();

    // Line following example
    float m = robot.surface.reading[1] - robot.surface.reading[3];
    float sum = robot.surface.reading[1] + robot.surface.reading[3];
    sum *= 0.5;
    m /= sum;
    m *= 2.0;
    //
    float turn = 15;
    float l_pwm, r_pwm;
    l_pwm = ( m * turn ) + 25.0;
    r_pwm = ( m * -turn ) + 25.0;
    robot.setMotorPWM( l_pwm, r_pwm );

  }
  if ( robot.getButton() ) robot.playTone( 330, 250 );

  if ( millis() - wifi_ts > 100 ) {
    serviceWiFiClient();
    wifi_ts = millis();
    if (client && client.connected() ) {
      client.println(dt);
    }
  }
  unsigned long end = micros();
  dt = end - start;

  
}
