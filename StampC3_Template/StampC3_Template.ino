
#include <Wire.h>

#include "WifiNet.h"
WifiAccessPointServer server("myAP", "myPassword", 80);

//#include <WiFi.h>
// Set these to your desired credentials.
//const char *ssid = "yourAP";
//const char *password = "yourPassword";
//
//WiFiServer server(80);
//WiFiClient client;

unsigned long sensors_ts;
unsigned long update_ts;

unsigned long led_ts;
bool led_state = true;

#include "Robot.h"
Robot_c robot;

void setup() {

  Serial.begin(115200);
  Serial.println("Starting");

//  WiFi.mode( WIFI_AP );
//
//  // You can remove the password parameter if you want the AP to be open.
//  // a valid password must have more than 7 characters
//  if (!WiFi.softAP(ssid, password)) {
//    log_e("Soft AP creation failed.");
//    while (1);
//  }
//  IPAddress myIP = WiFi.softAPIP();
//  Serial.print("AP IP address: ");
//  Serial.println(myIP);
//  server.begin();
//  server.setNoDelay(true);

  Serial.println("Server started");

  //  Serial.setDebugOutput(true);

  server.begin();

  robot.initialise();

  sensors_ts = millis();
  update_ts = millis();
  led_ts = millis();

  pinMode(6, OUTPUT );
  pinMode(7, OUTPUT );
  digitalWrite(6, led_state );
  analogWrite(7, 120);
  pinMode(1, INPUT);
  pinMode(0, INPUT);
  pinMode(4, INPUT);



}

//void serviceWiFiClient() {
//  if (!client || !client.connected()) {
//    if (client) {
//      client.stop();
//    }
//
//    WiFiClient newClient = server.accept();
//    if (newClient) {
//      client = newClient;
//      Serial.println("Client connected");
//    }
//  }
//}

static unsigned long dt;
void loop() {

  server.update();

//  if( millis() - led_ts > 500 ) {
//    led_ts = millis();
//
//    led_state = !led_state;
//    digitalWrite( 6, led_state );
//    digitalWrite( 7, led_state );
//  }
//  
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
//    Serial.print( analogRead(0) );
//    Serial.print(",");
//    Serial.print( analogRead( 1 ));
//    Serial.print(",");
//    Serial.println( analogRead( 4 ));
  }
  if ( robot.getButton() ) robot.playTone( 330, 250 );

  if ( millis() - sensors_ts > 250 ) {
      sensors_ts = millis();
      char buf[64];
      memset(buf, 0, sizeof( buf ));
      sprintf(buf, "%d, %d, %d, %d, %d, %d, %d, %d\n", analogRead(0), analogRead(1), analogRead(4), (int)robot.surface.reading[0], (int)robot.surface.reading[1], (int)robot.surface.reading[2], (int)robot.surface.reading[3], (int)robot.surface.reading[4]  );  
      server.send( buf );
  }
 
  
}
