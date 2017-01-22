#define VerM 0
#define Verm 03

unsigned long PERIOD = 1L;  // 3 min send once

#include <DHT22.h>
#include <SoftwareSerial.h>   // 引用程式庫
#include <SFE_BMP180.h>
#include <Wire.h>


#include <LCD5110_Graph.h>
#define DHT22_PIN 10

LCD5110 myGLCD(7, 6, 5, 3, 4);
extern unsigned char SmallFont[];
//extern uint8_t dinasur[];
extern uint8_t The_End[];
extern uint8_t arduino_logo[];
extern uint8_t dinov[];
extern uint8_t dinoh[];


DHT22 myDHT22(DHT22_PIN);
SoftwareSerial BT(8, 9); // 接收腳, 傳送腳
SFE_BMP180 pr;
#define ALTITUDE 55.0 // Altitude of SparkFun's HQ in Boulder, CO. in meters









unsigned long oldtime = 10000L;
double getPressure();
double baseline;

void setup() {
  char buf[40];
  int i;
  // put your setup code here, to run once:
  for (i = 0; i < 3; i++) {
    digitalWrite(12, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(100);              // wait for a second
    digitalWrite(12, LOW);    // turn the LED off by making the voltage LOW
    delay(100);
  }
  myGLCD.InitLCD();
  myGLCD.setFont(SmallFont);
  myGLCD.clrScr();
  myGLCD.update();

  myGLCD.drawBitmap(0, 0, dinov, 84, 48);
  myGLCD.update();
  delay(3000);

  myGLCD.clrScr();
  Serial.begin(9600);
  pinMode(0, OUTPUT);

  BT.begin(115200);
  myGLCD.print("WeatherMonitor", CENTER, 0);
  myGLCD.print("Outdoor", CENTER, 12);
  myGLCD.print("by cnwang 2016", CENTER, 30);
  sprintf(buf, "Ver. :%d,%d", VerM, Verm);
  myGLCD.print(buf, CENTER, 40);




  myGLCD.update();
  delay(5000);
  myGLCD.clrScr();

  myGLCD.print("BMP180 init...", CENTER, 0);
  if (pr.begin()) {
    myGLCD.print("SUCCESS", CENTER, 15);
    baseline = pr.startTemperature();
    baseline = getPressure();

  } else
  {
    // Oops, something went wrong, this is usually a connection problem,
    // see the comments at the top of this sketch for the proper connections.


    myGLCD.print("FAIL", CENTER, 15);

  }

  myGLCD.update();
  pinMode(12, OUTPUT);

  //oldtime = millis();
  Serial.begin(115200);
}

unsigned int counter=0;
void loop() {
  // put your main code here, to run repeatedly:
  if ((millis() -oldtime) > (PERIOD *1000 * 60L) || (millis() < oldtime)) {
    
//    Serial.print(oldtime);
//    Serial.print(" / ");
//    Serial.println(counter);
//    counter=0;
    for (int xxx = 0; xxx < 3; xxx++) {

      digitalWrite(12, HIGH);
      delay(10);
      digitalWrite(12, LOW);
      delay(10);
    }

    DHT22_ERROR_t errorCode;
    errorCode = myDHT22.readData();
    double a, P, P0;
    double T180;
    char prStatus;
    myGLCD.clrScr();
    //  myGLCD.drawBitmap(0, 0, dino, 84, 48);
    //myGLCD.update();
    T180 = 0.0;

    delay(3000);
    myGLCD.clrScr();
    char buf[128];
    char buf2[140];
    sprintf(buf, "Temp %hi.%01hi C",
            myDHT22.getTemperatureCInt() / 10, abs(myDHT22.getTemperatureCInt() % 10));
    //Serial.print (buf);
    //Serial.print (",");
    myGLCD.print(buf, 0, 0);
    sprintf(buf, "RH  %i.%01i %%",
            myDHT22.getHumidityInt() / 10, myDHT22.getHumidityInt() % 10);
    //Serial.println (buf);
    myGLCD.print(buf, 0, 10);
    //  sprintf(buf, "Code %d ", errorCode);
    //
    //
    //  myGLCD.print(buf, CENTER, 40);
    prStatus = pr.startTemperature();
    if (prStatus != 0) {
      delay(prStatus);
      prStatus  = pr.getTemperature(T180);
    }
    delay(2000);

    prStatus = pr.startPressure(3);
    //P = getPressure();
    if (prStatus != 0) {
      delay(prStatus);
      prStatus = pr.getPressure(P, T180);
    }


    P0 = pr.sealevel(P, ALTITUDE);

    a = pr.altitude(P, P0);
    sprintf(buf, "P0 = %d.%d mb", int(P), (int(P * 10) % 10));
    myGLCD.print(buf, 0, 20);
    //if(T180>=0) {
    sprintf(buf, "T= %d.%1d C", int(T180), abs(int (T180 * 10) % 10));
    //  //}
    myGLCD.print(buf, 00, 30);

    sprintf(buf, "a= %d.%1d m", int(a), abs(int(a * 10) % 10));

    myGLCD.print(buf, 0, 40);
    digitalWrite(12, HIGH);
    sprintf(buf2, "#%2d.%1d %2d.%1d %04d.%1d %2d.%1d %04d.%1d",
            myDHT22.getTemperatureCInt() / 10, abs(myDHT22.getTemperatureCInt() % 10),
            myDHT22.getHumidityInt() / 10, myDHT22.getHumidityInt() % 10,
            int(P), (int(P * 10) % 10),
            int(T180), abs(int (T180 * 10) % 10),
            int(a), abs(int(a * 10) % 10)
           );

    //myGLCD.print(buf2,0,20);
    delay(100);
    digitalWrite(12, LOW);


    myGLCD.update();

    BT.println(buf2);
    //delay(3000);
  } else {

    //pass.
  }

}

double getPressure()
{
  char status;
  double T, P, p0, a;

  // You must first get a temperature measurement to perform a pressure reading.

  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = pr.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:

    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Use '&T' to provide the address of T to the function.
    // Function returns 1 if successful, 0 if failure.

    status = pr.getTemperature(T);
    if (status != 0)
    {
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pr.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Use '&P' to provide the address of P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pr.getPressure(P, T);
        if (status != 0)
        {
          return (P);
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");
}


