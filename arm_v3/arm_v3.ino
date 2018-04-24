#include <XYZrobotServo.h>

// On boards with a hardware serial port available for use, use
// that port. For other boards, create a SoftwareSerial object
// using pin 10 to receive (RX) and pin 11 to transmit (TX).
#ifdef SERIAL_PORT_HARDWARE_OPEN
#define servoSerial SERIAL_PORT_HARDWARE_OPEN
#else
#include <SoftwareSerial.h>
SoftwareSerial servoSerial(10, 11);
#endif


// Max Revolutions
#define REVMAX  4

// Kyle's Arm Values 125, 400
#define CLOSE_THRESH 200
#define OPEN_THRESH 600

// Pins
#define l_sensor_power 13

// Max motor speed
#define MAXSPEED 1023

/******* GLOBALS *******/
/// Current position of the motor
uint16_t pos = 500;
/// Previous position of the motor
uint16_t prevPos = 500;
/*
   current number of completed revolutions
   completely closed: rev = 4
   completely open: rev = 0
*/
int rev = 0;

bool SetHandOpen = true;
bool SetHandClosed = false;

bool openstart = true;
bool closestart = true;

/// Is hand actively opening
bool opening = false;
/// Is hand actively closing
bool closing = false;

bool inc = false;
bool dec = false;
// Set up a servo object, specifying what serial port to use and
// what ID number to use.
XYZrobotServo servo(servoSerial, 1);

/**********************/


void setup()
{
  // Turn on the serial port and set its baud rate.
  Serial.begin(57600);
  servoSerial.begin(57600);
  // Start position
  servo.setPosition(500);
  // Allow motor time to get to start position


  // Set up power for sensor
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

}

void loop()
{
  int top = analogRead(0); // Sensor on top of arm
  int bottom = analogRead(1); // Sensor in bottom of arm
  //used for screwing on lead screw
//      while (1)
//      {
//        servo.setSpeed(200);
//      }
  startup();
  // Dont want to call startup again

  while (1)
  {
    int top = analogRead(0); // Sensor on top of arm
    int bottom = analogRead(2); // Sensor in bottom of arm
    //    Serial.print(top);
    //    Serial.print(" ");
    //    Serial.print(bottom);
    //    Serial.print("\n");
    //    delay(10);


    // Opening hand detected
    if (top > OPEN_THRESH)
    {
      SetHandOpen = true;
      SetHandClosed = false;
      //Serial.println("OPEN TRIGGERED");
    }
    // Closing hand detected
    else if ( bottom > CLOSE_THRESH)
    {

      SetHandClosed = true;
      SetHandOpen = false;
      //Serial.println("CLOSE TRIGGERED");
    }


    if (SetHandOpen) {
      // Alow the user to stop arm mid-movement
      if (closing)
      {
        Serial.println("stop closeing");
        //rev++;

        servo.setSpeed(0);

        delay(30);
        pos = servo.readPosition();
        if (pos >= 500 && prevPos < 500
            && valid(pos, prevPos))
        {
          rev++;
          Serial.println("completed close revolution");
          PrintPos();
        }
        if (pos != 500)
        {
          inc = true;
        }

        prevPos = pos;
        closing = false;
        opening = false;
        SetHandClosed = false;
        SetHandOpen = false;

        delay(970);
      }
      else
      {
        if ( inc)
        {
          rev++;
          Serial.println("inc");
        }
        dec = false;
        inc = false;
        closestart = true;
        OpenHand();
      }
    }

    else if (SetHandClosed) {

      if (opening)
      {
        Serial.println("stop opening");
        //rev--;

        servo.setSpeed(0);

        delay(30);
        pos = servo.readPosition();

        // If we have since completed a revolution
        if (pos <= 500 && prevPos > 500
            && valid(pos, prevPos))
        {
          rev--;
          Serial.println("completed open revolution");
          PrintPos();

        }

        
        if (pos != 500)
        {
          dec = true;
        }


        opening = false;
        closing = false;
        SetHandOpen = false;
        SetHandClosed = false;

        prevPos = pos;
        delay(970);

      }
      else
      {
        if (dec)
        {
          rev--;
          Serial.println("dec");
        }
        inc = false;
        dec = false;
        openstart = true;
        CloseHand();
      }
    }

  }

}

void OpenHand()
{


  // Just begining to open
  if (openstart)
  {
    Serial.println("open start");
    openstart = false;
    opening = true;

    // check if a revolution completed since last checked
    pos = servo.readPosition();
    if ( prevPos != pos && pos >= 500 && prevPos < 500
         && valid(pos, prevPos) )
    {
      rev++;
      Serial.println("add revolution");
    }

    prevPos = pos;


  }

  if (rev <= 0)
  {
    opening = false;
    closing = false;
    servo.setSpeed(0);
    delay(30);
    return;
  }

  else
  {
    servo.setSpeed(-MAXSPEED);
  }
  delay(30);
  pos = servo.readPosition();

  if (pos <= 500 && prevPos > 500
      && valid(pos, prevPos))
  {
    rev--;
    Serial.println("completed open revolution");
    PrintPos();

  }

  if (rev == 0)
  {
    servo.setSpeed(0);
    opening = false;

    // more than likely we've overshot (hopefully by a position or two)
    //    while (servo.readPosition() != 500)
    //    {
    //      servo.setPosition(500);
    //      delay(10);
    //    }
    servo.setPosition(500);
    delay(1000);
    pos = servo.readPosition();



  }
  prevPos = pos;
}



void CloseHand()
{

  // Just begining to open
  if (closestart)
  {
    closestart = false;
    closing = true;

    // check if a revolution completed since last checked
    pos = servo.readPosition();

    if ( prevPos != pos && pos <= 500 && prevPos > 500
         && valid(pos, prevPos))
    {
      rev--;
      Serial.println("remove revolution");
    }


    prevPos = pos;
    Serial.println("close start");

  }
  if (rev >= 4)
  {
    opening = false;
    closing = false;
    servo.setSpeed(0);
    delay(30);
    return;

  }
  else
  {
    servo.setSpeed(MAXSPEED);
  }

  delay(30);
  pos = servo.readPosition();
  if (pos >= 500 && prevPos < 500
      && valid(pos, prevPos))
  {
    rev++;
    Serial.println("completed close revolution");
    PrintPos();
  }
  if (rev == 4)
  {
    servo.setSpeed(0);
    closing = false;
    // We've overshot (hopefully by a position or two)
    //    while (servo.readPosition() != 500)
    //    {
    //      servo.setPosition(500);
    //      delay(10);
    //    }

    servo.setPosition(500);
    delay(1000);
    pos = servo.readPosition();
  }
  prevPos = pos;


}

/*
   Called at inial start up, is required because analog read
   initially reads in garbage values, this allows us to skip those
*/
void startup()
{
  // Delays at lead 10 seconds
  for (int i = 0; i < 10000; i++)
  {
    int x = analogRead(2); //bottom
    int y = analogRead(0); //top
    delay(1);
  }
  Serial.println("Done");
}

/*
   Print position information
*/
void PrintPos()
{
  Serial.print(pos);
  Serial.print(" ");
  Serial.print(prevPos);
  Serial.print("\n");
}

bool valid(int value1, int value2 )
{
  return value1 <= 1023 && value1 >= 0 && value2 <= 1023 && value2 >= 0 && abs(value1 - value2) > 10;
}

