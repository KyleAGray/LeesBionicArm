/*
 * EMG based servo motor control for actuating a prosthetic hand  
 * 
 * Authors: Kyle Gray, Jack Christie
 * 
 * Designed for use with 2 Myoware EMG sensor and XYZ robot servo motor A1-16
 * 
 * Top sensor should be placed near the flexor carpi ulnaris muscles and the
 * bottom sensor should be placed near the brachioradialis muscle for best results
 * 
 * XYZ robot servo library and documentation available at: 
 *  https://github.com/pololu/xyzrobot-servo-arduino
 * 
 * This specific arm requires exactly 4 revolutions of the motor to move
 * from closed to open or vise versa
 * 
 * Notes:
 *  The Myoware sensors initially read in bad values when first powered on
 *  make sure those are not used
 *  
 *  The servo can only read 330 degrees worth of positions ranging between
 *  0 and 1023. For the other 80 degrees, bad values will be read e.g.
 *  lower than 0 or higher than 1023. These values should also be ignored
 *  and a base starting point should be chosen away from this dead zone
 *  such as 500.
  */
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

// Kyle's Arm Values to reduce sensitivity increase values
// or vise versa for increasing sensitivity
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

/// set the hand to open position
bool SetHandOpen = true;
/// set the hand to close position
bool SetHandClosed = false;
/// Did the hand just begin an opening movement
bool openstart = true;
/// Did the hand just begin a closing movement
bool closestart = true;

/// Is hand actively opening
bool opening = false;
/// Is hand actively closing
bool closing = false;

/// Should the revolutions be incremented
bool inc = false;
/// Should the revolutions be decremented
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


  // Set up power for second sensor
  // One sensor can use 5V pin
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

}

void loop()
{
  int top = analogRead(0); // Sensor on top of arm
  int bottom = analogRead(1); // Sensor in bottom of arm
  
  // Used for screwing/unscrewing lead screw
  //      while (1)
  //      {
  //        servo.setSpeed(200);
  //      }

  // Needed to ignore initial values from sensors which are invalid
  startup();
  
  // Dont want to call startup again
  while (1)
  {
    int top = analogRead(0); // Sensor on top of arm
    int bottom = analogRead(2); // Sensor in bottom of arm
    
    // Helpful for determining threshold values
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
        //Serial.println("stop closeing");
        
        servo.setSpeed(0);
        delay(30); // Give time to stop
        pos = servo.readPosition();
        
        // Have we completed a revolution since the last time checked
        if (pos >= 500 && prevPos < 500
            && valid(pos, prevPos))
        {
          rev++;
          //Serial.println("completed close revolution");
          //PrintPos();
        }
        
        // If we go from half closed the opening, and we did not
        // complete exactly a full revolution- which is take care of
        // above- an extra revalution needs to be added 
        if (pos != 500)
        {
          inc = true;
        }

        prevPos = pos;
        closing = false;
        opening = false;
        SetHandClosed = false;
        SetHandOpen = false;

        // Ignore next bit of signals which will likely have many values
        // above the opening threshold
        delay(970); 
      }
      
      else
      {
        
        if ( inc)
        {
          rev++;
          //Serial.println("inc");
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
        //Serial.println("stop opening");
        servo.setSpeed(0);

        delay(30); // allow time to stop
        pos = servo.readPosition();

        // If we have since completed a revolution
        if (pos <= 500 && prevPos > 500
            && valid(pos, prevPos))
        {
          rev--;
          //Serial.println("completed open revolution");
          //PrintPos();

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
        delay(970); // future closing signal temporarily ignored

      }
      else
      {
        if (dec)
        {
          rev--;
          //Serial.println("dec");
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

  if (rev <= 0)
  {
    // should already be stopped, skip the delay
    return;
  }
  // Just begining to open
  if (openstart)
  {
    //Serial.println("open start");
    openstart = false;
    opening = true;

    // check if a revolution completed since last checked
    pos = servo.readPosition();
    if ( prevPos != pos && pos >= 500 && prevPos < 500
         && valid(pos, prevPos) )
    {
      rev++;
      //Serial.println("add revolution");
    }

    prevPos = pos;

    servo.setSpeed(-MAXSPEED);
    delay(100); // Give time for motor to get up to speed
  }

  if (rev <= 0)
  {
    opening = false;
    closing = false;
    servo.setSpeed(0);
    delay(30); // give time to stop
    return;
  }
  
  delay(30); // allow time for motor to move since last reading
  pos = servo.readPosition();

  // Has a revolution completed
  if (pos <= 500 && prevPos > 500
      && valid(pos, prevPos))
  {
    rev--;
    //Serial.println("completed open revolution");
    //PrintPos();

  }

  // All the way open
  if (rev == 0)
  {
    servo.setSpeed(0);
    opening = false;
    
    servo.setPosition(500); // maintain a standard start point
    // Quickly changing motor direction can cause problems, this delay avoids those
    delay(1000);
    pos = servo.readPosition();



  }
  prevPos = pos;
}



void CloseHand()
{
  if (rev >= 4)
  {
    // should already be stopped, skip the delay
    return;
  }
  
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
      //Serial.println("remove revolution");
    }


    prevPos = pos;
    servo.setSpeed(MAXSPEED);
    delay(100); // Give time for motor to get up to speed
    
    //Serial.println("close start");
    
  }
  
  // All the way closed
  if (rev >= 4)
  {
    opening = false;
    closing = false;
    servo.setSpeed(0);
    delay(30);
    return;

  }

  delay(30);
  pos = servo.readPosition();

  // Has revolution completed
  if (pos >= 500 && prevPos < 500
      && valid(pos, prevPos))
  {
    rev++;
    //Serial.println("completed close revolution");
    //PrintPos();
  }
  
  // All the way closed
  if (rev == 4)
  {
    servo.setSpeed(0);
    closing = false;

    servo.setPosition(500); // maintain a standard start point
    // Quickly changing motor direction can cause problems, this delay avoids those
    delay(1000);
    pos = servo.readPosition();
  }
  prevPos = pos;


}

/*
   Called at inial start up, is required because sensors
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
   Print position information, useful for debugging
*/
void PrintPos()
{
  Serial.print(pos);
  Serial.print(" ");
  Serial.print(prevPos);
  Serial.print("\n");
}

/*
 *  Return if two motor position values are considered valid.
 *  
 *  First it checks that they are within the range of readable
 *  values for the servo. 
 *  
 *  Next it checks if the they are far enough away from each other. 
 *  Occasionally the motor does not read the value correctly and cause 
 *  the revolutions to inc/dec prematurely. This always occurs when the 
 *  two positions are read to be very close to each other. With the delays
 *  in place, the motor previous and current motor posions should not be very 
 *  close.
 */
bool valid(int pos1, int pos2 )
{
  return pos1 <= 1023 && pos1 >= 0 && pos2 <= 1023 && pos2 >= 0 && abs(pos1 - pos2) > 10;
}





