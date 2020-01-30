// the only problem remaining (so far) is the one minute
// time stamp code occurs about every 26 ish seconds
// this means your loop count timing scheme blows
// so go fix dat

// mux goes like this:
// y0-y3 are ref_VCC inputs, y out goes to adc 1
// x0-x3 are ref_mv inputs, x out goes to adc2

//<debugging defines>
#define _DEBUG_ // for serial debugging
//#undef    _DEBUG_
//</debuging defines>
#define _TIMESTAMP_PER_MINUTE_
//#undef _TIMESTAMP_PER_MINUTE_
#define _TIMESTAMP_PER_POWERUP_
#undef _TIMESTAMP_PER_POWERUP_

//<constant defines>

//</constant defines>
//*****************************************************************************************************************
//<pin defines>
#define HEAT_1 6 // digital input
#define HEAT_2 7 // digital input
#define HEAT_3 8 // digital input
#define HEAT_4 9 // digital input
#define REDLED 3
#define GREENLED 4
#define DEBUG_PULSE_OUT 5
#define SPI_CHIPSEL 10
//
// the goal with the two adc inputs having different gain elements driving them
// is to scale the two forms of input to the 2.50v analog reference voltage.
// the two forms are sensors with outputs in the 0 to 1250 millivolts range(1250 mv x 2 = 2.50v)
// and sensors with outputs in the 0 to 5 volts range (5v / 2 = 2.50v)
// using a 2.50 volt analog reference voltage allows the input analog circuits
// to be used for either 5 volt or 3.3 volt microcontrollers with
// no analog circuit changes needed.
//

#define REF_VCC_IN A1 // op amp buffer with Vg X1 and 2:1 voltage divider output
#define REF_MV_IN A2  // op amp buffer with Vg x2 output

#define MUXADDRA A0
#define MUXADDRB A3


//</pin defines>
//******************************************************************************************************************
// <input_mux_addr defines>
// same adresses / first half of the mux (CD4052 X inputs)
#define REF_MV_1 0
#define REF_MV_2 1
#define REF_MV_3 2
#define REF_MV_4 3

// same adresses / second half of the mux (CD4052 Y inputs)
#define REF_VCC_1 0
#define REF_VCC_2 1
#define REF_VCC_3 2
#define REF_VCC_4 3
// </input_mux_addr defines>

//#define REF_VCC_SCALE 5000 // for 5V vcc
#define REF_VCC_SCALE 3300 // for 3.3V vcc

#define REF_MV_SCALE 1250

// time per frame not counting overhead
// in serial ouput and sd writes
// 5 msec ends up being about 270 msec per frame
// 3 msec is about 165 msec per frame
// 1 msec is about 105 msec per frame
// 500 usec is about 87 msec per frame
// overhead is therefore about 83 msec
#define SAMPLE_INTERVAL 250 // x 4 = 2 msec / frame
#define AVG_DELAY delayMicroseconds

//#define SAMPLE_INTERVAL 1 // x 4 = 4 msec / frame
//#define AVG_DELAY delay

#include <Wire.h>
#include <stdio.h>
#include <SPI.h>
#include <SD.h>
#include "RTClib.h"

RTC_PCF8523 rtc;
//char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

unsigned long startmillis = 0;
unsigned long currentmillis = 0;

void set_input_mux_addr(int input_mux_addr);
int adcaverage(int input_mux_addr, int adc_channel, int adc_scale);

//  used to timestamp output to file every minute
#ifdef _TIMESTAMP_PER_MINUTE_
int loopcount = 0;
#endif

//  cleared first trip through the loop
//  used to time stamp startup.
unsigned char powerup = 1;

/*
long mymap(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min;
}
*/

void setup()
{
    // put your setup code here, to run once:
    //
    // first set adc reference to external.
    // 2.50 Volt reference used , we can use 3.3v or
    // 5v microcontrollers with such a reference voltage
    analogReference(DEFAULT);

    digitalWrite(MUXADDRA, LOW);
    digitalWrite(MUXADDRB, LOW);
    pinMode(MUXADDRA, OUTPUT);
    pinMode(MUXADDRB, OUTPUT);
    // start with status leds LOW
    digitalWrite(REDLED, LOW);
    digitalWrite(GREENLED, LOW);
    pinMode(REDLED, OUTPUT);
    pinMode(GREENLED, OUTPUT);
    digitalWrite(DEBUG_PULSE_OUT, LOW);
    pinMode(DEBUG_PULSE_OUT, OUTPUT);
    // these are 12v driven but clamped to logic high
    // and they have pulldowns to logic low (i.e. ground)
    // used for o2 sensor heater circuit monitoring
    pinMode(HEAT_1, INPUT);
    pinMode(HEAT_2, INPUT);
    pinMode(HEAT_3, INPUT);
    pinMode(HEAT_4, INPUT);
    // SPI chip select for the SD card subsystem
    digitalWrite(SPI_CHIPSEL, HIGH);
    pinMode(SPI_CHIPSEL, OUTPUT);
    // unused pins set low
    digitalWrite(A4, LOW);
    pinMode(A4, OUTPUT);

#ifdef _DEBUG_
    Serial.begin(115200);
#endif
    // real time clock check
    Wire.begin();
    if (!rtc.begin())
    {
        digitalWrite(REDLED, HIGH);
#ifdef _DEBUG_
        Serial.println("Couldn't find RTC");
#endif
        while (1) // no clock, die here.
            delay(250);
        digitalWrite(REDLED, !REDLED);
        ;
    }

//delay(20); // a short delay to let things stabilize
#ifdef _DEBUG_

    Serial.print("Initializing SD card...");
#endif
    // see if the card is present and can be initialized:
    if (!SD.begin(SPI_CHIPSEL))
    {
#ifdef _DEBUG_
        Serial.println("Card failed, or not present");
// don't do anything more:
#endif
        digitalWrite(REDLED, HIGH);
        while (1)
            ; // hang till power down and card inserted
    }
#ifdef _DEBUG_
    Serial.println("card initialized.");
#endif
    String StartString = "";
    StartString += "Startup";

    //we made it!
    digitalWrite(REDLED, LOW);    // no errors
    digitalWrite(GREENLED, HIGH); // set leds for GO
    startmillis = millis();
}

void loop()
{
    // put your main code here, to run repeatedly:

    String dataString = "";
    String timeString = "";
    currentmillis = millis();

    //*******************************************************************************
    //                      loopcount timing is of by 2 to 1 low
    //*******************************************************************************

    dataString += String(millis() - startmillis);
    dataString += String(",");

    // <et heater input data>
    dataString += String(digitalRead(HEAT_1));
    dataString += String(",");
    dataString += String(digitalRead(HEAT_2));
    dataString += String(",");
    dataString += String(digitalRead(HEAT_3));
    dataString += String(",");
    dataString += String(digitalRead(HEAT_4));
    dataString += String(",");
    // this will be the end string for a given sample
    // frame so no comma separator
    // heatString += String(",");

    // </get heater input data>

    // get 4 samples and then average them
    //<get REF_MV sensor data (O2 sensors most common)>

    // input values are input_mux_addr, adc ref_mv input and  adc scale value
    // read  CD4052 X inputs for ref_mv values
    dataString += String(adcaverage(REF_MV_1, REF_MV_IN, REF_MV_SCALE));
    dataString += String(",");

    dataString += String(adcaverage(REF_MV_2, REF_MV_IN, REF_MV_SCALE));
    dataString += String(",");

    dataString += String(adcaverage(REF_MV_3, REF_MV_IN, REF_MV_SCALE));
    dataString += String(",");

    dataString += String(adcaverage(REF_MV_4, REF_MV_IN, REF_MV_SCALE));
    dataString += String(",");
    //</get REF_MV sensor data>

    // <get REF_VCC sensors (throttle pos. MAF, etc,)>
    // input values are input_mux_addr, adc ref_vcc input and adc scale value
    // read  CD4052 y inputs for ref_vcc values
    dataString += String(adcaverage(REF_VCC_1, REF_VCC_IN, REF_VCC_SCALE));
    dataString += String(",");

    dataString += String(adcaverage(REF_VCC_2, REF_VCC_IN, REF_VCC_SCALE));
    dataString += String(",");

    dataString += String(adcaverage(REF_VCC_3, REF_VCC_IN, REF_VCC_SCALE));
    dataString += String(",");

    dataString += String(adcaverage(REF_VCC_4, REF_VCC_IN, REF_VCC_SCALE)); // last channel no comma appended

    // </get REF_VCC sensors>

    // <SD card setup>

    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    File dataFile = SD.open("datalog.csv", FILE_WRITE);

    // if the file is available, write to it:
    if (dataFile)
    {
#ifdef _TIMESTAMP_PER_MINUTE_
        if (loopcount == 0)
        {
            digitalWrite(GREENLED, !digitalRead(GREENLED));
            DateTime now = rtc.now();
            timeString += now.unixtime();
            dataFile.println(timeString);
            Serial.println(timeString);
            timeString = "";
        }

#endif
#ifdef _TIMESTAMP_PER_POWERUP_
        powerup = 0;
        DateTime now = rtc.now();
        timeString += now.unixtime();
        dataFile.println(timeString);
        Serial.println(timeString);
        timeString = "";

#endif

        dataFile.println(dataString);
        dataFile.close();

#ifdef _DEBUG_
        // print to the serial port too:
        Serial.println(dataString);
#endif
    }

    // if the file isn't open, pop up an error:
    else
    {
        digitalWrite(REDLED, HIGH);
        digitalWrite(GREENLED, LOW);
#ifdef _DEBUG_
        Serial.println("error opening datalog.csv");
#endif
    }
    //*******************************************************************************
    //                      loopcount timing is of by 2 to 1 low
    //                      probably because this loop delay is bad
    //*******************************************************************************

    //*******************************************************************************
    //                      loopcount timing is LOW by 2 to 1 low
    //*******************************************************************************

    while (millis() - currentmillis < 100)
        ; // do every 100 millis aka 10 sample / sec.
// debugging the timing setup, minute counter is fast by 2:1
// a nice 10 herz square wave, we hope
// digitalWrite (DEBUG_PULSE_OUT, !digitalRead(DEBUG_PULSE_OUT));

#ifdef _TIMESTAMP_PER_MINUTE_
    if (loopcount <= 600)
    {
        loopcount++;
    }
    else
    {
        loopcount = 0; //reset each minute
                       // see beginning of loop for the usage of this}
        digitalWrite (DEBUG_PULSE_OUT, !digitalRead(DEBUG_PULSE_OUT));
    }
#endif
}

int adcaverage(int input_mux_addr, int adc_channel, int adc_scale)
{

    set_input_mux_addr(input_mux_addr);

    // sum 4 readings with some time before each
    // sample to allow mux address and output settling
    // to help suppress noise, divide by 4 for result
    // division is done by shifting.

    int temp = 0;

    AVG_DELAY(SAMPLE_INTERVAL);
    temp += analogRead(adc_channel);
    AVG_DELAY(SAMPLE_INTERVAL);
    temp += analogRead(adc_channel);
    AVG_DELAY(SAMPLE_INTERVAL);
    temp += analogRead(adc_channel);
    AVG_DELAY(SAMPLE_INTERVAL);
    temp += analogRead(adc_channel);
    temp /= 4;
    return map(temp, 0, 1023, 0, adc_scale);
}

void set_input_mux_addr(int input_mux_addr)
{

    if (input_mux_addr & 0b00000001)
    {
        digitalWrite(MUXADDRA, HIGH);
    }
    else
        digitalWrite(MUXADDRA, LOW);

    if (input_mux_addr & 0b00000010)
    {
        digitalWrite(MUXADDRB, HIGH);
    }
    else
        digitalWrite(MUXADDRB, LOW);
}
