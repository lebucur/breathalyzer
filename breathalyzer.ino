/*     PROJECT: Breathalyzer
 *
 * DESCRIPTION: This is the program for an Arduino breathalyzer.
 *              It's main purpose is to discourage drunk driving.
 *
 *      AUTHOR: Loredan Emilio Bucur (lebucur)
 *
 *     LICENSE: This program is free software: you can redistribute it and/or modify
 *              it under the terms of the GNU General Public License as published by
 *              the Free Software Foundation, either version 3 of the License, or
 *              (at your option) any later version.
 *
 *              This program is distributed in the hope that it will be useful,
 *              but WITHOUT ANY WARRANTY; without even the implied warranty of
 *              MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *              GNU General Public License for more details.
 *
 *   LAST_EDIT: June 2016
 */

//TODO
//fix Serial 115200 gsm debug mode
//cine mananca atata memorie
//use last_update sau fix_age sa nu setez aiurea pe 0
//phone calls - intrusive but effective
//remember to flush/read gsm buffer; same with gps?
//inline funcs all?

//NOTES
//sometimes a getter just fits better

//DEFINE
#define DEBUG 1
#define MEAN(a, b) ((a + b) / 2);
#define CRASH_G 2 //20-40 usual values
#define FAIL -1
#define TASK1 1
#define TASK2 2
#define TASK3 4

//INCLUDE
#include "Debug.h" //free HW serial when not needed
#include <SoftwareSerial.h>
#include <Adafruit_FONA.h>
#include <TinyGPS.h>
#include <ADXL335.h>

//CONSTANTS
struct {
    static const byte mq3 = A6;
    static const byte mq3_ref = A7;
    
    static const byte led = 13;
    
    static const byte acc_x = A1;
    static const byte acc_y = A2;
    static const byte acc_z = A3;
    
    static const byte gsm_pwr = 3;
    static const byte gsm_tx = 4;
    static const byte gsm_rx = 5;
    
    static const byte gps_tx = 8;
    static const byte gps_rx = 9;
    static const byte gps_en = 6;
    static const byte gps_fix = 7;
} Pins;

SoftwareSerial gps_ser = SoftwareSerial(Pins.gps_tx, Pins.gps_rx);
SoftwareSerial gsm_ser = SoftwareSerial(Pins.gsm_tx, Pins.gsm_rx);
Adafruit_FONA gsm = Adafruit_FONA(Pins.gsm_pwr);
ADXL335 accel(Pins.acc_x, Pins.acc_y, Pins.acc_z, 3.3); //3.3V is analog reference
TinyGPS gps;

//GLOBALS
byte active_tasks;
unsigned sen, sen_ref; //0..1023 ADC sensors
unsigned diff_stable, diff;
unsigned long difp;

struct Task {
    unsigned long interval, last_time;
} Task1, Task2, Task3;

struct GSM_data {
    char contact[11] = "0740000000";
    //IMEI, APN
    //latitude, longitude, datetime
} GSM;

struct GPS_data {
    int year;
    byte month, day, hour, minute, second;
    long latitude, longitude, altitude;
    float speed;
    unsigned long fix_age, time, date, course, last_update;
    const char * cardinal;
} GPS;

void setup()
{
    DBG_begin(9600); //9600 for gsm debug
    DBG_printsln("SETUP");
    
    gps_ser.begin(9600);

    //declare output/input_pullup pins if any
    pinMode(Pins.led, OUTPUT);
    pinMode(Pins.mq3, INPUT_PULLUP); //an alert should be generated
    pinMode(Pins.mq3_ref, INPUT_PULLUP); //if sensors are disconnected
    pinMode(Pins.gsm_pwr, OUTPUT);
    pinMode(Pins.gps_en, OUTPUT);

    //power on the GSM module
    digitalWrite(Pins.gps_en, HIGH);
    gsm_ser.begin(9600);
    gsm.begin(gsm_ser);
    DBG_prints("Battery: ");
    DBG_print(get_battery());
    DBG_printsln("%");
    
    Task1.interval = 1000;
    Task2.interval = 1000;
    Task3.interval = 5000;
    //active_tasks |= TASK1;
    //active_tasks |= TASK2;
    //active_tasks |= TASK3;
    
    print_commands();
}

void loop()
{
    unsigned long current_time = millis();
    
    //TASK1 ACCELERATION
    if ((active_tasks & TASK1) && (current_time - Task1.last_time > Task1.interval)) {
        if (get_acceleration()) {
            report_accident();
        }
        Task1.last_time = current_time;
        current_time = millis();
    }
    
    //TASK2 MAIN
    if ((active_tasks & TASK2) && (current_time - Task2.last_time > Task2.interval)) {
        read_sensors();
        if (process_data()) {
            report_dui();
        }
        Task2.last_time = current_time;
        current_time = millis();
    }
    
    //TASK3 LOCATION
    if ((active_tasks & TASK3) && (current_time - Task3.last_time > Task3.interval)) {
        invoke_gps();
        Task3.last_time = current_time;
        //current_time = millis();
    }
}

boolean get_acceleration()
{
    accel.update();
    if (accel.getRho() > CRASH_G) {
        DBG_println(accel.getRho());
        return true;
    }
    return false;
}    

unsigned get_battery()
{
    unsigned vbat;
    if (gsm.getBattPercent(&vbat)) {
        return vbat;
    }
    return 57341;
}

void read_sensors()
{
    sen = analogRead(Pins.mq3);
    delay(1);
    sen = MEAN(sen, analogRead(Pins.mq3)); //average readings; LPF
    delay(1);
    sen_ref = analogRead(Pins.mq3_ref);
    delay(1);
    sen_ref = MEAN(sen_ref, analogRead(Pins.mq3_ref));
    delay(1);
}

boolean process_data()
{
    diff = sen > sen_ref ? sen - sen_ref : 0; //unsigned for now..
    difp = diff > diff_stable ? diff - diff_stable : 0;
    difp *= difp;
    diff_stable = 0.9 * diff_stable + 0.1 * diff; //testez diferite valori
    /*
    DBG_prints("sen:");
    DBG_print(sen);
    DBG_prints(" ref:");
    DBG_print(sen_ref);
    DBG_prints(" diff_stable:");
    DBG_print(diff_stable);
    DBG_prints(" diff:");
    DBG_print(diff);
    DBG_prints(" difp:");
    DBG_println(difp);
    */
    return difp > 1000;
}

void report_dui()
{
    DBG_printsln("Taking action:");
    //connect gsm
    char message[141];
    sprintf(message, "Hello! You have a drunk driver at %l lat, %l lon.", GPS.latitude, GPS.longitude);
    send_sms(message);
}

void report_accident()
{
    DBG_printsln("Reporting crash:");
    char message[141];
    sprintf(message, "Hello! Your car crashed at %l lat, %l lon.", GPS.latitude, GPS.longitude);
    send_sms(message);
}

bool send_sms(char * message)
{
    unsigned char len = strlen(message);
    message[len] = 26; //Ctrl-Z SUB Ascii character
    if (gsm.sendSMS(GSM.contact, message)) {
        DBG_prints("SMS sent to "); DBG_print(GSM.contact);
        DBG_prints(": "); DBG_println(message);
        return true;
    }
    else {
        DBG_printsln("SMS failed to send");
        return false;
    }
}

void invoke_gps() //takes over a second
{
    gps_ser.listen(); //shared with gsm softwareserial
    unsigned long start_time = millis();
    do {
        if (gps_ser.available()) {
            char c = gps_ser.read();
            if (gps.encode(c)) { //if done processing a sentence
                gps.get_position(&GPS.latitude, &GPS.longitude); //+/- lat/long in degrees
                gps.get_datetime(&GPS.date, &GPS.time, &GPS.fix_age); //time in hhmmsscc, date in ddmmyy
                GPS.speed = gps.f_speed_kmph();
                GPS.altitude = gps.altitude();
                GPS.course = gps.f_course(); // course in 100ths of a degree
                GPS.cardinal = gps.cardinal(GPS.course);
                gps.crack_datetime(&GPS.year, &GPS.month, &GPS.day, &GPS.hour, &GPS.minute, &GPS.second);
            }
        }
    } while (millis() - start_time < 1000);
    /*
    DBG_prints("age:"); DBG_print(GPS.fix_age);
    DBG_prints(" lat:"); DBG_print(GPS.latitude);
    DBG_prints(" lon:"); DBG_print(GPS.longitude);
    DBG_prints(" speed:"); DBG_print(GPS.speed);
    DBG_prints(" course:"); DBG_print(GPS.course);
    DBG_prints(" date:"); DBG_print(GPS.date);
    DBG_prints(" time:"); DBG_println(GPS.time);
    DBG_prints("year:"); DBG_print(GPS.year);    
    DBG_prints(" month:"); DBG_print(GPS.month);
    DBG_prints(" day:"); DBG_print(GPS.day);
    DBG_prints(" hour:"); DBG_print(GPS.hour);
    DBG_prints(" minute:"); DBG_print(GPS.minute);
    DBG_prints(" second:"); DBG_println(GPS.second);
    */
}

void gps_debug_mode()
{
    gps_ser.listen();
#if DEBUG
    DBG_printsln("\n** GPS debug mode started **");
	char c = 100;
	do {
		if (gps_ser.available()) {
			Serial.write(gps_ser.read());
		}
		if (Serial.available()) {
			c = Serial.read();
            Serial.write(c);
			gps_ser.write(c);
		}
	} while (c != '`');
	DBG_printsln("\n** Exit debug mode **");
    print_commands();
#endif
}

void gsm_debug_mode()
{ //here the execution stops @ gsm_debug() @ serialEvent() @ loop()
    gsm_ser.listen();
#if DEBUG
	DBG_printsln("\n** GSM debug mode started **");
	char c = 100;
	do {
		if (gsm_ser.available()) {
			Serial.print(gsm_ser.read());
		}
		if (Serial.available()) {
			c = Serial.read();
            Serial.print(c);
			gsm_ser.print(c);
		}
	} while (c != '`');
	DBG_printsln("\n** Exit debug mode **");
    print_commands();
#endif
}

bool is_printable(char c)
{
    if (c >= 32) { //that's where printable chars begin in ASCII table
        return true;
    }
    return false;
}

bool is_figure(char c) {
    if (c >= '0' && c <= '9') {
        return true;
    }
    return false;
}

void print_commands()
{
    DBG_printsln("Commands:");
    DBG_printsln("  g: enter GSM debug mode");
    DBG_printsln("  p: enter GPS debug mode");
    DBG_printsln("  m: show free RAM");
    DBG_printsln("  n: change contact phone number");
    DBG_printsln("  s: send a test SMS");
    DBG_printsln("  z: toogle main task");
}

void serialEvent()
{ //called at the end of each loop() execution if new bytes arrive in RX
  //it takes a little while until it is executed again
#if DEBUG
    switch (Serial.read()) //one ASCII character command
    {
        case 'g':
            gsm_debug_mode();
        break;
        
        case 'p':
            gps_debug_mode();
        break;
            
        case 'm':
            DBG_prints("Free RAM: ");
            DBG_print(get_free_ram());
            DBG_printsln(" bytes");
        break;
        
        case 'n':
            while (Serial.available()) Serial.read(); //flush serial
            DBG_prints("New contact phone number: ");
            for (int i = 0; i < 10; i++) {
                while (!Serial.available()); //wait serial
                char f = Serial.read();
                if (is_figure(f)) {
                    GSM.contact[i] = f;
                    DBG_print(f);
                }
            }
            DBG_println();
        break;
        
        case 's':
            send_sms(GSM.contact);
        break;
        
        case 'z':
            active_tasks ^= TASK2;
            if (active_tasks & TASK2) {
                DBG_printsln("Main task activated.");
            }
            else {
                DBG_printsln("Main task deactivated.");
            }
        break;
        
        default:
            print_commands();
        break;
    }
#endif
}