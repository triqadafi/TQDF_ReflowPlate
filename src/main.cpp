#include <Arduino.h>

// LCD
#include <Wire.h>                   //Included by Arduino IDE
#include <LiquidCrystal_I2C.h>      //Downlaod it here: http://electronoobs.com/eng_arduino_liq_crystal.php
LiquidCrystal_I2C lcd(0x27,16,2);   //Define LCD address as 0x27. Also try 0x3f if it doesn't work. 

// Thermistor 
#include <thermistor.h>
thermistor therm1(A0,0);  // Analog Pin which is connected to the 3950 temperature sensor, and 0 represents TEMP_SENSOR_0 (see configuration.h for more information).

// RTD
#include <Adafruit_MAX31865.h>
// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 thermo = Adafruit_MAX31865(2, 7, 8, 4);
// use hardware SPI, just pass in the CS pin
//Adafruit_MAX31865 thermo = Adafruit_MAX31865(10);

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0


// PARAMETERS
float TQDF_CYCLE_preheat_setpoint = 150;
float TQDF_CYCLE_soak_setpoint = 200;
float TQDF_CYCLE_reflow_setpoint = 210;
float TQDF_CYCLE_cooldown_setpoint = 60;

unsigned long TQDF_CYCLE_preheat_duration = 30;
unsigned long TQDF_CYCLE_reflow_duration = 20;

// GPIO
int PIN_BUTTON_start = 12;
int PIN_BUTTON_increment = 11;
int PIN_BUTTON_decrement = 10; 
int PIN_BUTTON_stop = 9;
int PIN_SSR = 3;
int PIN_BUZZER = 6;
int PIN_THERMISTOR = A0;

// FSM Variable
unsigned long FSM_print_millis = 0;
unsigned long FSM_millis = 0;
int FSM_state = 0;
int FSM_MAIN_mode = 0;
int FSM_MAIN_tone = 0;
int FSM_HEATING_state = 0;

// FSM Cycle
int FSM_CYCLE_state = 0;
int FSM_CYCLE_second = 0;
float FSM_CYCLE_temperature = 0;
unsigned long FSM_CYCLE_millis = 0;
unsigned long FSM_CYCLE_second_millis = 0;

// PID Variables
unsigned long PID_millis = 0;
float PID_period = 50;

float Kp = 25;
float Ki = 0.0025;
float Kd = 100.0;

float PID_output = 0;
float PID_P, PID_I, PID_D;
float PID_error, PID_error_last;

float PID_setpoint = 0;                        //Used for PID control
float PID_pwm = 255;                          //The SSR is OFF with HIGH, so 255 PWM would turn OFF the SSR
float PID_PWM_min = 0;
float PID_PWM_max = 255;                      //Max PID value. You can change this. 


// Sensor
float SENSOR_rtd_celcius = 0;
float SENSOR_thermistor_celcius = 0;
unsigned long SENSOR_period = 25;
unsigned long SENSOR_millis = 0;

float MAIN_temperature = 0;

// Sensor Function
void SENSOR_begin();
void SENSOR_loop();
float SENSOR_read_thermistor();
float SENSOR_read_rtd();

// PID
void PID_loop(float setpoint, int temperature);

// FSM Cycle
void CYCLE_run_blocking(const char text[], float setpoint, int duration);
void CYCLE_cooling_blocking();

// Helper
void printParameters();
void deadLock();


void setup() {
  //Define the pins as outputs or inputs
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_SSR, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT); 
  pinMode(PIN_BUTTON_start, INPUT_PULLUP);
  pinMode(PIN_BUTTON_increment, INPUT_PULLUP);
  pinMode(PIN_BUTTON_decrement, INPUT_PULLUP);
  pinMode(PIN_BUTTON_stop, INPUT_PULLUP);
  pinMode(PIN_THERMISTOR, INPUT);

  digitalWrite(PIN_SSR, HIGH);        //Make sure we start with the SSR OFF (is off with HIGH)
  digitalWrite(PIN_BUZZER, LOW);  

  lcd.init();                     //Init the LCD
  lcd.backlight();              //Activate backlight   
  Serial.begin(115200);

  SENSOR_begin();


  FSM_state = 1;
}

void loop() {
  if(FSM_state == 1){
    tone(PIN_BUZZER, 1800, 200);

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("T: ");
    lcd.setCursor(15,0);
    lcd.print(FSM_MAIN_tone);
    lcd.setCursor(0,1);
    lcd.print("Select Mode     ");
    FSM_MAIN_mode = 0;
    while (1){
      SENSOR_loop();
      MAIN_temperature = SENSOR_rtd_celcius;

      if(millis() - FSM_millis > 1000){
        lcd.setCursor(2, 0);
        lcd.print(MAIN_temperature, 1);

        FSM_millis = millis();
      }

      if(digitalRead(PIN_BUTTON_increment) == LOW){
        FSM_MAIN_mode++;
        FSM_MAIN_mode = (FSM_MAIN_mode % 4);
        FSM_MAIN_mode = FSM_MAIN_mode==0 ? 1 : FSM_MAIN_mode;

        lcd.setCursor(0,1);
        lcd.print("MODE            ");
        lcd.setCursor(5,1);
        lcd.print(FSM_MAIN_mode);
        delay(200);
      }

      if(digitalRead(PIN_BUTTON_stop) == LOW){
        switch (FSM_MAIN_tone){
        case 0:
          tone(PIN_BUZZER, 2000, 150);
          delay(130);
          tone(PIN_BUZZER, 2200, 150);
          delay(130);
          tone(PIN_BUZZER, 2400, 150);
          break;
        case 1:
          tone(PIN_BUZZER, 2500, 150);
          delay(130);
          tone(PIN_BUZZER, 2200, 150);
          delay(130);
          tone(PIN_BUZZER, 2000, 150);
          break;
        case 2:
          tone(PIN_BUZZER, 2300, 40);  
          break;
        case 3:
          tone(PIN_BUZZER, 1000, 100); 
          break;
        case 4:
          tone(PIN_BUZZER, 1800, 1000);   
          break;
        }
        FSM_MAIN_tone++;
        FSM_MAIN_tone = FSM_MAIN_tone % 5; // 0-2

        lcd.setCursor(15,0);
        lcd.print(FSM_MAIN_tone);
        delay(2000);
      }

      if(FSM_MAIN_mode > 0 && digitalRead(PIN_BUTTON_start) == LOW){
        switch (FSM_MAIN_mode){
          default: FSM_state = 1; break;
          case 1: FSM_state = 2; break;
          case 2: FSM_state = 3; break;
          case 3: FSM_state = 4; break;
        }
        
        tone(PIN_BUZZER, 2000, 150);
        delay(130);
        tone(PIN_BUZZER, 2200, 150);
        delay(130);
        tone(PIN_BUZZER, 2400, 150);

        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Preparing...    ");
        lcd.setCursor(0,1);
        lcd.print("MODE            ");
        lcd.setCursor(5,1);
        lcd.print(FSM_MAIN_mode);

        delay(1000);
        break;
      }
    }
    
  }
  if(FSM_state == 2){
    FSM_HEATING_state = 1;
    while(1){
      if(FSM_HEATING_state == 1){
        CYCLE_run_blocking("Preaheating...", TQDF_CYCLE_preheat_setpoint, TQDF_CYCLE_preheat_duration);
        FSM_HEATING_state = 2;
      }
      if(FSM_HEATING_state == 2){
        tone(PIN_BUZZER, 1800, 200);     
        CYCLE_run_blocking("Reflowing...", TQDF_CYCLE_reflow_setpoint, TQDF_CYCLE_reflow_duration);
        FSM_HEATING_state = 3;
      }
      if(FSM_HEATING_state == 3){
        CYCLE_cooling_blocking();
        FSM_HEATING_state = 4;
      }
      if(FSM_HEATING_state == 4){
        break;
      }
    }
    FSM_state = 1;
  }

  if(FSM_state == 3){
    FSM_HEATING_state = 1;
    while(1){
      if(FSM_HEATING_state == 1){
        while(1){
          if(FSM_HEATING_state == 1){
            CYCLE_run_blocking("Soaking...", TQDF_CYCLE_soak_setpoint, 0);
            break;
          }
        }
        FSM_HEATING_state = 2;
      }
      if(FSM_HEATING_state == 2){
        CYCLE_cooling_blocking();
        FSM_HEATING_state = 4;
      }
      if(FSM_HEATING_state == 4){
        break;
      }
    }
    FSM_state = 1;
  }
}


void SENSOR_begin(){
  thermo.begin(MAX31865_2WIRE);  // set to 2WIRE or 4WIRE as necessary
}

float SENSOR_read_thermistor(){
  double temp = therm1.analog2temp(); // read temperature
  return temp;
}

float SENSOR_read_thermistor_R2(){
  float average = 0;
  int samples = 0;
  int samplingrate = 5;
  for (uint8_t i = 0; i < samplingrate; i++) {
    samples += analogRead(PIN_THERMISTOR);
    delay(10);
  }
  average = samples / samplingrate;
  // average = analogRead(ntc_pin);

  float Vref = 5.15;
  float buffer = average * Vref;
  float Vout = (buffer)/1023.0;

  // buffer = (Vref/Vout) - 1;
  // R2= R1 * buffer;

  float R1 = 4700.0;
  float R2 = (Vout*R1)/(5.15-Vout);

  // simple way
  // float Vout = average;
  // float R1 = 4700.0;
  // float R2 = R1 * (1023.0 / (float)Vout - 1.0);

  // Serial.print("Thermistor resistance ");
  // Serial.println(R2);
  return R2;
}

#define nominal_resistance 100000.0       //Nominal resistance at 25⁰C
#define nominal_temeprature 25.0   // temperature for nominal resistance (almost always 25⁰ C)
#define beta 3950.0  // The beta coefficient or the B value of the thermistor (usually 3000-4000) check the datasheet for the accurate value.

float SENSOR_read_thermistor_1(){
  float R2 = SENSOR_read_thermistor_R2();
  float temperature;
  temperature = R2 / nominal_resistance;     // (R/Ro)
  temperature = log(temperature);                  // ln(R/Ro)
  temperature /= beta;                   // 1/B * ln(R/Ro)
  temperature += 1.0 / (nominal_temeprature + 273.15); // + (1/To)
  temperature = 1.0 / temperature;                 // Invert
  temperature -= 273.15;                           // convert absolute temp to C
  return temperature;

}
float SENSOR_read_thermistor_2(){
  float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;
  float R2 = SENSOR_read_thermistor_R2();
  float logR2 = log(R2);
  float T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  float Tc = T - 273.15;
  // float Tf = (Tc * 9.0)/ 5.0 + 32.0; 
   
  return Tc;
}

float SENSOR_read_rtd(){
  // uint16_t rtd = thermo.readRTD();
  return thermo.temperature(RNOMINAL, RREF);
}

void SENSOR_loop(){
  if(millis() - SENSOR_millis > SENSOR_period){    //Refresh rate of the PID
    SENSOR_rtd_celcius = SENSOR_read_rtd();
    SENSOR_thermistor_celcius = SENSOR_read_thermistor();
    SENSOR_millis = millis(); 
  }
}

void PID_loop(float setpoint, int temperature){
  if(millis() - PID_millis > PID_period){    //Refresh rate of the PID
    // temperature = thermistor_read();
    PID_setpoint = setpoint;
    if(temperature < PID_setpoint - 10){
      PID_pwm = 255 - 255;
      analogWrite(PIN_SSR, PID_pwm);
    }else if(temperature > PID_setpoint){
      PID_pwm = 255 - 0;
      analogWrite(PIN_SSR, PID_pwm);
    }else{
      //Calculate PID
      PID_error = PID_setpoint - temperature;
      PID_P = Kp*PID_error;
      // PID_I = PID_I+(Ki*PID_ERROR);      
      PID_D = Kd * (PID_error-PID_error_last);
      PID_output = PID_P + 0 + PID_D;

      //Define maximun PID values
      if(PID_output > PID_PWM_max){
        PID_output = PID_PWM_max;
      }
      else if (PID_output < PID_PWM_min){
        PID_output = PID_PWM_min;
      }
      
      //Since the SSR is ON with LOW, we invert the pwm singal
      PID_pwm = 255 - PID_output;
      analogWrite(PIN_SSR, PID_pwm);           //We change the Duty Cycle applied to the SSR
      PID_error_last = PID_error;

      PID_millis = millis(); 
    }
  }
}


void CYCLE_run_blocking(const char text[], float setpoint, int duration){
  PID_setpoint = setpoint;
  FSM_CYCLE_second = 0;
  FSM_CYCLE_state = 0;
  FSM_CYCLE_millis = 0;
  lcd.clear();

  lcd.setCursor(0,1);
  lcd.print("                ");
  lcd.setCursor(0,1);
  lcd.print(text);
  delay(2000);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("T:              ");
  lcd.setCursor(0,1);
  lcd.print("S:");
  lcd.print(PID_setpoint, 1);
  while (1){
    SENSOR_loop();
    // TEST
    if(digitalRead(PIN_BUTTON_decrement) == LOW){
      SENSOR_rtd_celcius = setpoint;
    }

    FSM_CYCLE_temperature = SENSOR_rtd_celcius;

    PID_loop(PID_setpoint, FSM_CYCLE_temperature);
    printParameters();
    // waiting to reach set point
    if(FSM_CYCLE_state == 0){
      if(millis() - FSM_CYCLE_millis > 1000){
        lcd.setCursor(2, 0);
        lcd.print(FSM_CYCLE_temperature, 1);

        FSM_CYCLE_millis = millis();
      }

      if(FSM_CYCLE_temperature > PID_setpoint-5){
        FSM_CYCLE_second = 0;
        FSM_CYCLE_state = 1;
        FSM_CYCLE_second_millis = millis();
      }
    }
    if(FSM_CYCLE_state == 1){
      if(millis() - FSM_CYCLE_millis > 1000){
        lcd.setCursor(2, 0);
        lcd.print(FSM_CYCLE_temperature, 1);
        lcd.setCursor(8, 0);
        lcd.print("HOLD");
        lcd.setCursor(14, 0);
        if(duration > 0){
          lcd.print(FSM_CYCLE_second);
        }else{
          lcd.print("--");
        }
        FSM_CYCLE_millis = millis();
      }
      if(millis() - FSM_CYCLE_second_millis > 1000){
        FSM_CYCLE_second++;
        FSM_CYCLE_second_millis = millis();
      }
      if((duration > 0) && (FSM_CYCLE_second > duration)){
        FSM_CYCLE_state = 2;
      }
    }
    // force next step
    if(FSM_CYCLE_state == 2 || digitalRead(PIN_BUTTON_stop) == LOW){
      delay(200);
      break;
    }
  }
}
void CYCLE_cooling_blocking(){
  tone(PIN_BUZZER, 2500, 150);
  delay(130);
  tone(PIN_BUZZER, 2200, 150);
  delay(130);
  tone(PIN_BUZZER, 2000, 150);
  delay(130);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("T: ");
  lcd.setCursor(0,1);     
  lcd.print("Cooling Down");

  while (1){
    SENSOR_loop();
    FSM_CYCLE_temperature = SENSOR_rtd_celcius;
    printParameters();

    if(millis() - FSM_CYCLE_millis > 1000){
      lcd.setCursor(2, 0);
      lcd.print(FSM_CYCLE_temperature, 1);

      FSM_CYCLE_millis = millis();
    }
    if(digitalRead(PIN_BUTTON_stop) == LOW){
      delay(200);
      break;
    }
  }
}
void printParameters(){
  if(millis() - FSM_print_millis > 200){
    // Serial.print("temp_setpoint: ");
    // Serial.print("temp_setpoint:");
    Serial.print(PID_setpoint);
    Serial.print(",");
    Serial.print(SENSOR_rtd_celcius);
    Serial.print(",");
    Serial.print(SENSOR_thermistor_celcius);
    Serial.print(",");
    Serial.print(PID_pwm);
    Serial.println();
    FSM_print_millis = millis();
  }
}
void deadLock(){
  while(1){
    digitalWrite(LED_BUILTIN, LOW);  
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);  
    delay(100);
  }
}
 

