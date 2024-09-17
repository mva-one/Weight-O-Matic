#include <Arduino.h>
#include <HX711_ADC.h>
#include <EEPROM.h>
#include <LCD_I2C.h>
#include <Wire.h>
#include <CtrlBtn.h>
#include <CtrlEnc.h>

#define VERSION F("v0.4")

#define PIN_BEEP 9
#define PIN_SW_1 5
#define PIN_SW_COM 6
#define PIN_SW_2 7
#define PIN_OUTPUT 8

#define SERIAL_ENABLED

HX711_ADC LoadCell(11, 10);     // 11 <-(green)-> DT     10 <-(white)-> SCK
LCD_I2C lcd(0x27, 16, 2);       // A5 <-(blue)-> SCL    A4 <-(green)-> SDA

uint8_t cursor[8] = {0b10000,0b11000,0b11100,0b11110,0b11100,0b11000,0b10000,0};
uint8_t check[8] = {0,0b00001,0b00011,0b00010,0b10110,0b11100,0b01000,0};
uint8_t cross[8] = {0,0b10001,0b11011,0b01110,0b01110,0b11011,0b10001,0};
uint8_t infty[8] = {0,0,0b01010,0b10101,0b10101,0b01010,0,0};
uint8_t back[8] = {0b00100,0b01000,0b11110,0b01001,0b00101,0b00001,0b00110,0};
uint8_t up[8] = {0b00100,0b01110,0b11111,0,0,0,0,0};
uint8_t down[8] = {0,0,0,0,0,0b11111,0b01110,0b00100};

const uint16_t addr_cal_value = 4;   // data type: float (4 bytes)  - addr. 4, 5, 6, 7
const uint16_t addr_cal_saved = 8;   // data type: bool  (1 byte)   - addr. 8
const uint16_t addr_tara_value = 9;  // data type: float (4 bytes)  - addr. 9, 10, 11, 12
const uint16_t addr_tara_saved = 13; // data type: bool  (1 byte)   - addr. 13

uint32_t t = 0;

// "mode selection switch"
uint32_t t_switch = 0;
const uint32_t t_intv_switch = 242;
uint8_t sw_pos = 0; // 0, 1 or 2
uint8_t sw_pos_pre = 0;
bool sw_event = true;

uint8_t error = 0;


// measurement
float current_weight = -99.0f;
float target_weight = 10.2f;
float cal_value = 0.0f;
float tara_value = 0.0f;

long s6_known_mass_g = 1550;
bool s6_known_mass_ok = true;

bool target_fill_active = false;
bool target_fill_finished = false;


// audio
bool use_keytones = true;
bool use_endtone = true;


// LCD menu
bool redraw_screen = true;
uint32_t t_screen = 0;
uint32_t t_intv_screen = 100;

// Menu List:
uint8_t state = 0;
uint8_t sub_state = 0;

float current_reading = 42.24;

// quick and dirty ;)
unsigned int pow10(unsigned int exponent) {
  static int pow10[10] = {1, 10, 100, 1000, 10000};
  return pow10[exponent]; 
}

void disableOutput() {

}

void drawScreenForState(uint8_t targetState) {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.noBlink();
  switch (targetState) {
    case 4: 
    case 9: {
      lcd.print(F("Waage entlasten!"));
      lcd.setCursor(4,1);
      lcd.write(0);
      lcd.print(F(" weiter"));
      break;
    }
    case 5: {
      lcd.print(F("Bek. Masse aufl."));
      lcd.setCursor(4,1);
      lcd.write(0);
      lcd.print(F(" weiter"));
      break;
    }
    case 6: {
      lcd.print(F("Bekannte Masse:"));
      lcd.setCursor(1,1);
      lcd.write(0);
      break;
    }
    case 7: {
      lcd.print(F(" ROM speichern?"));
      lcd.setCursor(3,1);
      lcd.print(F("Ja     Nein"));
      break;
    }
    case 12: {
      lcd.print(F(" 00.0/00.0   VE1"));
      lcd.setCursor(0,1);
      lcd.print(F("  START  TV0.42"));
      break;
    }
    case 17: {
      lcd.print(F(" 00.0/00.0   VE2"));
      lcd.setCursor(0,1);
      lcd.print(F("  START  TV0.84"));
      break;
    }
    case 22: {
      // TODO hier echte Werte verwenden!
      lcd.print(F(" 00.0/00.0   VE"));
      lcd.write(2);
      lcd.setCursor(0,1);
      lcd.print(F("  START   Einst."));
      break;
    }
    case 23: {
      // TODO hier echte Werte verwenden!
      lcd.print(F(" 00.0/00.0   VE"));
      lcd.write(2);
      lcd.setCursor(0,1);
      lcd.print(F("  aktiv   STOPP!"));
      lcd.setCursor(9,1);
      lcd.write(0);
      break;
    }
    case 26: {
      lcd.setCursor(1,0);
      lcd.write(4);
      lcd.setCursor(7,0);
      lcd.print(F("TT    ET"));
      lcd.setCursor(1,1);
      lcd.print(F("TARA  KALI  RST"));
      break;
    }
  }
  redraw_screen = true;
}

void stateTransition(uint8_t targetState) {
  state = targetState;
  sub_state = 0;

  #ifdef SERIAL_ENABLED
  Serial.print("State transition to ");
  Serial.println(state);
  #endif

  switch (targetState) {
    case 4: {
      drawScreenForState(4);
      break;
    }
    case 5: {
      drawScreenForState(5);
      break;
    }
    case 6: {
      s6_known_mass_ok = true;
      drawScreenForState(6);
      break;
    }
    case 7:
    case 10: { // screen for 7 is the same as for 10!
      drawScreenForState(7);
      break;
    }
    case 9: {
      drawScreenForState(9);
      break;
    }
    case 12: {
      drawScreenForState(12);
      break;
    }
    case 17: {
      drawScreenForState(17);
      break;
    }
    case 22: {
      drawScreenForState(22);
      break;
    }
    case 23: {
      drawScreenForState(23);
      break;
    }
    case 26: {
      drawScreenForState(26);
      break;
    }
  }
}


void onTurnRight() {
  if (use_keytones) tone(PIN_BEEP, 1100, 15);
  #ifdef SERIAL_ENABLED
  Serial.println("Rotary turned right.");
  #endif

  switch (state) {
    case 6: {
      if (sub_state < 4) {
        s6_known_mass_g += pow10(4-sub_state);
        if (s6_known_mass_g > 42000) s6_known_mass_g = 42000;
      } 
      else s6_known_mass_ok = !s6_known_mass_ok;
      redraw_screen = true;
      break;
    }
    case 7: 
    case 10: {
      sub_state = (sub_state+1) % 2;
      redraw_screen = true;
      break;
    }
    case 22: {
      sub_state = (sub_state+1) % 3;
      redraw_screen = true;
      break;
    }
    case 26: {
      sub_state = (sub_state+1) % 6;
      redraw_screen = true;
      break;
    }
  }
}

void onTurnLeft() {
  if (use_keytones) tone(PIN_BEEP, 1350, 15);
  #ifdef SERIAL_ENABLED
  Serial.println("Rotary turned left.");
  #endif

  switch (state) {
    case 6: {
      if (sub_state < 4) {
        s6_known_mass_g -= pow10(4-sub_state);
        if (s6_known_mass_g < 10) s6_known_mass_g = 10;
      } 
      else s6_known_mass_ok = !s6_known_mass_ok;
      redraw_screen = true;
      break;
    }
    case 7: 
    case 10: {
      sub_state = (sub_state+1) % 2;
      redraw_screen = true;
      break;
    }
    case 22: {
      sub_state = (sub_state+2) % 3;
      redraw_screen = true;
      break;
    }
    case 26: {
      sub_state = (sub_state+5) % 6;
      redraw_screen = true;
      break;
    }
  }
}

void shortClick_enc() {
  if (use_keytones) tone(PIN_BEEP, 925, 40);
  #ifdef SERIAL_ENABLED
  Serial.println("Rotary button short click.");
  #endif

  switch (state) {
    case 4: {
      stateTransition(5);
      break;
    }
    case 5: {
      stateTransition(6);
      break;
    }
    case 6: {
      if (sub_state == 4 && s6_known_mass_ok) stateTransition(7);
      else {
        s6_known_mass_ok = true;
        sub_state = (sub_state+1) % 5;
      }
      redraw_screen = true;
      break;
    }
    case 7: {
      if (sub_state == 1) stateTransition(71);
      else stateTransition(3);
      break;
    }
    case 9: {
      stateTransition(91);
      break;
    }
    case 10: {
      if (sub_state == 1) stateTransition(101);
      else stateTransition(8);
      break;
    }
    case 22: {
      switch (sub_state) {
        case 0: {
          // TODO
          // Dosierung starten? Oder wirklich nur bei langem Klick?
          stateTransition(23);
          break;
        }
        case 1: {
          stateTransition(26);
          break;
        }
        case 2: {
          stateTransition(25);
          break;
        }
      }
      break;
    }
    case 23: {
      // TODO hier noch unbedingt den Ausgang ausschalten?
      stateTransition(22);
      break;
    }
    case 26: {
      switch (sub_state) {
        case 0: {
          stateTransition(22);
          break;
        }
        case 1: {
          use_keytones = !use_keytones;
          redraw_screen = true;
          break;
        }
        case 2: {
          use_endtone = !use_endtone;
          redraw_screen = true;
          break;
        }
        case 3: {
          stateTransition(9);
          break;
        }
        case 4: {
          stateTransition(4);
          break;
        }
        case 5: {
          // TODO wirklich noch alles zurücksetzen!
          // stateTransition(27);
          break;
        }
      }
    }
  }
}

void longClick_enc() {
  if (use_keytones) tone(PIN_BEEP, 925, 250);
  #ifdef SERIAL_ENABLED
  Serial.println("Rotary button long click.");
  #endif

}

CtrlEnc encoder(3, 4, onTurnRight, onTurnLeft);                   // 3 <-> CLK    4 <-> DT     2 <-> SW
CtrlBtn btn_enc(2, 20, nullptr, shortClick_enc, longClick_enc);

void setup() {
  pinMode(PIN_SW_COM, OUTPUT);
  pinMode(PIN_OUTPUT, OUTPUT);
  pinMode(PIN_SW_1, INPUT_PULLUP);
  pinMode(PIN_SW_2, INPUT_PULLUP);
  digitalWrite(PIN_SW_COM, LOW);
  digitalWrite(PIN_OUTPUT, LOW);

  
  #ifdef SERIAL_ENABLED
  Serial.begin(115200);
  delay(10);
  Serial.println();
  Serial.println("Starting...");
  #endif

  lcd.begin();
  lcd.backlight();
  lcd.createChar(0, cursor);
  lcd.createChar(1, check);
  lcd.createChar(2, cross);
  lcd.createChar(3, infty);
  lcd.createChar(4, back);
  lcd.createChar(5, up);
  lcd.createChar(6, down);

  lcd.setCursor(0,0);
  lcd.write(0);
  lcd.print(" Starte...");
  lcd.setCursor(0,1);
  lcd.print(VERSION);
  lcd.setCursor(13,0);
  lcd.blink();

  LoadCell.begin();
  LoadCell.start(2000, false);

  if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {

    #ifdef SERIAL_ENABLED
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    #endif
    while (1);
  }
  else {
    LoadCell.setCalFactor(1.0); // user set calibration value (float), initial value 1.0 may be used for this sketch

    #ifdef SERIAL_ENABLED
    Serial.println("Load Cell Startup is complete");
    #endif
  }
  while (!LoadCell.update());

  lcd.clear();

  // --> Calibration Check
  stateTransition(3);
}

bool break_loop;

void loop() {
  t = millis();
  break_loop = false;

  // process current state
  switch (state) {
    case 3: { // Calibration Check
      // is calibration value saved to EEPROM?
      uint8_t cal_saved = false;
      EEPROM.get(addr_cal_saved, cal_saved);
      if (cal_saved == 1) {
        EEPROM.get(addr_cal_value, cal_value);
        stateTransition(8);
      }
      else stateTransition(4);
      break_loop = true;
      break;
    }
    case 71: { // save calibration data, delete tara
      // TODO
      //EEPROM.put(addr_cal_value, ??);
      EEPROM.put(addr_cal_saved, (uint8_t)1);
      EEPROM.put(addr_tara_saved, (uint8_t)0);
      stateTransition(3);
      break_loop = true;
      break;
    }
    case 8: { // check tara
      // is tara offset saved to EEPROM?
      uint8_t tara_saved = false;
      EEPROM.get(addr_tara_saved, tara_saved);
      if (tara_saved == 1) {
        EEPROM.get(addr_tara_value, tara_value);
        stateTransition(11);
      }
      else stateTransition(9);
      break_loop = true;
      break;
    }
    case 91: { // measure tara
      // TODO  -  actually measure something!
      tara_value = 14.6;
      stateTransition(10);
      break_loop =true;
      break;
    }
    case 101: { // save tara to eeprom
      // TODO  -  actually save it
      //EEPROM.put(addr_tara_value, ??);
      EEPROM.put(addr_tara_saved, (uint8_t)1);
      stateTransition(8);
      break_loop =true;
      break;
    }
    case 11: { // check switch state, then move to resp. next state
      disableOutput();

      switch (sw_pos) {
        case 0: { stateTransition(22); break; }
        case 1: { stateTransition(12); break; }
        case 2: { stateTransition(17); break; }
      }
      break_loop =true;
      break;
    }
  }

  // skip the rest of loop for certain states
  if (break_loop) return;



  encoder.process();
  btn_enc.process();

  // read the mode selection switch
  if (t - t_switch > t_intv_switch) {
    t_switch = t;
    sw_pos_pre = sw_pos;
    if (digitalRead(PIN_SW_1) == LOW) sw_pos = 1;
    else if (digitalRead(PIN_SW_2) == LOW) sw_pos = 2;
    else sw_pos = 0;

    if (sw_pos != sw_pos_pre) sw_event = true;
  }


  // handle switch change event
  if (sw_event) {
    switch (sw_pos) {
      case 0: {
        #ifdef SERIAL_ENABLED
        Serial.println("Switch changed to position 0.");
        #endif

        if (use_keytones) tone(PIN_BEEP,440,150);

        break;
      }
      case 1: {
        #ifdef SERIAL_ENABLED
        Serial.println("Switch changed to position 1.");
        #endif

        if (use_keytones) tone(PIN_BEEP,440,150);
        
        break;
      }
      case 2: {
        #ifdef SERIAL_ENABLED
        Serial.println("Switch changed to position 2.");
        #endif

        if (use_keytones) tone(PIN_BEEP,440,150);
        
        break;
      }
    }

    // TODO
    if (state > 8 && state < 27) stateTransition(11);

    sw_event = false;
  }


  // refresh screen if required
  if (t - t_screen > t_intv_screen && redraw_screen) {
    t_screen = t;
    redraw_screen = false;

    switch(state) {
      case 6: {
        lcd.setCursor(3,1);
        lcd.print(s6_known_mass_g/10000);
        lcd.print(s6_known_mass_g % 10000 / 1000);
        lcd.write('.');
        lcd.print(s6_known_mass_g % 1000 / 100);
        lcd.print(s6_known_mass_g % 100 / 10);
        lcd.print(F(" kg "));
        if (s6_known_mass_ok) lcd.write(1); else lcd.write(4);
        switch (sub_state) {
          case 0: { lcd.setCursor(3,1); break; }
          case 1: { lcd.setCursor(4,1); break; }
          case 2: { lcd.setCursor(6,1); break; }
          case 3: { lcd.setCursor(7,1); break; }
          case 4: { lcd.setCursor(12,1); break; }
        }
        lcd.blink();
        break;
      }
      case 7: 
      case 10: {
        lcd.setCursor(2,1);
        if (sub_state == 0) lcd.write(' '); else lcd.write(0);
        lcd.setCursor(9,1);
        if (sub_state == 0) lcd.write(0); else lcd.write(' ');
        break;
      }
      case 22: {
        lcd.setCursor(0,0);
        if (sub_state == 2) lcd.write(0); else lcd.write(' ');
        lcd.setCursor(1,1);
        if (sub_state == 0) lcd.write(0); else lcd.write(' ');
        lcd.setCursor(9,1);
        if (sub_state == 1) lcd.write(0); else lcd.write(' ');
        break;
      }
      case 26: {
        for (uint8_t row = 0; row < 2; row++) {
          for (uint8_t col = 0; col < 3; col++) {
            lcd.setCursor(col*6,row); if (sub_state == row*3+col) lcd.write(0); else lcd.write(' ');
          }
        }
        lcd.setCursor(9,0); if (use_keytones) lcd.write(1); else lcd.write(2);
        lcd.setCursor(15,0); if (use_endtone) lcd.write(1); else lcd.write(2);
        break;
      }
    }
  }


  //static boolean newDataReady = 0;
  //const int serialPrintInterval = 3330; //increase value to slow down serial print activity

  

  // check for new data/start next conversion:
  //if (LoadCell.update()) newDataReady = true;

  // get smoothed value from the dataset:
  // if (newDataReady) {
  //   if (millis() > t + serialPrintInterval) {
  //     float i = LoadCell.getData();
  //     Serial.print("Load_cell output val: ");
  //     Serial.println(i);
  //     newDataReady = 0;
  //     t = millis();
  //   }
  // }

  // receive command from serial terminal
  // if (Serial.available() > 0) {
  //   char inByte = Serial.read();
  //   if (inByte == 't') LoadCell.tareNoDelay(); //tare
  //   else if (inByte == 'r') calibrate(); //calibrate
  //   else if (inByte == 'c') changeSavedCalFactor(); //edit calibration value manually
  // }

  // check if last tare operation is complete
  // if (LoadCell.getTareStatus() == true) {
  //   Serial.println("Tare complete");
  // }

}

// void calibrate() {
//   Serial.println("***");
//   Serial.println("Start calibration:");
//   Serial.println("Place the load cell an a level stable surface.");
//   Serial.println("Remove any load applied to the load cell.");
//   Serial.println("Send 't' from serial monitor to set the tare offset.");

//   boolean _resume = false;
//   while (_resume == false) {
//     LoadCell.update();
//     if (Serial.available() > 0) {
//       if (Serial.available() > 0) {
//         char inByte = Serial.read();
//         if (inByte == 't') LoadCell.tareNoDelay();
//       }
//     }
//     if (LoadCell.getTareStatus() == true) {
//       Serial.println("Tare complete");
//       _resume = true;
//     }
//   }

//   Serial.println("Now, place your known mass on the loadcell.");
//   Serial.println("Then send the weight of this mass (i.e. 100.0) from serial monitor.");

//   float known_mass = 0;
//   _resume = false;
//   while (_resume == false) {
//     LoadCell.update();
//     if (Serial.available() > 0) {
//       known_mass = Serial.parseFloat();
//       if (known_mass != 0) {
//         Serial.print("Known mass is: ");
//         Serial.println(known_mass);
//         _resume = true;
//       }
//     }
//   }

//   LoadCell.refreshDataSet(); //refresh the dataset to be sure that the known mass is measured correct
//   float newCalibrationValue = LoadCell.getNewCalibration(known_mass); //get the new calibration value

//   Serial.print("New calibration value has been set to: ");
//   Serial.print(newCalibrationValue);
//   Serial.println(", use this as calibration value (calFactor) in your project sketch.");
//   Serial.print("Save this value to EEPROM adress ");
//   Serial.print(calVal_eepromAdress);
//   Serial.println("? y/n");

//   _resume = false;
//   while (_resume == false) {
//     if (Serial.available() > 0) {
//       char inByte = Serial.read();
//       if (inByte == 'y') {
//         EEPROM.put(calVal_eepromAdress, newCalibrationValue);
//         EEPROM.get(calVal_eepromAdress, newCalibrationValue);
//         Serial.print("Value ");
//         Serial.print(newCalibrationValue);
//         Serial.print(" saved to EEPROM address: ");
//         Serial.println(calVal_eepromAdress);
//         _resume = true;

//       }
//       else if (inByte == 'n') {
//         Serial.println("Value not saved to EEPROM");
//         _resume = true;
//       }
//     }
//   }

//   Serial.println("End calibration");
//   Serial.println("***");
//   Serial.println("To re-calibrate, send 'r' from serial monitor.");
//   Serial.println("For manual edit of the calibration value, send 'c' from serial monitor.");
//   Serial.println("***");
// }

// void changeSavedCalFactor() {
//   float oldCalibrationValue = LoadCell.getCalFactor();
//   boolean _resume = false;
//   Serial.println("***");
//   Serial.print("Current value is: ");
//   Serial.println(oldCalibrationValue);
//   Serial.println("Now, send the new value from serial monitor, i.e. 696.0");
//   float newCalibrationValue;
//   while (_resume == false) {
//     if (Serial.available() > 0) {
//       newCalibrationValue = Serial.parseFloat();
//       if (newCalibrationValue != 0) {
//         Serial.print("New calibration value is: ");
//         Serial.println(newCalibrationValue);
//         LoadCell.setCalFactor(newCalibrationValue);
//         _resume = true;
//       }
//     }
//   }
//   _resume = false;
//   Serial.print("Save this value to EEPROM adress ");
//   Serial.print(calVal_eepromAdress);
//   Serial.println("? y/n");
//   while (_resume == false) {
//     if (Serial.available() > 0) {
//       char inByte = Serial.read();
//       if (inByte == 'y') {
//         EEPROM.put(calVal_eepromAdress, newCalibrationValue);
//         EEPROM.get(calVal_eepromAdress, newCalibrationValue);
//         Serial.print("Value ");
//         Serial.print(newCalibrationValue);
//         Serial.print(" saved to EEPROM address: ");
//         Serial.println(calVal_eepromAdress);
//         _resume = true;
//       }
//       else if (inByte == 'n') {
//         Serial.println("Value not saved to EEPROM");
//         _resume = true;
//       }
//     }
//   }
//   Serial.println("End change calibration value");
//   Serial.println("***");
// }