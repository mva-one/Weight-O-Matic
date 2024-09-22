#include <Arduino.h>
#include <HX711_ADC.h>
#include <EEPROM.h>
#include <LCD_I2C.h>
#include <Wire.h>
#include <CtrlBtn.h>
#include <CtrlEnc.h>

#define VERSION F("v0.6")

#define PIN_BEEP 9
#define PIN_SW_1 5
#define PIN_SW_COM 6
#define PIN_SW_2 7
#define PIN_OUTPUT 8

#define SERIAL_ENABLED

#define DEFAULT_CAL_FACTOR 28.44
#define DEFAULT_TAR_OFFSET 8240259
#define DEFAULT_P1_TARGET 5000
#define DEFAULT_P1_OFFSET 1000
#define DEFAULT_P2_TARGET 10000
#define DEFAULT_P2_OFFSET 2000
#define DEFAULT_TOGGLESETTINGS 0b11000000

HX711_ADC LoadCell(11, 10);     // 11 <-(green)-> DT     10 <-(white)-> SCK
LCD_I2C lcd(0x27, 16, 2);       // A5 <-(blue)-> SCL     A4 <-(green)-> SDA

const uint16_t addr_cal_value = 0x10;   // data type: float (4 bytes)  - addr. 0x10 - 0x13
const uint16_t addr_tar_value = 0x14;   // data type: long  (4 bytes)  - addr. 0x14 - 0x17
const uint16_t addr_saved_flag = 0x18;  // data type: bool  (1 byte)   - addr. 0x18

const uint16_t addr_p1_target = 0x20;       // stores long (4B) - addr. 0x20 - 0x23
const uint16_t addr_p1_offset = 0x24;       // stores long (4B) - addr. 0x24 - 0x27
const uint16_t addr_p1_saved_flag = 0x28;   // stores bool (1B) - addr. 0x28

const uint16_t addr_p2_target = 0x29;       // stores long (4B) - addr. 0x29 - 0x2c
const uint16_t addr_p2_offset = 0x2d;       // stores long (4B) - addr. 0x2d - 0x30
const uint16_t addr_p2_saved_flag = 0x31;   // stores bool (1B) - addr. 0x31

const uint16_t addr_toggle_settings = 0x32;       // stores byte      - addr. 0x32
const uint16_t addr_settings_saved_flag = 0x33;   // stores bool (1B) - addr. 0x33
// toggle settings bitvector content:
// (MSB) 7 -> keytones enabled
//       6 -> endtone enabled


uint32_t t = 0;
uint32_t t_switch = 0;
const uint32_t t_intv_switch = 276;

uint32_t t_weight = 0;
const uint32_t t_intv_weight = 1234;

const uint32_t t_timeout_weight_reading = 2024;

uint8_t sw_pos = 0;
uint8_t sw_pos_pre = 0;
bool sw_event = false;

long s6_known_mass_g = 1550;
bool s6_known_mass_ok = true;

bool target_fill_active = false;
bool target_fill_finished = false;

long current_weight_g = 13420;
long s22_target_g = 5400;
bool s22_target_ok = true;

long s12_target_g = 11000;
long s12_tara_offset_g = 1110;
bool s15_target_ok = true;
long s17_tara_offset_g = 2200;
bool s16_tara_offset_ok = true;
long s17_target_g = 22000;
bool s20_target_ok = true;
bool s21_tara_offset_ok = true;

long last_target_g = 0;
long last_target_done_g = 0;
long t_last_target_started = 0;
long t_last_target_duration = 0;

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

bool output_enabled = true;

// quick and dirty ;)
unsigned int pow10(unsigned int exponent) {
  static unsigned int pow10[10] = {1, 10, 100, 1000, 10000};
  return pow10[exponent]; 
}

void(* reset_function) (void) = 0; // JUST SHUT UP AND RESET!

void disableOutput() {
  digitalWrite(PIN_OUTPUT, LOW);
  output_enabled = false;
  #ifdef SERIAL_ENABLED
  Serial.println("Ausgang deaktiviert.");
  #endif
}

void enableOutput() {
  output_enabled = true;
  digitalWrite(PIN_OUTPUT, HIGH);
  #ifdef SERIAL_ENABLED
  Serial.println("Ausgang aktiviert.");
  #endif
}

uint8_t getToggleSettingsFromState() {
  uint8_t settings_bitvector = 0;
  settings_bitvector = use_keytones ? settings_bitvector | 1 << 7 : settings_bitvector & ~ (1 << 7);
  settings_bitvector = use_endtone ? settings_bitvector | 1 << 6 : settings_bitvector & ~ (1 << 6);
  return settings_bitvector;
}

void setToggleSettingsFromBitvector(uint8_t settings_bitvector) {
  use_keytones = (settings_bitvector & (1 << 7)) >> 7;
  use_endtone = (settings_bitvector & (1 << 6)) >> 6;
}

long w = 0;
void drawCurrentWeight(long* weigth, long* offset = nullptr) {
  w = offset==nullptr ? *weigth : *weigth - *offset;
  lcd.setCursor(1,0);
  if (w < 0) {
    lcd.print(F("-"));
    w = 0 - w;
  }
  else lcd.print(F(" "));
  if (w >= 65536) {
    lcd.print(F("  "));
    lcd.setCursor(5,0);
    lcd.write(3);
  } else {    
    if (w / 10000 != 0) lcd.print(w / 10000); else lcd.write(' ');
    lcd.print(w % 10000 / 1000);
    lcd.setCursor(5,0);
    lcd.print(w % 1000 / 100);
  }  
}

void drawTragetWeight(long* target) {
  lcd.setCursor(7,0);
  // wenn man im Bearbeitungs-Modus ist, soll auch die führende 0 immer erscheinen!
  if (*target / 10000 != 0 || state==15 || state == 20 || state==25) 
    lcd.print(*target / 10000); 
  else lcd.write(' ');
  lcd.print(*target % 10000 / 1000);
  lcd.setCursor(10,0);
  lcd.print(*target % 1000 / 100);
}

void drawTaraOffsetValue(long* value) {
  lcd.setCursor(11,1);
  lcd.print(*value / 1000);
  lcd.setCursor(13,1);
  lcd.print(*value % 1000 / 100);
  lcd.print(*value % 100 / 10);
}

void drawScreenForState(uint8_t targetState) {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.noBlink();
  switch (targetState) {
    case 2: {
      lcd.print(F("  FEHLER!!  :(  "));
      lcd.setCursor(4,1);
      lcd.write(0);
      lcd.print(F(" RESET"));
      break;
    }
    case 4: 
    case 9: {
      lcd.println(F("Waage entlasten!"));
      lcd.print(F("(Tara)"));
      lcd.setCursor(8,1);
      lcd.write(0);
      lcd.print(F(" weiter"));
      break;
    }
    case 41: 
    case 61: 
    case 91: {
      lcd.println(F(" Bitte warten!  "));
      break;
    }
    case 5: {
      lcd.println(F("Bek. Masse aufl."));
      lcd.print(F("(Kal.)"));
      lcd.setCursor(8,1);
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
      lcd.print(F("Kal. speichern?"));
      lcd.setCursor(3,1);
      lcd.print(F("Ja     Nein"));
      break;
    }
    case 10: {
      lcd.print(F("Tara speichern?"));
      lcd.setCursor(3,1);
      lcd.print(F("Ja     Nein"));
      break;
    }
    case 12: {
      lcd.print(F("  --.-/--.-  VE1"));
      lcd.setCursor(0,1);
      lcd.print(F("  START  TV-.--"));
      redraw_screen = true;
      break;
    }
    case 17: {
      lcd.print(F("  --.-/--.-  VE2"));
      lcd.setCursor(0,1);
      lcd.print(F("  START  TV-.--"));
      redraw_screen = true;
      break;
    }
    case 22:
    case 25: {
      lcd.print(F("  --.-/--.-  VE"));
      lcd.write(2);
      lcd.setCursor(0,1);
      lcd.print(F("  START   Einst."));
      redraw_screen = true;
      break;
    }
    case 13:
    case 18:
    case 23: {
      lcd.print(F("  --.-/--.-  VE"));
      if (state==23) lcd.write(2);
      else if (state==13) lcd.write('1');
      else lcd.write('2');
      lcd.setCursor(0,1);
      lcd.print(F("  aktiv   STOPP!"));
      lcd.setCursor(9,1);
      lcd.write(0);
      redraw_screen = true;
      break;
    }
    case 14:
    case 19:
    case 24: {
      long secs = t_last_target_duration / 1000;
      lcd.write(1);
      lcd.print(F(" --.-/--.-  VE"));
      if (state==24) lcd.write(2);
      else if (state==14) lcd.write('1');
      else lcd.write('2');
      drawCurrentWeight(&last_target_done_g);
      drawTragetWeight(&last_target_g);
      lcd.setCursor(0,1);
      lcd.print(F("--min--s  fertig"));
      lcd.setCursor(0,1);
      if (secs > 5999) {
        lcd.write(' ');
        lcd.write(3);
      } else {
        if (secs/600 != 0) lcd.print(secs/600); else lcd.print(' ');
        lcd.print(secs % 600 / 60);
        lcd.setCursor(5,1);
        if (secs % 60 / 10 != 0) lcd.print(secs % 60 / 10); else lcd.print(' ');
        lcd.print(secs % 10);
      }      
      lcd.setCursor(9,1);
      lcd.write(0);
      redraw_screen = true;
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
    case 27: {
      lcd.print(F(" ZURUECKSETZEN? "));
      lcd.setCursor(0,1);
      lcd.print(F("Sicher?  nee  ja"));
      break;
    }
  }
  redraw_screen = true;
}

void stateTransition(uint8_t targetState, uint8_t targetSubState = 0) {
  state = targetState;
  sub_state = targetSubState;

  #ifdef SERIAL_ENABLED
  Serial.print("State transition to ");
  Serial.println(state);
  #endif

  switch (targetState) {
    case 2: {
      drawScreenForState(2);
      break;
    }
    case 4: {
      drawScreenForState(4);
      break;
    }
    case 41: {
      drawScreenForState(41);
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
    case 61: {
      drawScreenForState(61);
      break;
    }
    case 7: {
      drawScreenForState(7);
      break;
    }
    case 9: {
      drawScreenForState(9);
      break;
    }
    case 91: {
      drawScreenForState(91);
      break;
    }
    case 10: {
      drawScreenForState(10);
      break;
    }
    case 12: {
      drawScreenForState(12);
      break;
    }
    case 13: {
      drawScreenForState(13);
      break;
    }
    case 14: {
      drawScreenForState(14);
      break;
    }
    case 15:
    case 16:
    case 20:
    case 21: {
      redraw_screen = true;
      break;
    }
    case 17: {
      drawScreenForState(17);
      break;
    }
    case 18: {
      drawScreenForState(18);
      break;
    }
    case 19: {
      drawScreenForState(19);
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
    case 24: {
      drawScreenForState(24);
      break;
    }
    case 25: {
      s22_target_ok = true;
      redraw_screen = true;
      //drawScreenForState(25);
      break;
    }
    case 26: {
      drawScreenForState(26);
      break;
    }
    case 27: {
      drawScreenForState(27);
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
    case 12:
    case 17: {
      sub_state = (sub_state+1) % 3;
      redraw_screen = true;
      break;
    }
    case 15: {
      if (sub_state < 3) {
        s12_target_g += pow10(4-sub_state);
        if (s12_target_g > 42000) s12_target_g = 42000;
      } 
      else s15_target_ok = !s15_target_ok;
      redraw_screen = true;
      break;
    }
    case 16: {
      if (sub_state < 3) {
        s12_tara_offset_g += pow10(3-sub_state);
        if (s12_tara_offset_g > 9990) s12_tara_offset_g = 9990;
      } 
      else s16_tara_offset_ok = !s16_tara_offset_ok;
      redraw_screen = true;
      break;
    }
    case 20: {
      if (sub_state < 3) {
        s17_target_g += pow10(4-sub_state);
        if (s17_target_g > 42000) s17_target_g = 42000;
      } 
      else s20_target_ok = !s20_target_ok;
      redraw_screen = true;
      break;
    }
    case 21: {
      if (sub_state < 3) {
        s17_tara_offset_g += pow10(3-sub_state);
        if (s17_tara_offset_g > 9990) s17_tara_offset_g = 9990;
      } 
      else s21_tara_offset_ok = !s21_tara_offset_ok;
      redraw_screen = true;
      break;
    }
    case 22: {
      sub_state = (sub_state+1) % 3;
      redraw_screen = true;
      break;
    }
    case 25: {
      if (sub_state < 3) {
        s22_target_g += pow10(4-sub_state);
        if (s22_target_g > 42000) s22_target_g = 42000;
      } 
      else s22_target_ok = !s22_target_ok;
      redraw_screen = true;
      break;
    }
    case 26: {
      sub_state = (sub_state+1) % 6;
      redraw_screen = true;
      break;
    }
    case 27: {
      sub_state = (sub_state+1) % 2;
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
    case 12:
    case 17: {
      sub_state = (sub_state+2) % 3;
      redraw_screen = true;
      break;
    }    
    case 15: {
      if (sub_state < 3) {
        s12_target_g -= pow10(4-sub_state);
        if (s12_target_g < 100) s12_target_g = 100;
      } 
      else s15_target_ok = !s15_target_ok;
      redraw_screen = true;
      break;
    }
    case 16: {
      if (sub_state < 3) {
        s12_tara_offset_g -= pow10(3-sub_state);
        if (s12_tara_offset_g < 10) s12_tara_offset_g = 10;
      } 
      else s16_tara_offset_ok = !s16_tara_offset_ok;
      redraw_screen = true;
      break;
    }
    case 20: {
      if (sub_state < 3) {
        s17_target_g -= pow10(4-sub_state);
        if (s17_target_g < 100) s17_target_g = 100;
      } 
      else s20_target_ok = !s20_target_ok;
      redraw_screen = true;
      break;
    }
    case 21: {
      if (sub_state < 3) {
        s17_tara_offset_g -= pow10(3-sub_state);
        if (s17_tara_offset_g < 10) s17_tara_offset_g = 10;
      } 
      else s21_tara_offset_ok = !s21_tara_offset_ok;
      redraw_screen = true;
      break;
    }
    case 22: {
      sub_state = (sub_state+2) % 3;
      redraw_screen = true;
      break;
    }
    case 25: {
      if (sub_state < 3) {
        s22_target_g -= pow10(4-sub_state);
        if (s22_target_g < 100) s22_target_g = 100;
      } 
      else s22_target_ok = !s22_target_ok;
      redraw_screen = true;
      break;
    }
    case 26: {
      sub_state = (sub_state+5) % 6;
      redraw_screen = true;
      break;
    }
    case 27: {
      sub_state = (sub_state+1) % 2;
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
    case 2: {
      reset_function();
      break;
    }
    case 4: {
      stateTransition(41);
      break;
    }
    case 5: {
      stateTransition(6);
      break;
    }
    case 6: {
      if (sub_state == 4 && s6_known_mass_ok) {
        stateTransition(61);
      }
      else {
        s6_known_mass_ok = true;
        sub_state = (sub_state+1) % 5;
        redraw_screen = true;
      }
      break;
    }
    case 7: {
      if (sub_state == 1) stateTransition(71);
      else stateTransition(11);
      break;
    }
    case 9: {
      stateTransition(91);
      break;
    }
    case 10: {
      if (sub_state == 1) stateTransition(101);
      else stateTransition(11);
      break;
    }
    case 12: 
    case 17: {
      switch(sub_state) {
        case 0: { stateTransition(state+1); break; }
        case 1: { stateTransition(state+4); break; }
        case 2: { stateTransition(state+3); break; }
      }
      break;
    }
    case 13:
    case 18:
    case 23: {
      // WARNUNG: Rechenoperation mit state!
      stateTransition(state+1);
      break;
    }
    case 14:
    case 19:
    case 24: {
      // WARNUNG: Rechenoperation mit state!
      stateTransition(state-2);
      break;
    }
    case 15: {
      if (sub_state == 3 && s15_target_ok) stateTransition(151);
      else {
        s15_target_ok = true;
        sub_state = (sub_state+1) % 4;
        redraw_screen = true;
      }
      break;
    }
    case 16: {
      if (sub_state == 3 && s16_tara_offset_ok) stateTransition(151);
      else {
        s16_tara_offset_ok = true;
        sub_state = (sub_state+1) % 4;
        redraw_screen = true;
      }
      break;
    }
    case 20: {
      if (sub_state == 3 && s20_target_ok) stateTransition(201);
      else {
        s20_target_ok = true;
        sub_state = (sub_state+1) % 4;
        redraw_screen = true;
      }
      break;
    }
    case 21: {
      if (sub_state == 3 && s21_tara_offset_ok) stateTransition(201);
      else {
        s21_tara_offset_ok = true;
        sub_state = (sub_state+1) % 4;
        redraw_screen = true;
      }
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
    case 25: {
      if (sub_state == 3 && s22_target_ok) stateTransition(22);
      else {
        s22_target_ok = true;
        sub_state = (sub_state+1) % 4;
        redraw_screen = true;
      }
      break;
    }
    case 26: {
      switch (sub_state) {
        case 0: {
          stateTransition(28);
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
          stateTransition(27);
          break;
        }
      }
      break;
    }
    case 27: {
      if (sub_state == 1) stateTransition(29);
      else stateTransition(26);
      break;
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

  uint8_t cursor[8] = {0b10000,0b11000,0b11100,0b11110,0b11100,0b11000,0b10000,0};
  uint8_t check[8] = {0,0b00001,0b00011,0b00010,0b10110,0b11100,0b01000,0};
  uint8_t cross[8] = {0,0b10001,0b11011,0b01110,0b01110,0b11011,0b10001,0};
  uint8_t infty[8] = {0,0,0b01010,0b10101,0b10101,0b01010,0,0};
  uint8_t back[8] = {0b00100,0b01000,0b11110,0b01001,0b00101,0b00001,0b00110,0};
  uint8_t up[8] = {0b00100,0b01110,0b11111,0,0,0,0,0};
  uint8_t down[8] = {0,0,0,0,0,0b11111,0b01110,0b00100};

  lcd.begin();
  lcd.backlight();
  lcd.createChar(0, cursor);
  lcd.createChar(1, check);
  lcd.createChar(2, cross);
  lcd.createChar(3, infty);
  lcd.createChar(4, back);
  lcd.createChar(5, up);
  lcd.createChar(6, down);

  stateTransition(1);
  lcd.setCursor(0,0);
  lcd.write(0);
  lcd.print(" Starte...");
  lcd.setCursor(0,1);
  lcd.print(VERSION);
  lcd.setCursor(13,0);
  lcd.blink();



  // check if any value is saved as tara offset and calibration. If not so, save default values
  uint8_t saved_flag = 0;
  EEPROM.get(addr_saved_flag, saved_flag);
  if (saved_flag != 1) {
    EEPROM.put(addr_cal_value, DEFAULT_CAL_FACTOR);
    EEPROM.put(addr_tar_value, DEFAULT_TAR_OFFSET);
    EEPROM.put(addr_saved_flag, (uint8_t)1);
    #ifdef SERIAL_ENABLED
    Serial.println(F("Keine Kalibrierungswerte im EEPROM gefunden, Standardwerte geladen!"));
    #endif
  }

  // the same for the toggle-settings:
  EEPROM.get(addr_settings_saved_flag, saved_flag);
  if (saved_flag != 1) {
    EEPROM.put(addr_toggle_settings, DEFAULT_TOGGLESETTINGS);
    EEPROM.put(addr_settings_saved_flag, (uint8_t)1);
    #ifdef SERIAL_ENABLED
    Serial.println(F("Keine gespeicherten allg. Einstellungen im EEPROM gefunden, Standardwerte geladen!"));
    #endif
  }

  // ...and for preset 1:
  EEPROM.get(addr_p1_saved_flag, saved_flag);
  if (saved_flag != 1) {
    EEPROM.put(addr_p1_target, DEFAULT_P1_TARGET);
    EEPROM.put(addr_p1_offset, DEFAULT_P1_OFFSET);
    EEPROM.put(addr_p1_saved_flag, (uint8_t)1);
    #ifdef SERIAL_ENABLED
    Serial.println(F("Keine gespeicherten Einstellungen für VE 1 im EEPROM gefunden, Standardwerte geladen!"));
    #endif
  }

  // ...and for preset 2:
  EEPROM.get(addr_p2_saved_flag, saved_flag);
  if (saved_flag != 1) {
    EEPROM.put(addr_p2_target, DEFAULT_P2_TARGET);
    EEPROM.put(addr_p2_offset, DEFAULT_P2_OFFSET);
    EEPROM.put(addr_p2_saved_flag, (uint8_t)1);
    #ifdef SERIAL_ENABLED
    Serial.println(F("Keine gespeicherten Einstellungen für VE 2 im EEPROM gefunden, Standardwerte geladen!"));
    #endif
  }

  /*  =============================
        Übergang zu Zustand 1
      ============================= */
  
  // Kalibrierungseinstellungen aus EEPROM laden
  long tar_offset = 0;
  float cal_value = 0.0f;
  EEPROM.get(addr_tar_value, tar_offset);
  EEPROM.get(addr_cal_value, cal_value);
  LoadCell.setTareOffset(tar_offset);
  #ifdef SERIAL_ENABLED
  Serial.print(F("Kalibrierungswerte aus EEPROM erfolgreich geladen: "));
  Serial.print(tar_offset);
  Serial.print(F(" / "));
  Serial.println(cal_value);
  #endif

  // allg. Einstellungen laden
  uint8_t loaded_settings = 0;
  EEPROM.get(addr_toggle_settings, loaded_settings);
  setToggleSettingsFromBitvector(loaded_settings);
  #ifdef SERIAL_ENABLED
  Serial.print(F("Gespeicherte allg. Einstellungen erfolgreich geladen: "));
  Serial.println(loaded_settings, BIN);
  #endif

  // Einstellungen für VE 1 laden
  EEPROM.get(addr_p1_target, s12_target_g);
  EEPROM.get(addr_p1_offset, s12_tara_offset_g);
  #ifdef SERIAL_ENABLED
  Serial.println(F("Gespeicherte Einstellungen für VE 1 erfolgreich geladen!"));
  #endif

  // Einstellungen für VE 2 laden
  EEPROM.get(addr_p2_target, s17_target_g);
  EEPROM.get(addr_p2_offset, s17_tara_offset_g);
  #ifdef SERIAL_ENABLED
  Serial.println(F("Gespeicherte Einstellungen für VE 2 erfolgreich geladen!"));
  #endif

  LoadCell.begin();
  LoadCell.start(2000, false);

  if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
    #ifdef SERIAL_ENABLED
    Serial.println(F("Fehler bei der Verbindung MCU <-> HX711."));
    #endif
    stateTransition(2);
    return;
  }
  else {
    LoadCell.setCalFactor(cal_value);
    #ifdef SERIAL_ENABLED
    Serial.println(F("Wiegezelle erfolgreich initialisiert."));
    #endif
  }
  while (!LoadCell.update());

  lcd.clear();

  stateTransition(11);
}

bool break_loop = false;
void loop() {
  t = millis();
  break_loop = false;

  static float loadcell_reading = 0.0f;
  static uint32_t t_last_weight_reading = t;


  if (LoadCell.update()) {
    t_last_weight_reading = t;
    loadcell_reading = LoadCell.getData();
    if (loadcell_reading < -65536) current_weight_g = -65536;
    else if (loadcell_reading > 65536) current_weight_g = 65536;
    else if (current_weight_g != (long)loadcell_reading) {
      current_weight_g = (long)loadcell_reading;
      redraw_screen = true;
    }
    
    // if (t - t_weight > t_intv_weight) {
    //   t_weight = t;
    //   Serial.print("LoadCell reading / current weight: ");
    //   Serial.print(current_weight_g);
    //   Serial.print(" / ");
    //   Serial.println(current_weight_g);
    // }
  }

  // wenn zu lange kein Messwert mehr gelesen wurde --> Fehlerzustand
  if (t - t_last_weight_reading > t_timeout_weight_reading && state != 2) stateTransition(2);

  // process current state
  switch (state) {
    case 11: { // check switch state, then move to resp. next state
      disableOutput();

      static uint8_t sw_pos = 0;
      if (digitalRead(PIN_SW_1) == LOW) sw_pos = 1;
      else if (digitalRead(PIN_SW_2) == LOW) sw_pos = 2;
      else sw_pos = 0;

      switch (sw_pos) {
        case 0: { stateTransition(22); break; }
        case 1: { stateTransition(12); break; }
        case 2: { stateTransition(17); break; }
      }
      break_loop = true;
      break;
    }
    case 13: {
      last_target_done_g = current_weight_g - s12_tara_offset_g;
      t_last_target_duration = t - t_last_target_started;
      if (current_weight_g - s12_tara_offset_g >= s12_target_g) stateTransition(14);
      else if (!output_enabled && current_weight_g - s12_tara_offset_g < s12_target_g) {
        t_last_target_started = t;
        last_target_g = s12_target_g;
        enableOutput();
      }
      break;
    }
    case 18: {
      last_target_done_g = current_weight_g - s17_tara_offset_g;
      t_last_target_duration = t - t_last_target_started;
      if (current_weight_g - s17_tara_offset_g >= s17_target_g) stateTransition(19); 
      else if (!output_enabled && current_weight_g - s17_tara_offset_g < s17_target_g) {
        t_last_target_started = t;
        last_target_g = s17_target_g;
        enableOutput();
      }
      break;
    }
    case 23: {
      last_target_done_g = current_weight_g;
      t_last_target_duration = t - t_last_target_started;
      if (current_weight_g >= s22_target_g) stateTransition(24);
      else if (!output_enabled && current_weight_g < s22_target_g) {
        t_last_target_started = t;
        last_target_g = s22_target_g;
        enableOutput();
      }
      break;
    }
    case 28: {
      uint8_t settings_new = getToggleSettingsFromState();
      EEPROM.put(addr_toggle_settings, settings_new);
      #ifdef SERIAL_ENABLED
      Serial.print(F("(28) Einstellungen im EEPROM gespeichert: "));
      Serial.println(settings_new, BIN);
      #endif
      stateTransition(22);
      break;
    }
    case 29: {
      for (uint16_t i = 0 ; i < EEPROM.length() ; i++) EEPROM.write(i, 0);
      #ifdef SERIAL_ENABLED
      Serial.println(F("(29) Es wurde alles zurückgesetzt. Starte neu..."));
      #endif
      reset_function();
    }
    case 41: {
      LoadCell.update();
      LoadCell.tare();
      long tar_value = LoadCell.getTareOffset();
      LoadCell.setTareOffset(tar_value);
      #ifdef SERIAL_ENABLED
      Serial.print(F("(41) Tara abgeschlossen. Neuer Wert für tara_offset: "));
      Serial.println(tar_value);
      #endif
      stateTransition(5);
      break;
    }
    case 61: {
      LoadCell.update();
      LoadCell.refreshDataSet();
      float cal_value = LoadCell.getNewCalibration(s6_known_mass_g);
      #ifdef SERIAL_ENABLED
      Serial.print(F("(61) Kalibrierung abgeschlossen. Neuer Wert für cal_value: "));
      Serial.println(cal_value);
      #endif
      stateTransition(7);
      break;
    }
    case 71: {
      float cal_value = LoadCell.getCalFactor();
      long tar_value = LoadCell.getTareOffset();
      EEPROM.put(addr_cal_value, cal_value);
      EEPROM.put(addr_tar_value, tar_value);
      EEPROM.put(addr_saved_flag, (uint8_t)1);
      #ifdef SERIAL_ENABLED
      Serial.print(F("(71) Kalibrierungswerte im EEPROM gespeichert: "));
      Serial.print(cal_value);
      Serial.print(F(" / "));
      Serial.println(tar_value);
      #endif
      stateTransition(11);
      break;
    }
    case 91: {
      long tar_value = 0;
      LoadCell.tare();
      tar_value = LoadCell.getTareOffset();
      LoadCell.setTareOffset(tar_value);
      #ifdef SERIAL_ENABLED
      Serial.print(F("(91) Tara abgeschlossen. Neuer Wert für tara_offset: "));
      Serial.println(tar_value);
      #endif
      stateTransition(10);
      break;
    }
    case 101: {
      long tar_value = LoadCell.getTareOffset();
      EEPROM.put(addr_tar_value, tar_value);
      EEPROM.put(addr_saved_flag, (uint8_t)1);
      #ifdef SERIAL_ENABLED
      Serial.print(F("(101) Tara-Offset im EEPROM gespeichert: "));
      Serial.println(tar_value);
      #endif
      stateTransition(11);
      break;
    }
    case 151: {
      EEPROM.put(addr_p1_target, s12_target_g);
      EEPROM.put(addr_p1_offset, s12_tara_offset_g);
      #ifdef SERIAL_ENABLED
      Serial.print(F("(151) Sollwert / Tara-Versatz für VE 1 im EEPROM gespeichert: "));
      Serial.print(s12_target_g);
      Serial.print(F(" / "));
      Serial.println(s12_tara_offset_g);
      #endif
      stateTransition(12);
      break;
    }
    case 201: {
      EEPROM.put(addr_p2_target, s17_target_g);
      EEPROM.put(addr_p2_offset, s17_tara_offset_g);
      #ifdef SERIAL_ENABLED
      Serial.print(F("(201) Sollwert / Tara-Versatz für VE 2 im EEPROM gespeichert: "));
      Serial.print(s17_target_g);
      Serial.print(F(" / "));
      Serial.println(s17_tara_offset_g);
      #endif
      stateTransition(17);
      break;
    }
  }

  // Ausgang ausschalten, wenn nicht in einem entsprechenden State
  if (output_enabled && state != 13 && state != 18 && state != 23) {
    disableOutput();
  }

  // skip the rest of loop for certain states
  if (break_loop) {
    #ifdef SERIAL_ENABLED
    Serial.println(F("The loop has been broken. Just like my heart <|3"));
    #endif
    return;
  }

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

        if (use_keytones) tone(PIN_BEEP, 440, 150);
        break;
      }
      case 1: {
        #ifdef SERIAL_ENABLED
        Serial.println("Switch changed to position 1.");
        #endif

        if (use_keytones) tone(PIN_BEEP, 440, 150);
        break;
      }
      case 2: {
        #ifdef SERIAL_ENABLED
        Serial.println("Switch changed to position 2.");
        #endif

        if (use_keytones) tone(PIN_BEEP, 440, 150);
        break;
      }
    }

    // TODO
    //if (state > 8 && state < 27)
    stateTransition(11);
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
      case 12: 
      case 17: {
        lcd.setCursor(0,0);
        if (sub_state == 2) lcd.write(0); else lcd.write(' ');
        if (state == 12) {
          drawCurrentWeight(&current_weight_g, &s12_tara_offset_g);
          drawTragetWeight(&s12_target_g);
          drawTaraOffsetValue(&s12_tara_offset_g);
        } else {
          drawCurrentWeight(&current_weight_g, &s17_tara_offset_g);
          drawTragetWeight(&s17_target_g);
          drawTaraOffsetValue(&s17_tara_offset_g);
        }
        lcd.setCursor(1,1);
        if (sub_state == 0) lcd.write(0); else lcd.write(' ');
        lcd.setCursor(8,1);
        if (sub_state == 1) lcd.write(0); else lcd.write(' ');
        break;
      }
      case 13: {
        drawCurrentWeight(&current_weight_g, &s12_tara_offset_g);
        drawTragetWeight(&s12_target_g);
        break;
      }
      case 15:
      case 20: {
        lcd.setCursor(0,0);
        lcd.write(0);
        if (state == 15) {
          drawCurrentWeight(&current_weight_g, &s12_tara_offset_g);
          drawTragetWeight(&s12_target_g);
          if (s15_target_ok) lcd.write(1); else lcd.write(4);
        } else {
          drawCurrentWeight(&current_weight_g, &s17_tara_offset_g);
          drawTragetWeight(&s17_target_g);
          if (s20_target_ok) lcd.write(1); else lcd.write(4);
        }
        
        switch (sub_state) {
          case 0: { lcd.setCursor(7,0); break; }
          case 1: { lcd.setCursor(8,0); break; }
          case 2: { lcd.setCursor(10,0); break; }
          case 3: { lcd.setCursor(11,0); break; }
        }
        lcd.blink();
        break;
      }
      case 16: 
      case 21: {
        lcd.setCursor(8,1);
        lcd.write(0);
        
        if (state == 16) {
          drawCurrentWeight(&current_weight_g, &s12_tara_offset_g);
          drawTaraOffsetValue(&s12_tara_offset_g);
          if (s16_tara_offset_ok) lcd.write(1); else lcd.write(4);
        } else {
          drawCurrentWeight(&current_weight_g, &s17_tara_offset_g);
          drawTaraOffsetValue(&s17_tara_offset_g);
          if (s21_tara_offset_ok) lcd.write(1); else lcd.write(4);
        }

        switch(sub_state) {
          case 0: { lcd.setCursor(11,1); break; }
          case 1: { lcd.setCursor(13,1); break; }
          case 2: { lcd.setCursor(14,1); break; }
          case 3: { lcd.setCursor(15,1); break; }
        }
        lcd.blink();
        break;
      }
      case 18: {
        drawCurrentWeight(&current_weight_g, &s17_tara_offset_g);
        drawTragetWeight(&s17_target_g);
        break;
      }
      case 22: {
        lcd.setCursor(0,0);
        if (sub_state == 2) lcd.write(0); else lcd.write(' ');
        drawCurrentWeight(&current_weight_g);
        drawTragetWeight(&s22_target_g);
        lcd.setCursor(1,1);
        if (sub_state == 0) lcd.write(0); else lcd.write(' ');
        lcd.setCursor(9,1);
        if (sub_state == 1) lcd.write(0); else lcd.write(' ');
        break;
      }
      case 23: {
        drawCurrentWeight(&current_weight_g);
        drawTragetWeight(&s22_target_g);
        break;
      }
      case 25: {
        lcd.setCursor(0,0);
        lcd.write(0);
        drawCurrentWeight(&current_weight_g);
        drawTragetWeight(&s22_target_g);
        if (s22_target_ok) lcd.write(1); else lcd.write(4);
        switch (sub_state) {
          case 0: { lcd.setCursor(7,0); break; }
          case 1: { lcd.setCursor(8,0); break; }
          case 2: { lcd.setCursor(10,0); break; }
          case 3: { lcd.setCursor(11,0); break; }
        }
        lcd.blink();
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
      case 27: {
        lcd.setCursor(8,1);
        if (sub_state == 0) lcd.write(0); else lcd.write(' ');
        lcd.setCursor(13,1);
        if (sub_state == 0) lcd.write(' '); else lcd.write(0);
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