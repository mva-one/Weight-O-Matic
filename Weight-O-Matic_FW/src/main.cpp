#include <Arduino.h>
#include <HX711_ADC.h>
#include <EEPROM.h>
#include <LCD_I2C.h>
#include <Wire.h>
#include <CtrlBtn.h>
#include <CtrlEnc.h>

#define VERSION F("v0.7")

#define PIN_BEEP 9              // 9 <-(rot)-> Piepser
#define PIN_SW_1 5              // 5 <-(braun)-> Schalter 'I'
#define PIN_SW_COM 6            // 5 <-(schwarz)-> Schalter Common
#define PIN_SW_2 7              // 5 <-(braun)-> Schalter 'II'
#define PIN_OUTPUT 8            // 8 <-(rot)-> Schaltmodul
#define PIN_HX711_DAT 11        // 11 <-(grün)-> DT HX711
#define PIN_HX711_SCK 10        // 10 <-(weiß)-> SCK HX711
#define PIN_ENCODER_CLK 3       // 3 <-(blau)-> CLK Dreh/Drückschalter
#define PIN_ENCODER_DAT 4       // 4 <-(grün)-> DAT Dreh/Drückschalter
#define PIN_ENCODER_BTN 2       // 2<-(braun)-> SW Dreh/Drückschalter
// I2C Pins sind default:       // A5 <-(blau)-> SCL     A4 <-(grün)-> SDA  

// Tonhöhen und -Längen für Tastentöne
#define BEEP_FREQ_LEFT 1350
#define BEEP_FREQ_RIGHT 1100
#define BEEP_FREQ_CLICK 925
#define BEEP_LENGTH_TURN 15
#define BEEP_LENGTH_SHORT 40
#define BEEP_LENGTH_LONG 250

// Tonhöhen in Hz und "Takt" für Ende-Melodie
#define BEEP_FREQ_A5 880
#define BEEP_FREQ_F5 698
#define BEEP_FREQ_G5 784
#define BEEP_FREQ_B5 932
#define BEEP_UNIT_LENGTH 62
#define BEEP_END_REPETITIONS 4  // Wiederholungen der Ende-Melodie -> bei 4 Wiederholungen wird 5mal abgespielt

// Debug-Ausgaben über die Serielle Konsole aktivieren (Baud 115200)
#define SERIAL_ENABLED

// Standardwerte, werden bei leerem EEPROM geladen (z.B. auch nach dem Zurücksetzen über das Menü)
#define DEFAULT_CAL_FACTOR 28.44
#define DEFAULT_TAR_OFFSET 8240259
#define DEFAULT_P1_TARGET 5000
#define DEFAULT_P1_OFFSET 1000
#define DEFAULT_P2_TARGET 10000
#define DEFAULT_P2_OFFSET 2000
#define DEFAULT_TOGGLESETTINGS 0b11000000


// EEPROM-Adressen für verschiedene Einstellungen:
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

// Intervall (ms) zum Auslesen des Modus-Schalters
const uint32_t t_intv_switch = 276;

// Timeout (ms) für Kommunikation mit Wiegezelle
const uint32_t t_timeout_weight_reading = 2024;

uint8_t sw_pos = 0;
uint8_t sw_pos_pre = 0;
bool sw_event = false;

long s6_known_mass_g = 1550;
bool s6_known_mass_ok = true;

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

// Audio-Einstellungen
bool use_keytones = true;
bool use_endtone = true;
int endtone_repetitions_done = 0;


// LCD-Menü und Zustände
bool redraw_screen = true;
const uint32_t t_intv_screen = 100;
uint8_t state = 0;
uint8_t sub_state = 0;

// Status-Flag, ob der Ausgang aktiviert ist.
bool output_enabled = true;

// Hardware aus Libraries
HX711_ADC loadcell  (PIN_HX711_DAT, PIN_HX711_SCK);
LCD_I2C   lcd       (0x27, 16, 2);

// quick and dirty ;)
unsigned int pow10(unsigned int exponent) {
  static unsigned int pow10[10] = {1, 10, 100, 1000, 10000};
  return pow10[exponent]; 
}

// Einfache Prozedur für Software-Reset
void(* reset_function) (void) = 0;

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

// Einstellungs-Bitvektor aus den aktuell aktiven Einstellungen erstellen
uint8_t getToggleSettingsFromState() {
  uint8_t settings_bitvector = 0;
  settings_bitvector = use_keytones ? settings_bitvector | 1 << 7 : settings_bitvector & ~ (1 << 7);
  settings_bitvector = use_endtone ? settings_bitvector | 1 << 6 : settings_bitvector & ~ (1 << 6);
  return settings_bitvector;
}

// Aktive Einstellungen aus Einstellungs-Bitvektor übernehmen
void setToggleSettingsFromBitvector(uint8_t settings_bitvector) {
  use_keytones = (settings_bitvector & (1 << 7)) >> 7;
  use_endtone = (settings_bitvector & (1 << 6)) >> 6;
}

void drawCurrentWeight(long* weigth, long* offset = nullptr) {
  static long w = 0;
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
  static bool beep;
  beep = true;
  redraw_screen = true;

  switch (state) {
    case 6: {
      if (sub_state < 4) {
        s6_known_mass_g += pow10(4-sub_state);
        if (s6_known_mass_g > 42000) s6_known_mass_g = 42000;
      } 
      else s6_known_mass_ok = !s6_known_mass_ok;
      break;
    }
    case 7: 
    case 10: {
      sub_state = (sub_state+1) % 2;
      break;
    }
    case 12:
    case 17: {
      sub_state = (sub_state+1) % 3;
      break;
    }
    case 15: {
      if (sub_state < 3) {
        s12_target_g += pow10(4-sub_state);
        if (s12_target_g > 42000) s12_target_g = 42000;
      } 
      else s15_target_ok = !s15_target_ok;
      break;
    }
    case 16: {
      if (sub_state < 3) {
        s12_tara_offset_g += pow10(3-sub_state);
        if (s12_tara_offset_g > 9990) s12_tara_offset_g = 9990;
      } 
      else s16_tara_offset_ok = !s16_tara_offset_ok;
      break;
    }
    case 20: {
      if (sub_state < 3) {
        s17_target_g += pow10(4-sub_state);
        if (s17_target_g > 42000) s17_target_g = 42000;
      } 
      else s20_target_ok = !s20_target_ok;
      break;
    }
    case 21: {
      if (sub_state < 3) {
        s17_tara_offset_g += pow10(3-sub_state);
        if (s17_tara_offset_g > 9990) s17_tara_offset_g = 9990;
      } 
      else s21_tara_offset_ok = !s21_tara_offset_ok;
      break;
    }
    case 22: {
      sub_state = (sub_state+1) % 3;
      break;
    }
    case 25: {
      if (sub_state < 3) {
        s22_target_g += pow10(4-sub_state);
        if (s22_target_g > 42000) s22_target_g = 42000;
      } 
      else s22_target_ok = !s22_target_ok;
      break;
    }
    case 26: {
      sub_state = (sub_state+1) % 6;
      break;
    }
    case 27: {
      sub_state = (sub_state+1) % 2;
      break;
    }
    default: {
      redraw_screen = false;
      beep = false;
    }
  }

  if (beep && use_keytones) tone(PIN_BEEP, BEEP_FREQ_RIGHT, BEEP_LENGTH_TURN);
  
  #ifdef SERIAL_ENABLED
  Serial.println("Rotary turned right.");
  #endif
}

void onTurnLeft() {
  static bool beep;
  beep = true;
  redraw_screen = true;

  switch (state) {
    case 6: {
      if (sub_state < 4) {
        s6_known_mass_g -= pow10(4-sub_state);
        if (s6_known_mass_g < 10) s6_known_mass_g = 10;
      } 
      else s6_known_mass_ok = !s6_known_mass_ok;
      break;
    }
    case 7: 
    case 10: {
      sub_state = (sub_state+1) % 2;
      break;
    }
    case 12:
    case 17: {
      sub_state = (sub_state+2) % 3;
      break;
    }    
    case 15: {
      if (sub_state < 3) {
        s12_target_g -= pow10(4-sub_state);
        if (s12_target_g < 100) s12_target_g = 100;
      } 
      else s15_target_ok = !s15_target_ok;
      break;
    }
    case 16: {
      if (sub_state < 3) {
        s12_tara_offset_g -= pow10(3-sub_state);
        if (s12_tara_offset_g < 10) s12_tara_offset_g = 10;
      } 
      else s16_tara_offset_ok = !s16_tara_offset_ok;
      break;
    }
    case 20: {
      if (sub_state < 3) {
        s17_target_g -= pow10(4-sub_state);
        if (s17_target_g < 100) s17_target_g = 100;
      } 
      else s20_target_ok = !s20_target_ok;
      break;
    }
    case 21: {
      if (sub_state < 3) {
        s17_tara_offset_g -= pow10(3-sub_state);
        if (s17_tara_offset_g < 10) s17_tara_offset_g = 10;
      } 
      else s21_tara_offset_ok = !s21_tara_offset_ok;
      break;
    }
    case 22: {
      sub_state = (sub_state+2) % 3;
      break;
    }
    case 25: {
      if (sub_state < 3) {
        s22_target_g -= pow10(4-sub_state);
        if (s22_target_g < 100) s22_target_g = 100;
      } 
      else s22_target_ok = !s22_target_ok;
      break;
    }
    case 26: {
      sub_state = (sub_state+5) % 6;
      break;
    }
    case 27: {
      sub_state = (sub_state+1) % 2;
      break;
    }
    default: {
      redraw_screen = false;
      beep = false;
    }
  }

  if (beep && use_keytones) tone(PIN_BEEP, BEEP_FREQ_LEFT, BEEP_LENGTH_TURN);
  
  #ifdef SERIAL_ENABLED
  Serial.println("Rotary turned left.");
  #endif
}

void shortClick_enc(bool beep = true) {
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
    case 12: {
      switch(sub_state) {
        case 0: {
          // Starten nur mit langem Klick möglich!
          beep = false; 
          break; 
        }
        case 1: {
          stateTransition(16);
          break;
        }
        case 2: {
          stateTransition(15);
          break;
        }
      }
      break;
    }
    case 17: {
      switch(sub_state) {
        case 0: { 
          // Starten nur mit langem Klick möglich!
          beep = false; 
          break; 
        }
        case 1: { 
          stateTransition(21); 
          break; 
          }
        case 2: {
          stateTransition(20);
          break;
        }
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
          // Starten nur mit langem Klick möglich!
          beep = false;
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
    default: {
      beep = false;
      break;
    }
  }
  
  if (beep && use_keytones) tone(PIN_BEEP, BEEP_FREQ_CLICK, BEEP_LENGTH_SHORT);
  
  #ifdef SERIAL_ENABLED
  Serial.println("Rotary button short click.");
  #endif
}

// das ist nötig, um die Signatur von CtrlBtn::CallbackFunction {aka void (*)()} zu erfüllen.
void shortClick_enc() {
  shortClick_enc(true);
}

void longClick_enc() {
  static bool beep;
  beep = true;

  switch (state) {
    case 4:
    case 5:
    case 9:
    case 13:
    case 14:
    case 18:
    case 19:
    case 23: 
    case 24: {
      // in diesen Fällen ist lang-Klick äquivalent zu kurz-Klick
      shortClick_enc(false);
      break;
    }
    case 6: {
      sub_state = 4;
      shortClick_enc(false);
      break;
    }
    case 12: {
      if (sub_state == 0) stateTransition(13);
      break;
    }
    case 15:
    case 16:
    case 20:
    case 21:
    case 25: {
      // in diesen Fällen ist lang-Klick äquivalent zu "speichern" bei 3stelligen Eingaben
      sub_state = 3;
      shortClick_enc(false);
      break;
    }
    case 17: {
      if (sub_state == 0) stateTransition(18);
      break;
    }
    case 22: {
      if (sub_state == 0) stateTransition(23);
      break;
    }
    default: {
      beep = false;
      break;
    }
  }
  
  if (beep && use_keytones) tone(PIN_BEEP, BEEP_FREQ_CLICK, BEEP_LENGTH_LONG);
  
  #ifdef SERIAL_ENABLED
  Serial.println("Rotary button long click.");
  #endif
}

// Hardware aus Libraries
CtrlEnc   encoder   (PIN_ENCODER_CLK, PIN_ENCODER_DAT, onTurnRight, onTurnLeft);
CtrlBtn   btn_enc   (PIN_ENCODER_BTN, 20, nullptr, shortClick_enc, longClick_enc);

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

  // Eigene Zeichen für das Display
  uint8_t cursor[8] = {0b10000,0b11000,0b11100,0b11110,0b11100,0b11000,0b10000,0};
  uint8_t check[8] = {0,0b00001,0b00011,0b00010,0b10110,0b11100,0b01000,0};
  uint8_t cross[8] = {0,0b10001,0b11011,0b01110,0b01110,0b11011,0b10001,0};
  uint8_t infty[8] = {0,0,0b01010,0b10101,0b10101,0b01010,0,0};
  uint8_t back[8] = {0b00100,0b01000,0b11110,0b01001,0b00101,0b00001,0b00110,0};
  // uint8_t up[8] = {0b00100,0b01110,0b11111,0,0,0,0,0};                                 // z.Zt. nicht benötigt
  // uint8_t down[8] = {0,0,0,0,0,0b11111,0b01110,0b00100};

  lcd.begin();
  lcd.backlight();
  lcd.createChar(0, cursor);
  lcd.createChar(1, check);
  lcd.createChar(2, cross);
  lcd.createChar(3, infty);
  lcd.createChar(4, back);
  // lcd.createChar(5, up);
  // lcd.createChar(6, down);


  // Startbildschirm
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F(" Weight-O-Matic "));
  lcd.setCursor(0,1);
  lcd.print(VERSION);
  lcd.setCursor(6,1);
  lcd.print("Starte...");
  lcd.setCursor(15,1);
  lcd.blink();


  // Prüfen, ob im EEPROM Werte für Kalibrierung und Tara gespeichert sind. Wenn nicht, Standardwerte laden:
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

  // ...das Gleiche für die EInstellungen:
  EEPROM.get(addr_settings_saved_flag, saved_flag);
  if (saved_flag != 1) {
    EEPROM.put(addr_toggle_settings, DEFAULT_TOGGLESETTINGS);
    EEPROM.put(addr_settings_saved_flag, (uint8_t)1);
    #ifdef SERIAL_ENABLED
    Serial.println(F("Keine gespeicherten allg. Einstellungen im EEPROM gefunden, Standardwerte geladen!"));
    #endif
  }

  // ...für Voreinstellung 1:
  EEPROM.get(addr_p1_saved_flag, saved_flag);
  if (saved_flag != 1) {
    EEPROM.put(addr_p1_target, DEFAULT_P1_TARGET);
    EEPROM.put(addr_p1_offset, DEFAULT_P1_OFFSET);
    EEPROM.put(addr_p1_saved_flag, (uint8_t)1);
    #ifdef SERIAL_ENABLED
    Serial.println(F("Keine gespeicherten Einstellungen für VE 1 im EEPROM gefunden, Standardwerte geladen!"));
    #endif
  }

  // ...für Voreinstellung 2:
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
  stateTransition(1);

  // Kalibrierungseinstellungen aus EEPROM laden
  long tar_offset = 0;
  float cal_value = 0.0f;
  EEPROM.get(addr_tar_value, tar_offset);
  EEPROM.get(addr_cal_value, cal_value);
  loadcell.setTareOffset(tar_offset);
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

  // Wiegezelle initialisieren - 2000ms Startzeit auf Empfehlung der Library
  loadcell.begin();
  loadcell.start(2000, false);

  // Prüfen ob eine Verbindung zur Wiegezelle besteht
  if (loadcell.getTareTimeoutFlag() || loadcell.getSignalTimeoutFlag()) {
    #ifdef SERIAL_ENABLED
    Serial.println(F("Fehler bei der Verbindung MCU <-> HX711."));
    #endif
    stateTransition(2);
    return;
  }
  else {
    loadcell.setCalFactor(cal_value);
    #ifdef SERIAL_ENABLED
    Serial.println(F("Wiegezelle erfolgreich initialisiert."));
    #endif
  }
  while (!loadcell.update());

  // Übergang zur loop, mit Zustand, der den Schalter ausliest
  stateTransition(11);
}

bool break_loop = false;
void loop() {
  static uint32_t t;
  t = millis();

  static uint32_t t_switch = 0;
  static uint32_t t_last_weight_reading = t;
  static uint32_t t_screen = 0;
  static uint32_t t_tone_started = 0;

  static float loadcell_reading = 0.0f;
  
  break_loop = false;

  // Messwert aus Wiegezelle auslesen
  if (loadcell.update()) {
    t_last_weight_reading = t;
    loadcell_reading = loadcell.getData();
    if (loadcell_reading < -65536) current_weight_g = -65536;
    else if (loadcell_reading > 65536) current_weight_g = 65536;
    else if (current_weight_g != (long)loadcell_reading) {
      current_weight_g = (long)loadcell_reading;
      redraw_screen = true;
    }
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
      if (current_weight_g - s12_tara_offset_g >= s12_target_g) stateTransition(14, 1);
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
      if (current_weight_g - s17_tara_offset_g >= s17_target_g) stateTransition(19, 1); 
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
      if (current_weight_g >= s22_target_g) stateTransition(24, 1);
      else if (!output_enabled && current_weight_g < s22_target_g) {
        t_last_target_started = t;
        last_target_g = s22_target_g;
        enableOutput();
      }
      break;
    }
    case 14:
    case 19:
    case 24: { // in diesen Zuständen ist die Dosierung regulär beendet und es wird die "Ende-Musik" gespielt. Könnte man sicherlich schöner programmieren! ;)
      if (sub_state > 0 && use_endtone) {
        switch (sub_state) {
          case 1: { t_tone_started = t; tone(PIN_BEEP, BEEP_FREQ_A5, BEEP_UNIT_LENGTH); sub_state++; break; }
          case 2: { if (t-t_tone_started>=BEEP_UNIT_LENGTH*4) {t_tone_started = t; tone(PIN_BEEP, BEEP_FREQ_F5, BEEP_UNIT_LENGTH); sub_state++;} break; }
          case 3: { if (t-t_tone_started>=BEEP_UNIT_LENGTH*2) {t_tone_started = t; tone(PIN_BEEP, BEEP_FREQ_F5, BEEP_UNIT_LENGTH); sub_state++;} break; }
          case 4: { if (t-t_tone_started>=BEEP_UNIT_LENGTH*2) {t_tone_started = t; tone(PIN_BEEP, BEEP_FREQ_G5, BEEP_UNIT_LENGTH); sub_state++;} break; }
          case 5: { if (t-t_tone_started>=BEEP_UNIT_LENGTH*4) {t_tone_started = t; tone(PIN_BEEP, BEEP_FREQ_F5, BEEP_UNIT_LENGTH); sub_state++;} break; }
          case 6: { if (t-t_tone_started>=BEEP_UNIT_LENGTH*8) {t_tone_started = t; tone(PIN_BEEP, BEEP_FREQ_A5, BEEP_UNIT_LENGTH); sub_state++;} break; }
          case 7: { if (t-t_tone_started>=BEEP_UNIT_LENGTH*4) {t_tone_started = t; tone(PIN_BEEP, BEEP_FREQ_B5, BEEP_UNIT_LENGTH*4); sub_state++;} break; }          
          case 8: { if (t-t_tone_started>=BEEP_UNIT_LENGTH*16) {t_tone_started = t; tone(PIN_BEEP, BEEP_FREQ_A5*2, BEEP_UNIT_LENGTH); sub_state++;} break; }
          case 9: { if (t-t_tone_started>=BEEP_UNIT_LENGTH*4) {t_tone_started = t; tone(PIN_BEEP, BEEP_FREQ_F5*2, BEEP_UNIT_LENGTH); sub_state++;} break; }
          case 10: { if (t-t_tone_started>=BEEP_UNIT_LENGTH*2) {t_tone_started = t; tone(PIN_BEEP, BEEP_FREQ_F5*2, BEEP_UNIT_LENGTH); sub_state++;} break; }
          case 11: { if (t-t_tone_started>=BEEP_UNIT_LENGTH*2) {t_tone_started = t; tone(PIN_BEEP, BEEP_FREQ_G5*2, BEEP_UNIT_LENGTH); sub_state++;} break; }
          case 12: { if (t-t_tone_started>=BEEP_UNIT_LENGTH*4) {t_tone_started = t; tone(PIN_BEEP, BEEP_FREQ_F5*2, BEEP_UNIT_LENGTH); sub_state++;} break; }
          case 13: { if (t-t_tone_started>=BEEP_UNIT_LENGTH*8) {t_tone_started = t; tone(PIN_BEEP, BEEP_FREQ_A5*2, BEEP_UNIT_LENGTH); sub_state++;} break; }
          case 14: { if (t-t_tone_started>=BEEP_UNIT_LENGTH*4) {t_tone_started = t; tone(PIN_BEEP, BEEP_FREQ_B5*2, BEEP_UNIT_LENGTH*4); sub_state++;} break; }
          case 15: {
            if (t-t_tone_started>=BEEP_UNIT_LENGTH*32 && endtone_repetitions_done < BEEP_END_REPETITIONS) {
              sub_state = 1;
              endtone_repetitions_done++;
            } else if (endtone_repetitions_done >= BEEP_END_REPETITIONS) {
              sub_state = 0;
              endtone_repetitions_done = 0;
            }
            break;
          }
        }
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
      loadcell.update();
      loadcell.tare();
      long tar_value = loadcell.getTareOffset();
      loadcell.setTareOffset(tar_value);
      #ifdef SERIAL_ENABLED
      Serial.print(F("(41) Tara abgeschlossen. Neuer Wert für tara_offset: "));
      Serial.println(tar_value);
      #endif
      stateTransition(5);
      break;
    }
    case 61: {
      loadcell.update();
      loadcell.refreshDataSet();
      float cal_value = loadcell.getNewCalibration(s6_known_mass_g);
      #ifdef SERIAL_ENABLED
      Serial.print(F("(61) Kalibrierung abgeschlossen. Neuer Wert für cal_value: "));
      Serial.println(cal_value);
      #endif
      stateTransition(7);
      break;
    }
    case 71: {
      float cal_value = loadcell.getCalFactor();
      long tar_value = loadcell.getTareOffset();
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
      loadcell.tare();
      tar_value = loadcell.getTareOffset();
      loadcell.setTareOffset(tar_value);
      #ifdef SERIAL_ENABLED
      Serial.print(F("(91) Tara abgeschlossen. Neuer Wert für tara_offset: "));
      Serial.println(tar_value);
      #endif
      stateTransition(10);
      break;
    }
    case 101: {
      long tar_value = loadcell.getTareOffset();
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
}