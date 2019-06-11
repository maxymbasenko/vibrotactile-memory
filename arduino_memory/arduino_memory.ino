#include <Wire.h>
#include "Adafruit_DRV2605.h"
#include <SPI.h>                  
#include <MFRC522.h>              

#define TCAADDR 0x70              
#define RST_PIN   9     // SPI Reset Pin            RF
#define SS_PIN    10    // SPI Slave Select Pin     RF

namespace{
   const uint8_t HIGH_VAL = 120, MIDDLE_VAL = 90, LOW_VAL = 60;
   const uint8_t NUM_OF_MOTORS = 4; 
   uint32_t num_of_steps, value;
   uint8_t pattern_scale[NUM_OF_MOTORS][25] = { 
     {10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200,210,220,230,240,250}
    ,{10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200,210,220,230,240,250}
    ,{10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200,210,220,230,240,250}
    ,{10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200,210,220,230,240,250}
    };


    uint8_t pattern_pyramid[NUM_OF_MOTORS][6] = {
       {40 ,60  ,0  ,0 ,0 ,0},
       {0  ,60  ,80 ,100 ,120 ,0},
       {0  ,60  ,80 ,0 ,0 ,0},
       {40 ,60  ,0 ,0 ,0 ,0}
    };


    uint8_t pattern_location_0[NUM_OF_MOTORS][1] = {
       {HIGH_VAL},
       {0  },
       {0  },
       {0 }
    };

    uint8_t pattern_location_1[NUM_OF_MOTORS][1] = {
       {0  },
       {HIGH_VAL},
       {0  },
       {0 }
    };

    uint8_t pattern_location_2[NUM_OF_MOTORS][1] = {
       {0  },
       {0  },
       {HIGH_VAL},
       {0 }
    };

    uint8_t pattern_location_3[NUM_OF_MOTORS][1] = {
       {0  },
       {0  },
       {0 },
       {HIGH_VAL}
    };
    uint8_t pattern_falling_down[NUM_OF_MOTORS][1] = {
       {HIGH_VAL},
       {0  },
       {0  },
       {0 }
    };


    uint8_t pattern_rising_up[NUM_OF_MOTORS][7] = {
       {HIGH_VAL,MIDDLE_VAL ,0  ,0 ,0 ,0,0},
       {0  ,MIDDLE_VAL ,HIGH_VAL,MIDDLE_VAL,0 ,0,0},
       {0  ,0  ,0  ,MIDDLE_VAL,HIGH_VAL,MIDDLE_VAL,0},
       {0  ,0  ,0  ,0 ,0 ,MIDDLE_VAL,HIGH_VAL}
    };

    uint8_t pattern_rising_up_clear[NUM_OF_MOTORS][4] = {
       {HIGH_VAL,0 ,0  ,0},
       {0  ,HIGH_VAL ,0,0},
       {0  ,0 ,HIGH_VAL,0},
       {0  ,0  ,0 ,HIGH_VAL}
       };

    uint8_t pattern_rising_down[NUM_OF_MOTORS][7] = {
       {0  ,0  ,0  ,0 ,0 ,MIDDLE_VAL,HIGH_VAL},
       {0  ,0  ,0  ,MIDDLE_VAL,HIGH_VAL,MIDDLE_VAL,0},
       {0  ,MIDDLE_VAL ,HIGH_VAL,MIDDLE_VAL,0 ,0,0},
       {HIGH_VAL,MIDDLE_VAL ,0  ,0 ,0 ,0,0}
    };

   uint8_t pattern[NUM_OF_MOTORS][12] = { 
        {180,180,0,0,0,0,0,0,0,0,180,180}
       ,{0,0,180,180,0,0,0,0,0,0,180,180}
       ,{0,0,0,0,180,180,0,0,0,0,180,180}
       ,{0,0,0,0,0,0,180,180,0,0,180,180}
   } ;

    uint8_t pattern_collecting_at_top[NUM_OF_MOTORS][21] = {
       {LOW_VAL,LOW_VAL ,0  ,0 ,0 ,0,0,LOW_VAL,LOW_VAL ,0  ,0 ,0 ,0,0,LOW_VAL,LOW_VAL ,0  ,0 ,0 ,0,0},
       {0  ,LOW_VAL ,LOW_VAL,LOW_VAL,0 ,0,0,0  ,LOW_VAL ,LOW_VAL,LOW_VAL,0 ,0,0,0  ,LOW_VAL ,LOW_VAL,LOW_VAL,0 ,0,0},
       {0  ,0  ,0  ,LOW_VAL,LOW_VAL,LOW_VAL,0,0  ,0  ,0  ,LOW_VAL,LOW_VAL,LOW_VAL,0,0  ,0  ,0  ,LOW_VAL,LOW_VAL,LOW_VAL,0},
       {0  ,0  ,0  ,0 ,0 ,LOW_VAL,LOW_VAL,LOW_VAL  ,LOW_VAL  ,LOW_VAL  ,LOW_VAL ,LOW_VAL ,LOW_VAL,MIDDLE_VAL,MIDDLE_VAL  ,MIDDLE_VAL  ,MIDDLE_VAL  ,MIDDLE_VAL ,MIDDLE_VAL ,MIDDLE_VAL,HIGH_VAL}
    };

   uint8_t pattern_dance[NUM_OF_MOTORS][12] = { 
       {0,0,0,0,0,0,0,0,0,0,0,0}
       ,{0,0,0,0,0,0,0,0,0,0,0,0}
       ,{0,0,0,0,0,0,0,0,0,0,0,0}
       ,{0,0,0,0,0,0,0,0,0,0,0,0}
   } ;

   uint8_t m = 0;
   uint32_t t_0, t_1;
   uint8_t time_frame;

   Adafruit_DRV2605 m_ctrl;
   Adafruit_DRV2605 m_ctrls[NUM_OF_MOTORS];
   MFRC522 mfrc522(SS_PIN, RST_PIN);   // Instanz des MFRC522 erzeugen   RF

   byte uids[16][7] = {
        { 0x04,0x20,0x39,0xD2,0xF2,0x62,0x81}, //1
        { 0x04,0x58,0x8A,0xD2,0xF2,0x62,0x81}, //2 

        { 0x04,0x25,0x7D,0xD2,0xF2,0x62,0x80}, //...
        { 0x04,0x22,0xC0,0xCA,0xF3,0x59,0x80},

        { 0x04,0x25,0xB9,0xCA,0xF3,0x59,0x80},
        { 0x04,0x2D,0xC1,0xCA,0xF3,0x59,0x80},

        { 0x04,0x2B,0xC7,0xCA,0xF3,0x59,0x80},
        { 0x04,0x2E,0xDE,0xCA,0xF3,0x59,0x80},

        { 0x04,0x56,0x78,0xD2,0xF2,0x62,0x80},
        { 0x04,0x59,0x3A,0xD2,0xF2,0x62,0x81},

        { 0x04,0x58,0xA0,0xD2,0xF2,0x62,0x81},
        { 0x04,0x0F,0x21,0xD2,0xF2,0x62,0x81},

        { 0x04,0x0F,0x5F,0xD2,0xF2,0x62,0x81},
        { 0x04,0x2B,0x25,0xD2,0xF2,0x62,0x81},

        { 0x04,0x0F,0x68,0xD2,0xF2,0x62,0x81},
        { 0x04,0x0F,0x42,0xD2,0xF2,0x62,0x81}, //16
    };
    };

void tcaselect(uint8_t i) {             //MP
    if (i > 7) return;                    //MP

    Wire.beginTransmission(TCAADDR);        //MP
    Wire.write(1 << i);                     //MP
    Wire.endTransmission();                 //MP
}

void setup(){
    //while(!Serial);
    delay(1000);
    Serial.begin(9600);

    Serial.println("Program started!");
//for (uint8_t i = 0; i < NUM_OF_MOTORS; i++)
//{
//     tcaselect(i);
//    // m_ctrl.begin();
//    m_ctrls[i].begin();
//    m_ctrls[i].selectLibrary(1);
//    m_ctrls[i].setMode(DRV2605_MODE_REALTIME);
//}
  
  //m_ctrl.begin();
  //Serial.println("m_ctrl.begin() executed");
  //m_ctrl.selectLibrary(1);
  //Serial.println("m_ctrl.selectLibrary(1) executed");
  //m_ctrl.setMode(DRV2605_MODE_REALTIME);
  //Serial.println("Set to Realtime Mode");
    
    Serial.println("Motor Controller initiated!");
    SPI.begin();        // Initialisiere SPI Kommunikation  RF
    mfrc522.PCD_Init();  // Initialisiere MFRC522 Lesemodul  RF
    Serial.println("READY!");
}

uint8_t is_scanned_tag(byte tag[7]){

    for (int j=0; j<7; j++) {
        if (mfrc522.uid.uidByte[j] != tag[j]) {
            return false;
        } 
    }
    return true;
}

uint8_t get_scanned_tag_index(){
        for(uint8_t tag = 0; 0 < 16; tag++ ){
            if(is_scanned_tag(uids[tag])){
                return tag;
            }
        }
        return NULL;
}

void seq_run_p(uint8_t *pattern, uint8_t num_of_motors, uint8_t num_of_steps, uint32_t step_length_ms, uint8_t iterations){
    for(uint8_t iteration = 0; iteration < iterations; iteration++){
        for ( uint32_t step = 0; step < num_of_steps; step++) {
            t_0 = millis();
            for (uint8_t motor = 0; motor < num_of_motors; motor++) {
                    tcaselect(motor);
                    value = *((pattern+motor*num_of_steps) + step);

                    Serial.print("M");
                    Serial.print(motor);
                    Serial.print(" w/ Val");
                    Serial.println(value);

                    m_ctrls[motor].setRealtimeValue(value);
                    //m_ctrls[motor].setWaveform(0,value);
                    //m_ctrls[motor].setWaveform(1,0);
                    m_ctrls[motor].go();
            } 
                t_1 = millis();
                while(t_1-t_0 < step_length_ms){
                    t_1 = millis();
                }
        }
    }
        Serial.println("RESETTING MOTORS!");
        for (uint8_t motor = 0; motor < NUM_OF_MOTORS; motor++) {
                tcaselect(motor);
                value = 0;
                m_ctrls[motor].setRealtimeValue(value);
                m_ctrls[motor].setWaveform(0,value);
                m_ctrls[motor].setWaveform(1,0);
                m_ctrls[motor].go();
        } 
}

void print_rfid_uid(){
    if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial() ) {
        Serial.print("Gelesene UID:");
        for (byte i = 0; i < mfrc522.uid.size; i++) {
            Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
            Serial.print(mfrc522.uid.uidByte[i], HEX);
        } 
        Serial.println();
    }
}

void loop(){
    //Needed for "Resetting" Motor 0
    //tcaselect(0);
    //value = 130;
    //m_ctrls[0].setRealtimeValue(value);
    //m_ctrls[0].setWaveform(0,value);
    //m_ctrls[0].setWaveform(1,0);
    //m_ctrls[0].go();

    //print_rfid_uid();

      // PICC = proximity integrated circuit card = kontaktlose Chipkarte
    if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial() ) {
        Serial.println("SEARCHING FOR CARD");
        switch (get_scanned_tag_index())
        {
        case 0:
            Serial.println("CARD 1!");
            seq_run_p((uint8_t*)pattern_scale,4,25,500,1);
            /* code */
            break;
        case 1:
            Serial.println("CARD 2!");
            for (uint8_t i = 0; i < 3; i++)
            {
                seq_run_p((uint8_t*)pattern_location_0,4,1,100,1);
                delay(200);
            }
            break;
        case 2:
            Serial.println("CARD 3!");
            for (uint8_t i = 0; i < 3; i++)
            {
                seq_run_p((uint8_t*)pattern_location_0,4,1,250,1);
                seq_run_p((uint8_t*)pattern_rising_up_clear,4,4,250,1);
                delay(200);
            }
            break;
        case 3:
            Serial.println("CARD 4!");
            for (uint8_t i = 0; i < 3; i++)
            {
                seq_run_p((uint8_t*)pattern_location_1,4,1,100,1);
                delay(200);
            }
            break;
        case 4:
            Serial.println("CARD 5!");
            for (uint8_t i = 0; i < 5; i++)
            {
                seq_run_p((uint8_t*)pattern_location_0,4,1,100,1);
                seq_run_p((uint8_t*)pattern_rising_up,4,7,100,1);
                seq_run_p((uint8_t*)pattern_location_3,4,1,100,1);
                delay(250);
                seq_run_p((uint8_t*)pattern_location_3,4,1,100,1);
                seq_run_p((uint8_t*)pattern_rising_down,4,7,100,1);
                seq_run_p((uint8_t*)pattern_location_0,4,1,100,1);
                delay(250);
            }
            /* code */
            break;
        case 5:
            Serial.println("CARD 6!");
                seq_run_p((uint8_t*)pattern_collecting_at_top,4,21,100,1);
            /* code */
            break;
        case 6:
            /* code */
            Serial.println("CARD 7!");
            break;
        case 7:
            Serial.println("CARD 8!");
            /* code */
            break;
        case 8:
            Serial.println("CARD 9!");
            /* code */
            break;
        case 9:
            Serial.println("CARD 10!");
            /* code */
            break;
        case 10:
            Serial.println("CARD 11!");
            /* code */
            break;
        case 11:
            Serial.println("CARD 12!");
            /* code */
            break;
        case 12:
            Serial.println("CARD 13!");
            /* code */
            break;
        case 13:
            Serial.println("CARD 14!");
            /* code */
            break;
        case 14:
            Serial.println("CARD 15!");
            /* code */
            break;
        case 15:
            Serial.println("CARD 16!");
            /* code */
            break;
        default:
            Serial.println("CARD NOT FOUND!");
            break;
        }

    } 
    // Versetzt die gelesene Karte in einen Ruhemodus, um nach anderen Karten suchen zu können.
    mfrc522.PICC_HaltA();
  }