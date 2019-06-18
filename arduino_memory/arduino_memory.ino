#include <Wire.h>
#include "Adafruit_DRV2605.h"
#include <SPI.h>                  
#include <MFRC522.h>              

#define TCAADDR 0x70              
#define RST_PIN   9     // SPI Reset Pin            RF
#define SS_PIN    10    // SPI Slave Select Pin     RF
#define SERIAL 1
namespace{
   const uint8_t HIG = 120, MID = 90, LOW_V = 60;
   const uint8_t NUM_OF_MOTORS = 4; 

   uint8_t m = 0;
   uint32_t t_0, t_1;
   uint8_t time_frame;

   Adafruit_DRV2605 m_ctrl;
   Adafruit_DRV2605 m_ctrls[NUM_OF_MOTORS];
   MFRC522 mfrc522(SS_PIN, RST_PIN);   // Instanz des MFRC522 erzeugen   RF
   uint32_t num_of_steps, value;
    uint8_t pattern_scale[NUM_OF_MOTORS][25] = { 
        {10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,HIG,190,200,210,220,230,240,250}
        ,{10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,HIG,190,200,210,220,230,240,250}
        ,{10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,HIG,190,200,210,220,230,240,250}
        ,{10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,HIG,190,200,210,220,230,240,250}
        };
    uint8_t pattern_collecting_at_top[NUM_OF_MOTORS][21] = {
        {LOW_V,LOW_V,0  ,0  ,0  ,0  ,0  ,LOW_V,LOW_V,0  ,0  ,0  ,0  ,0  ,LOW_V,LOW_V,0  ,0  ,0  ,0  ,0},
        {0  ,LOW_V,LOW_V,LOW_V,0  ,0  ,0  ,0  ,LOW_V,LOW_V,LOW_V,0  ,0  ,0  ,0  ,LOW_V,LOW_V,LOW_V,0  ,0  ,0},
        {0  ,0  ,0  ,LOW_V,LOW_V,LOW_V,0  ,0  ,0  ,0  ,LOW_V,LOW_V,LOW_V,0  ,0  ,0  ,0  ,LOW_V,LOW_V,LOW_V,0},
        {0  ,0  ,0  ,0  ,0  ,LOW_V,LOW_V,LOW_V,LOW_V,LOW_V,LOW_V,LOW_V,LOW_V,MID,MID,MID,MID,MID,MID,MID,HIG}
        };

    uint8_t pattern_rising_up[NUM_OF_MOTORS][7] = {
       {HIG ,MID ,0   ,0   ,0  ,0   ,0},
       {0   ,MID ,HIG ,MID ,0  ,0   ,0},
       {0   ,0   ,0   ,MID ,HIG,MID ,0},
       {0   ,0   ,0   ,0   ,0  ,MID ,HIG}
    };

    uint8_t pattern_collect[NUM_OF_MOTORS][11] = {
       {HIG ,0  ,0   ,0  ,HIG,0  ,0  ,HIG,0  ,HIG,HIG},
       {0   ,HIG,0   ,0  ,0  ,HIG,0  ,0  ,HIG,HIG,HIG},
       {0   ,0  ,HIG ,0  ,0  ,0  ,HIG,HIG,HIG,HIG,HIG},
       {0   ,0  ,0   ,HIG,HIG,HIG,HIG,HIG,HIG,HIG,HIG}
    };

    uint8_t pattern_double_rise[NUM_OF_MOTORS][9] = {
       {HIG ,0  ,0   ,0  ,0,HIG,0  ,0   ,0},
       {0   ,HIG,0   ,0  ,0,0  ,HIG,0   ,0},
       {0   ,0  ,HIG ,0  ,0,0  ,0  ,HIG ,0},
       {0   ,0  ,0   ,HIG,0,0  ,0  ,0   ,HIG}
    };

    uint8_t pattern[NUM_OF_MOTORS][12] = { 
         {HIG,HIG,0,0,0,0,0,0,0,0,HIG,HIG}
        ,{0,0,HIG,HIG,0,0,0,0,0,0,HIG,HIG}
        ,{0,0,0,0,HIG,HIG,0,0,0,0,HIG,HIG}
        ,{0,0,0,0,0,0,HIG,HIG,0,0,HIG,HIG}
    };
    uint8_t pattern_updown[NUM_OF_MOTORS][13] = { 
        {0 ,20 ,40 ,60 ,80 ,100 ,120 ,100 ,80 ,60 ,40 ,20 ,0 },
        {0 ,20 ,40 ,60 ,80 ,100 ,120 ,100 ,80 ,60 ,40 ,20 ,0 },
        {0 ,20 ,40 ,60 ,80 ,100 ,120 ,100 ,80 ,60 ,40 ,20 ,0 },
        {0 ,20 ,40 ,60 ,80 ,100 ,120 ,100 ,80 ,60 ,40 ,20 ,0 }
    };
   uint8_t pattern_inout[NUM_OF_MOTORS][6] = { 
        {HIG,0  ,HIG,0  ,HIG,0}
       ,{0  ,HIG,0  ,HIG,0  ,HIG}
       ,{0  ,HIG,0  ,HIG,0  ,HIG}
       ,{HIG,0  ,HIG,0  ,HIG,0}
   };

   uint8_t pattern_inout_double[NUM_OF_MOTORS][8] = { 
        {HIG,HIG,0  ,0  ,HIG,HIG,0  ,0  }
       ,{0  ,0  ,HIG,HIG,0  ,0  ,HIG,HIG}
       ,{0  ,0  ,HIG,HIG,0  ,0  ,HIG,HIG}
       ,{HIG,HIG,0  ,0  ,HIG,HIG,0  ,0  }
   };

    uint8_t pattern_pulse[NUM_OF_MOTORS][5] = {
       {0   ,MID  ,HIG  ,MID ,0},
       {MID ,HIG  ,HIG  ,HIG ,MID},
       {MID ,HIG  ,HIG  ,HIG ,MID},
       {0   ,MID  ,HIG  ,MID ,0}
    };
    uint8_t pattern_zigzag[NUM_OF_MOTORS][5] = {
       {HIG ,0   ,HIG ,0   ,HIG},
       {0   ,HIG ,0   ,HIG ,0  },
       {HIG ,0   ,HIG ,0   ,HIG},
       {0   ,HIG ,0   ,HIG ,0  }
    };
    uint8_t HI7 = HIG/7;

    uint8_t pattern_sidewave[NUM_OF_MOTORS][7] = {
       {HI7  ,HI7*2,HI7*3,HI7*4,HI7*5},
       {HI7*2,HI7*3,HI7*4,HI7*5,HI7*6},
       {HI7*3,HI7*4,HI7*5,HI7*6,HIG},
       {HI7*4,HI7*5,HI7*6,HIG ,0  }
    };
    uint8_t pattern_every_other[NUM_OF_MOTORS][5] = {
       {HIG ,0 ,HIG ,0 ,HIG},
       {HIG ,0 ,HIG ,0 ,HIG},
       {HIG ,0 ,HIG ,0 ,HIG},
       {HIG ,0 ,HIG ,0 ,HIG}
    };



    
    uint8_t pattern_pyramid[NUM_OF_MOTORS][6] = {
       {40 ,60  ,0  ,0  ,0  ,0},
       {0  ,60  ,80 ,100,120,0},
       {0  ,60  ,80 ,0  ,0  ,0},
       {40 ,60  ,0  ,0  ,0  ,0}
    };



    uint8_t pattern_location_0[NUM_OF_MOTORS][1] = {
       {HIG},
       {0  },
       {0  },
       {0 }
    };

    uint8_t pattern_location_1[NUM_OF_MOTORS][1] = {
       {0  },
       {HIG},
       {0  },
       {0 }
    };

    uint8_t pattern_location_2[NUM_OF_MOTORS][1] = {
       {0  },
       {0  },
       {HIG},
       {0 }
    };

    uint8_t pattern_location_3[NUM_OF_MOTORS][1] = {
       {0  },
       {0  },
       {0 },
       {HIG}
    };
    uint8_t pattern_falling_down[NUM_OF_MOTORS][1] = {
       {HIG},
       {0  },
       {0  },
       {0 }
    };



    uint8_t pattern_rising_up_clear[NUM_OF_MOTORS][4] = {
       {HIG,0 ,0  ,0},
       {0  ,HIG ,0,0},
       {0  ,0 ,HIG,0},
       {0  ,0  ,0 ,HIG}
       };

    uint8_t pattern_rising_down[NUM_OF_MOTORS][7] = {
       {0  ,0  ,0  ,0 ,0 ,MID,HIG},
       {0  ,0  ,0  ,MID,HIG,MID,0},
       {0  ,MID ,HIG,MID,0 ,0,0},
       {HIG,MID ,0  ,0 ,0 ,0,0}
    };


   uint8_t pattern_dance[NUM_OF_MOTORS][12] = { 
       {0,0,0,0,0,0,0,0,0,0,0,0}
       ,{0,0,0,0,0,0,0,0,0,0,0,0}
       ,{0,0,0,0,0,0,0,0,0,0,0,0}
       ,{0,0,0,0,0,0,0,0,0,0,0,0}
   } ;


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
    if(SERIAL){
    Serial.begin(9600);
    Serial.println("Program started!");
    }

    if(SERIAL){
    Serial.begin(9600);
    Serial.println("Program started!");
    Serial.println("Starting Motor init");
    }

  for (uint8_t i = 0; i < NUM_OF_MOTORS; i++)
  {
    if(SERIAL){
        Serial.print("Motor ");
        Serial.print(i);
        Serial.println(" initiation begins!");
    }
      tcaselect(i);
      m_ctrls[i].begin();
      m_ctrls[i].selectLibrary(1);
      m_ctrls[i].setMode(DRV2605_MODE_REALTIME);
    if(SERIAL){
        Serial.print("Motor ");
        Serial.print(i);
        Serial.println(" initiated!");
    }
  }
    SPI.begin();        // Initialisiere SPI Kommunikation  RF
    mfrc522.PCD_Init();  // Initialisiere MFRC522 Lesemodul  RF
    if(SERIAL){
        Serial.println("Motor Controller initiated!");
        Serial.println("RDID  initiated!");
        Serial.println("READY!");
    }
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

                if(SERIAL){
                    Serial.print("M");
                    Serial.print(motor);
                    Serial.print(" w/ Val");
                    Serial.println(value);
                }

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
        if(SERIAL){
            Serial.println("RESETTING MOTORS!");
        }
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
    if (SERIAL && mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial() ) {
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
//    tcaselect(0);
//    value = 130;
//    m_ctrls[0].setRealtimeValue(value);
//    m_ctrls[0].setWaveform(0,value);
//    m_ctrls[0].setWaveform(1,0);
//    m_ctrls[0].go();

      // PICC = proximity integrated circuit card = kontaktlose Chipkarte
    if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial() ) {
        if(SERIAL){
            Serial.println("SEARCHING FOR CARD");
        }
        switch (get_scanned_tag_index())
        {
        case 0:
            if(SERIAL){
                Serial.println("CARD 1!");
            }
            //seq_run_p((uint8_t*)pattern_scale,4,25,500,1);
            for (uint8_t i = 0; i < 5; i++)
            {
                seq_run_p((uint8_t*)pattern_location_0,4,1,100,1);
                delay(200);
            }
            /* code */
            break;
        case 1:
            if(SERIAL){
                Serial.println("CARD 2!");
            }
            for (uint8_t i = 0; i < 5; i++)
            {
                seq_run_p((uint8_t*)pattern_location_0,4,1,100,1);
                delay(200);
            }
            break;
        case 2:
            if(SERIAL){

                Serial.println("CARD 3!");
            }
            for (uint8_t i = 0; i < 2; i++)
            {
                seq_run_p((uint8_t*)pattern_location_0,4,1,250,1);
                seq_run_p((uint8_t*)pattern_rising_up_clear,4,4,250,1);
                delay(200);
            }
            break;
        case 3:
            if(SERIAL){

                Serial.println("CARD 4!");
            }
            for (uint8_t i = 0; i < 2; i++)
            {
                seq_run_p((uint8_t*)pattern_location_0,4,1,250,1);
                seq_run_p((uint8_t*)pattern_rising_up_clear,4,4,250,1);
                delay(200);
            }
            break;
        case 4:
            if(SERIAL){
                Serial.println("CARD 5!");
            }
            for (uint8_t i = 0; i < 2; i++)
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
            if(SERIAL){
                Serial.println("CARD 6!");
            }
            //    seq_run_p((uint8_t*)pattern_collecting_at_top,4,21,100,1);
            for (uint8_t i = 0; i < 2; i++)
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
        case 6:
            /* code */
            if(SERIAL){
                Serial.println("CARD 7!");
            }
            seq_run_p((uint8_t*)pattern_updown,4,12,100,1);
            break;
        case 7:
            if(SERIAL){
                Serial.println("CARD 8!");
            }
            seq_run_p((uint8_t*)pattern_updown,4,12,100,1);
            /* code */
            break;
        case 8:
            if(SERIAL){
                Serial.println("CARD 9!");
            }
            for (uint8_t i = 0; i < 5; i++)
            {
                seq_run_p((uint8_t*)pattern_location_1,4,1,100,1);
                delay(200);
            }
            //seq_run_p((uint8_t*)pattern_inout_double,4,8,100,1);
            break;
        case 9:
            if(SERIAL){
                Serial.println("CARD 10!");
            }
            
            for (uint8_t i = 0; i < 5; i++)
            {
                seq_run_p((uint8_t*)pattern_location_1,4,1,100,1);
                delay(200);
            }
            //seq_run_p((uint8_t*)pattern_pulse,4,5,300,1);
            /* code */
            break;
        case 10:
        if(SERIAL){

            Serial.println("CARD 11!");
        }
            //seq_run_p((uint8_t*)pattern_zigzag,4,5,500,1);
            seq_run_p((uint8_t*)pattern_sidewave,4,7,100,1);
            /* code */
            break;
        case 11:
        if(SERIAL){
            Serial.println("CARD 12!");
        }
            seq_run_p((uint8_t*)pattern_sidewave,4,7,100,1);
            /* code */
            break;
        case 12:
        if(SERIAL){
            Serial.println("CARD 13!");

        }
        for (uint8_t i = 0; i < 2; i++)
        {
            seq_run_p((uint8_t*)pattern_every_other,4,5,100,1);
        }
        
            /* code */
            break;
        case 13:
        if(SERIAL){

            Serial.println("CARD 14!");
        }
        for (uint8_t i = 0; i < 2; i++)
        {
            seq_run_p((uint8_t*)pattern_every_other,4,5,100,1);
        }
            /* code */
            break;
        case 14:
        if(SERIAL){
            Serial.println("CARD 15!");
        }
            seq_run_p((uint8_t*)pattern_inout,4,6,300,1);
            /* code */
            break;
        case 15:
        if(SERIAL){

            Serial.println("CARD 16!");
        }
            seq_run_p((uint8_t*)pattern_inout,4,6,300,1);
            /* code */
            break;
        default:
            if(SERIAL){
                Serial.println("CARD NOT FOUND!");
            }
            break;
        }
    }
    // Versetzt die gelesene Karte in einen Ruhemodus, um nach anderen Karten suchen zu können.
    mfrc522.PICC_HaltA();
  }