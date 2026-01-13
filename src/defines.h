//
//  Header.h
//  Charger21_Slave
//
//  Created by Ruedi Heimlicher on 05.08.2021.
//

#ifndef Header_h
#define Header_h

#define TIMER2_COMPA 0xA0 // 10ms
//#define TIMER2_DELAY 997   // 10ms mit OC2A 0xA0
#define TIMER2_DELAY 97
//#define LADEPORT  PORTC
//#define LADEDDR  DDRC

#define TEENSYVREF  3.30  // VRef 

#define MAX_STROM_L_BYTE   10
#define MAX_STROM_H_BYTE   11

#define  TEENSY_DATA        0xFC // Daten des teensy lesen

#define DATACOUNT_LO_BYTE  5
#define DATACOUNT_HI_BYTE  6

#define DATA_START_BYTE 8 // erstes Databyte im buffer und sendbuffer
// ADC

#define ADC_M              A0 // 14
#define ADC_O              A1 // 15
#define ADC_SHUNT          A2 // 16

#define ADC_BALANCE        A3 // 17

#define CHARGE_SET          18 // 18
#define CHARGE_RESET        19 // 18
#define DETECT_RESET        20 // 18

#define ADC_TEMP_SOURCE    A7 // 21
#define ADC_TEMP_BATT      A8 // 22

// Tasten
#define LOAD_START         18   // Laden START  Programm
#define LOAD_STOP          19    // Laden STOP  Programm

#define RESET_IN           20    // Eingaang fuer Meldung Reset

#define LOAD_MANUELL       8     // Laden mit Taste

#define CURR_UP            4  // Current UP
#define CURR_DOWN          5  // Current DOWN

//#define ADC_TIMERDELAY  4800

// hoststatus

#define FIRSTRUN  7
#define SEND_OK   1



#define ADC_U_BIT 0 // ISR: U messen
#define ADC_I_BIT 1 // ISR: I messen

#define U_MIN  3.0
#define U_OFF  2.5
#define U_MAX  4.2

#define RAW_U_FAKTOR
#define BATT_MIN_RAW 350 // Wert fuer Vergleich mit ADC
#define BATT_OFF_RAW 885 
#define BATT_MAX_RAW 880

//#define STROM_HI  1000 // mA
#define STROM_LO  50   // mA
#define STROM_REP  100 // Reparaturstrom bei Unterspannung

#define STROM_HI_RAW  850 // ADC-Wert
#define STROM_LO_RAW  50   // mA
#define STROM_REP_RAW  100 // Reparaturstrom bei Unterspannung

#define PWM_REP   100


#define STROM_FAKTOR 1080 // 


#define SHUNT_OFFSET 17    // Ruhestrom nach ADC

//#define ADC_U_FAKTOR 145
#define ADC_U_FAKTOR 147


#define STOM_PWM  23

// USB
#define USB_SEND  0 


//USB Bytes 
#define TASK_BYTE          0
#define DEVICECOUNT_BYTE   3


// logger
#define LOGGER_START       0xA0

#define LOGGER_CONT        0xA1

#define LOGGER_NEXT        0xA2

#define LOGGER_STOP        0xAF

#define LOGGER_SETTING     0xB0 // Setzen der Settings fuer die Messungen

#define MESSUNG_DATA       0xB1
#define MESSUNG_START      0xC0
#define MESSUNG_STOP       0xC1

#define KANAL_WAHL         0xC2

#define READ_START         0xCA

#define USB_STOP           0xAA

#define STROM_SET          0x88
#define MAX_STROM_SET      0x90


#define PACKET_SIZE        0x18 // 24 bytes fuer USB-Transfer
#define USB_PACKETSIZE     64
#define SD_DATA_SIZE       512
#define HEADER_SIZE        0x0A // Header zu beginn der Loggerdaten
#define BLOCK_SIZE         0x1E0 // Datenblock, 480 Bytes


// Bytes fuer Sicherungsort der Daten auf SD

#define PACKETCOUNT_BYTE         2
#define BLOCKOFFSETLO_BYTE      3
#define BLOCKOFFSETHI_BYTE      4


// Nur bei Messung_Start:
#define  STARTMINUTELO_BYTE   5
#define  STARTMINUTEHI_BYTE   6
#define BLOCK_ANZAHL_BYTE              9 // Nur bei LOGGER_START: anzahl zu lesender Blocks
#define DOWNLOADBLOCKNUMMER_BYTE      10 // aktuelle nummer des downloadblocks

// Bei Messung_Start mitgeben
#define TAKT_LO_BYTE       14
#define TAKT_HI_BYTE       15

#define KANAL_BYTE       16 // aktivierte Kanaele pro device, Liste bis anz Kanaele. Bei Start Messung uebertragen

#define HEADER_OFFSET      4



//MARK: Charger Konstanten

//Bits von loadstatus
#define BATT_MAX_BIT   0
#define BATT_MIN_BIT   1
#define BATT_DOWN_BIT   2

// buffer
#define USB_DATENBREITE 64

#define  STROM_A_L_BYTE    8
#define  STROM_A_H_BYTE    9

#define  MAX_STROM_L_BYTE    10
#define  MAX_STROM_H_BYTE    11


// sendbuffer
#define U_M_L_BYTE 16
#define U_M_H_BYTE 17
#define U_O_L_BYTE 18
#define U_O_H_BYTE 19
#define I_SHUNT_L_BYTE 20
#define I_SHUNT_H_BYTE 21
#define I_SHUNT_O_L_BYTE 22
#define I_SHUNT_O_H_BYTE 23

#define TEMP_SOURCE_L_BYTE 24
#define TEMP_SOURCE_H_BYTE 25

#define TEMP_BATT_L_BYTE   26
#define TEMP_BATT_H_BYTE   27

#define BALANCE_L_BYTE   28
#define BALANCE_H_BYTE   29

#define DEVICE_BYTE        0
// Bits fuer DEVICE_BYTE (byte auf 0 gesetzt)
#define  SPANNUNG_ID       4 // Bit fuer Batteriespannung
#define  STROM_ID          5 // Bit fuer Strom
#define  TEMP_ID           6 // Bit fuer Temperatur


//OSZI

#define OSZI_PULS_A        0
#define OSZI_PULS_B        1


#define TIMER0_STARTWERT   0x40

//#define LOOPLEDDDR          DDRD    //DDRD
//#define LOOPLEDPORT         PORTD   //PORTD
#define LOOPLED               13       //wie arduino 
#define BLINKLED               7       //nicht wie arduino 



// lookup temperatur
int templookup[48][2] = 
{ 
   {90,109},
   {92,106},
   {94,104},
   {96,102},
   {98,99},
   {100,97},
   {102,95},
   {104,92},
   {106,90},
   {108,88},
   {110,85},
   {112,83},
   {114,81},
   {116,78},
   {118,76},
   {120,73},
   {122,71},
   {124,69},
   {126,66},
   {128,64},
   {130,62},
   {132,59},
   {134,57},
   {136,55},
   {138,52},
   {140,50},
   {142,48},
   {144,45},
   {146,43},
   {148,40},
   {150,38},
   {152,36},
   {154,33},
   {156,31},
   {158,29},
   {160,26},
   {162,24},
   {164,22},
   {166,19},
   {168,17},
   {170,15},
   {172,12},
   {174,10},
   {176,8},
   {178,5},
   {180,3},
   {182,0},
   {184,-2}

};




#endif /* Header_h */
