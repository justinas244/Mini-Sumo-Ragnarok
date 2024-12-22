#include <IRremote.h>
#include <EEPROM.h>
int RECV_PIN = 12; // įvesties pinas, kuriuo priimami IR signalai

IRrecv irrecv(RECV_PIN);
decode_results results;

//////////////////// JSUMO pultelio kodai //////////////////////////
int kanalArray[11][4] = {
    { 2559, 2438, 2439, 2440 }, // Kanalo parinkimas, Ready, Start, Stop kodai
    { 2687, 2566, 2567, 2568 },
    { 2815, 2694, 2695, 2696 },
    { 2943, 2822, 2823, 2824 },
    { 3071, 2950, 2951, 2952 },
    { 3199, 3078, 3079, 3080 },
    { 3327, 3206, 3207, 3208 },
    { 3455, 3334, 3335, 3336 },
    { 3583, 3462, 3463, 3464 },
    { 3711, 3590, 3591, 3592 },
    { 3839, 3718, 3719, 3720 }
};


int pasirinktas_kanalas = 0;
bool paleista = false;
bool sustabdyta = false;

////////////////////////////////////////////////////////////////////

const byte program_led = 13;

//LINE begin
const byte LINE_LEFT = 3;
const byte LINE_RIGHT = 2;
//LINE end

//FRONT SENSOR begin
const byte BACK = A0;
const byte FRONT_90_RIGHT = A1;
const byte FRONT_45_RIGHT  = A2;
const byte FRONT = A3;
const byte FRONT_90_LEFT = A4;
const byte FRONT_45_LEFT = A5;
//FRONT SENSOR end

//START MODULE begin
const byte START_MODULE = 7;
//START MODULE end

//SENSORS SUMS
byte line_sum = 0;
byte sharp_sum = 0;

//MOTOR CONTROL VARIABLES
double INTEGRAL = 0;
int V = 0;

//MOTORS begin
int motor_R = 10;    //pwm
int motor_L = 9;   //pwm

int motor_r = 11;  //dir
int motor_l = 8;  // dir
//MOTORS end

//VALDYMO KINTAMIEJI begin
const byte SPEED_STOP = 0;
 byte SPEED_NORMAL = 140;//80
const byte SPEED_ATTACK2 = 180;//180
const byte SPEED_ATTACK = 255;//255

const byte SPEED_GO_BACK_LINE = 255;//255
const byte SPEED_GO_BACK_LINE_TURN = 255;//200
const byte SPEED_GO_BACK_LINE_TURN2 = 255;//200

/// LAIKAI ///////////////////////
const unsigned short TIME_FOR_GO_BACK = 80;
const unsigned short TIME_FOR_TURNING = 100;
const unsigned short TIME_FOR_TURNING_BOTH = 110;
unsigned short TURNING_TIME = TIME_FOR_TURNING;
unsigned long LINE_TIME = 2;

//STATES
byte RUN_STATE = 0;
byte LINE_AVOID_STATE = 0;
byte LAST_SEEN = 0;
byte LAST_STATE = 0;
byte STRATEGY_STATE = 0;

//STATE TIMERS
unsigned long LAST_SEEN_TIME = 0;
unsigned long STRATEGY_START_TIME = 0;
unsigned long TIME_FOR_STRATEGY = 0;



unsigned long TIME_FOR_ATACK_SET = 0;
unsigned long TIME = 0;


int check_time = 300;

int pasirinkta_programa = 0;

int selectedProgram = -1;

unsigned long valdymo[][3] = {  /// pultelio programu kodai hex formatu
  {0x530DB67C, 0x17B4A228, 0xA90},     // Programu pultelio Start komanda
  {0xCD76C8CB, 0x7BA067FF, 0x5D0}      // Programu pultelio Start komanda
};

unsigned long programos_check[3] = {0x54E680D6, 0x903F952A, 0xA70}; 

unsigned long sensoriu_isjungimas[3] = {0xA74647DD, 0xFDAC0152, 0xDD0}; 
unsigned long sensoriu_ijungimas[3] = {0xD41A2FD5, 0x8243CF09, 0xFD0}; 

byte naudoti_pulta = 0;
bool naudoti_sensorius = true;


void setup() {
  
  Serial.begin(9600);
  //myservo.writeMicroseconds(1500);  // set servo to mid-point
  pinMode(LINE_LEFT, INPUT);
  pinMode(LINE_RIGHT, INPUT);
  
   pinMode(BACK, INPUT_PULLUP);
  pinMode( FRONT_90_RIGHT , INPUT_PULLUP);
  pinMode( FRONT_45_RIGHT  , INPUT_PULLUP);
  pinMode(FRONT , INPUT_PULLUP);

  pinMode(FRONT_90_LEFT , INPUT_PULLUP);
  pinMode(FRONT_45_LEFT , INPUT_PULLUP);
  
  pinMode(START_MODULE, INPUT);
  pinMode(program_led,OUTPUT);
  
 irrecv.enableIRIn(); // įjungti priėmimą
 // pinMode(led,OUTPUT);
  
    pinMode(motor_R,OUTPUT);
    pinMode(motor_L,OUTPUT);
    pinMode(motor_r,OUTPUT);
    pinMode(motor_l,OUTPUT);

  motor(0,0);
  delay(1000);


 /* TCCR1A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM20);
  TCCR1B = TCCR1B & 0b11111000 | 0x02;*/  //3.9KHz daznis motorams

 TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM10);  // Fast PWM, 8-bit
 TCCR1B = (TCCR1B & 0b11111000) | 0x04;  // Prescaler 256            //244Hz daznis motorams
  
  pasirinktas_kanalas = EEPROM.read(0);
  

//////// LAUKIAM STARTO KOMANDOS ////////////////
  while(true){
     jsumo_pultelis();
     pultelio_programos();
     if(digitalRead(START_MODULE)){ naudoti_pulta = 0; break; }

     if (irrecv.decode(&results)) {
        unsigned long kodas = results.value;
        int jsumo_kodas = IrReceiver.decodedIRData.decodedRawData;
        
        Serial.println(kodas, HEX);
        if(kodas == valdymo[0][0] || kodas == valdymo[0][1] || kodas == valdymo[0][2] || jsumo_kodas == kanalArray[pasirinktas_kanalas][2]){
          pinMode(START_MODULE, OUTPUT);
          //delay(5000);
          delay(1);
          digitalWrite(START_MODULE, HIGH);
          naudoti_pulta = 1;
          break;
        }
        
        delay(1);
      }
  }
  
  
  STRATEGY_START_TIME = millis();
}

void loop() {
  if (millis() - LINE_TIME > TURNING_TIME) LINE_AVOID_STATE = 0;
  if (millis() - STRATEGY_START_TIME > TIME_FOR_STRATEGY) STRATEGY_STATE = 0;
    
  readSensors();
  
   switch (naudoti_pulta){
     case 1:
     stop_skaitymas();
     break;
   }
  
  switch (RUN_STATE) {
    case 0: switch(LINE_AVOID_STATE) {
              case 0: switch(STRATEGY_STATE) {
                        case 0: motor(SPEED_NORMAL, SPEED_NORMAL); break;

                        case 1: TIME_FOR_STRATEGY = 100;
                                motor(-SPEED_ATTACK,SPEED_ATTACK);
                                break;
                  
                        case 2: TIME_FOR_STRATEGY = 800;
                                motor(SPEED_ATTACK,SPEED_ATTACK);
                                break;
                                
                        case 3: TIME_FOR_STRATEGY = 100;
                                motor(SPEED_ATTACK,-SPEED_ATTACK);
                                break;
                                
                        case 4: //TIME_FOR_STRATEGY = 35;
                                 motor(-SPEED_ATTACK, SPEED_ATTACK);
                                 delay(65);//70
                                 STRATEGY_STATE = 41;
                                //break;
                                
                        case 41: TIME_FOR_STRATEGY = 900;
                                 motor(SPEED_ATTACK, 115);
                                 break;
                       
                        case 5: Mega_sumo_taktika();
                                break;

                        case 6: //TIME_FOR_STRATEGY = 35;
                                motor(SPEED_ATTACK, -SPEED_ATTACK);
                                delay(65);//70
                                STRATEGY_STATE = 61;
                                //break;
                                
                       case 61: TIME_FOR_STRATEGY = 900;
                                 motor(110,SPEED_ATTACK);
                                 break;

                                 
                       case 7: motor(-SPEED_ATTACK, SPEED_ATTACK);
                               delay(40);
                               STRATEGY_STATE = 71;
                               
                       case 71: motor(SPEED_ATTACK, SPEED_ATTACK);
                                delay(180);
                                motor(SPEED_ATTACK, -SPEED_ATTACK);
                                delay(120);
                                STRATEGY_STATE = 72;
                           
                       case 72: motor(SPEED_ATTACK, SPEED_ATTACK);
                                delay(200);
                                break;

                      case 8:  paieska(); 

                      case 9: motor(SPEED_ATTACK, -SPEED_ATTACK);
                               delay(40);
                               STRATEGY_STATE = 91;
                               
                      case 91: motor(SPEED_ATTACK, SPEED_ATTACK);
                                delay(180);
                                motor(-SPEED_ATTACK, SPEED_ATTACK);
                                delay(120);
                                STRATEGY_STATE = 92;
                           
                       case 92: motor(SPEED_ATTACK, SPEED_ATTACK);
                                delay(200);
                                break;
                        
                        
                                        
                      } break;
              case 1: motor(SPEED_GO_BACK_LINE_TURN, -SPEED_GO_BACK_LINE_TURN2);  break;
              case 2: motor(-SPEED_GO_BACK_LINE_TURN2, SPEED_GO_BACK_LINE_TURN);  break;
              case 3: switch(LAST_SEEN) {
                        case 0: motor(SPEED_GO_BACK_LINE_TURN, -SPEED_GO_BACK_LINE_TURN2);  break;
                        case 1: motor(-SPEED_GO_BACK_LINE_TURN2, SPEED_GO_BACK_LINE_TURN);  break;
                        case 2: motor(SPEED_GO_BACK_LINE_TURN, -SPEED_GO_BACK_LINE_TURN2);  break;
                      } break;   
            } 
            INTEGRAL = 0;
            break;
                
    case 1: motor(-SPEED_GO_BACK_LINE, -SPEED_GO_BACK_LINE);
            delay(TIME_FOR_GO_BACK);
            LINE_AVOID_STATE = 1;
            TIME_FOR_STRATEGY = 0;
            TURNING_TIME = TIME_FOR_TURNING;
            LINE_TIME = millis();
            break;
            
    case 2: motor(-SPEED_GO_BACK_LINE, -SPEED_GO_BACK_LINE);
            delay(TIME_FOR_GO_BACK);
            LINE_AVOID_STATE = 2;
            TIME_FOR_STRATEGY = 0;
            TURNING_TIME = TIME_FOR_TURNING;
            LINE_TIME = millis();
            break;
            
    case 3: motor(-SPEED_GO_BACK_LINE, -SPEED_GO_BACK_LINE);
            delay(TIME_FOR_GO_BACK);
            LINE_AVOID_STATE = 3;
            TURNING_TIME = TIME_FOR_TURNING_BOTH;
            TIME_FOR_STRATEGY = 0;
            LINE_TIME = millis();
            break;

    case 4: motor(-SPEED_ATTACK,SPEED_ATTACK );
          // delay(15);
           break;
    case 5: motor(-SPEED_ATTACK,SPEED_ATTACK2 );
            //delay(5);
            break;
    case 6: motor(SPEED_ATTACK,SPEED_ATTACK);
            //delay(2000);
            break;
    case 7: motor(SPEED_ATTACK2,-SPEED_ATTACK );
           //delay(5);
           break;
    case 8: motor(SPEED_ATTACK,-SPEED_ATTACK );
            //delay(15);
            break;
            
   case 9: motor(SPEED_ATTACK,SPEED_ATTACK);
           delay(50);
            break;
  }
}
void readSensors() {
  line_sum =  PIND&0b1100;
  sharp_sum = PINC & 0b111110;
  //sharp_sum = 0;
 //Serial.println(sharp_sum, BIN);
   
   switch(line_sum){
     case 0b1000: RUN_STATE = 2; break;
     case 0b0100: RUN_STATE = 1; break;
     case 0b1100: RUN_STATE = 3; break;
     case 0b0000: 
        switch(sharp_sum){
          case 0b111110: RUN_STATE = 0; break;   // niekas nemato
          case 0b111100: RUN_STATE = 4; break;   // FL90 Mato
          case 0b111010: RUN_STATE = 5; break;   // FL45 Mato
          case 0b110110: RUN_STATE = 6; break;   // Front Mato
          case 0b101110: RUN_STATE = 8; break;   // FR90 Mato
          case 0b011110: RUN_STATE = 7; break;   // FR45 Mato

          case 0b110010: RUN_STATE = 9; break;   // Front  ir FL45 mato
          case 0b010110: RUN_STATE = 9; break;   // Front  ir FR45 mato
          case 0b010010: RUN_STATE = 9; break;   // Visas priekis mato
          
          case 0b000010: RUN_STATE = 8; break;   // Visas priekis mato + FR90
          case 0b010000: RUN_STATE = 9; break;   // Visas priekis mato + FL90


         //case 0b111110: RUN_STATE = 0; break;   // niekas nemato
          
        } 
   }
  
 delay(1);
}

/*void motor(int left, int right) {
  // right = right*(-1);
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  if (right >= 0) {
    digitalWrite(motor_r, LOW);
    analogWrite(motor_R, right);
  }
  else {
    digitalWrite(motor_r, HIGH);
    analogWrite(motor_R, 255+right);
  }

  if (left >= 0) {
    digitalWrite(motor_l, LOW);
    analogWrite(motor_L, left);
  }
  else {
    digitalWrite(motor_l, HIGH);
    analogWrite(motor_L, 255+left);
  }
}*/

void motor(int left, int right) {
  left=constrain(left, -255, 255);
  right=constrain(right, -255, 255);

  if(right >= 0) {
    //digitalWrite(IN4, LOW);
   digitalWrite(motor_r, LOW);
    OCR1B = right;
  } else {
    //digitalWrite(IN4, HIGH);
    digitalWrite(motor_r, HIGH);
    OCR1B = 255+right;
  }
  if(left >= 0) {
    //digitalWrite(IN2, LOW);
    digitalWrite(motor_l, LOW);
    OCR1A = left;
  } else {
    //digitalWrite(IN2, HIGH);
   digitalWrite(motor_l, HIGH);
    OCR1A = 255+left;
  }
}





