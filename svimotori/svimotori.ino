 #include <Servo.h>

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 700 
#define MOTOR_PINur 9
#define MOTOR_PINul 10 // dod
#define MOTOR_PINdr 11 // dod 
#define MOTOR_PINdl 12 // dod
Servo motor_ur;
Servo motor_ul;
Servo motor_dr;
Servo motor_dl;
int i=0;
int incomingByte=0;
void setup() {
  Serial.begin(9600);
  Serial.println("Program beg`n...");

  motor_ur.attach(MOTOR_PINur);  
  motor_ul.attach(MOTOR_PINul);
  motor_dr.attach(MOTOR_PINdr);
  motor_dl.attach(MOTOR_PINdl);
 
  motor_ur.writeMicroseconds(MIN_SIGNAL);
  motor_ul.writeMicroseconds(MIN_SIGNAL);
  motor_dr.writeMicroseconds(MIN_SIGNAL);
  motor_dl.writeMicroseconds(MIN_SIGNAL);
}
void loop() {  
// send data only when you receive data:
        if ((Serial.available() > 0)){
                // read the incoming byte:
                if((i%4)==0){
                  incomingByte = Serial.parseInt();
                  Serial.println(incomingByte);
                
             motor_ur.writeMicroseconds(incomingByte);
             
                }
                  if((i%4)==1){
                  incomingByte = Serial.parseInt();
                  Serial.println(incomingByte);
                  
             motor_ul.writeMicroseconds(incomingByte);
             
                }
                  if((i%4)==2){
                  incomingByte = Serial.parseInt();
                  Serial.println(incomingByte);
                  
             motor_dr.writeMicroseconds(incomingByte);
            
                }    
                  if((i%4)==3){
                  incomingByte = Serial.parseInt();
                  Serial.println(incomingByte);
                  
             motor_dl.writeMicroseconds(incomingByte);
             
                }            
            
            //    incomingByte = Serial.parseInt();
                
               //if((1200<incomingByte)&(incomingByte<2100))
                // say what you got:
             //   Serial.println(incomingByte);
            // motor_ur.writeMicroseconds(incomingByte);
 i++;           
}
}
