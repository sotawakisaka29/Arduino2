#include <Wire.h>
#include <ZumoShieldN.h>

// int speed = 50;
bool turn = false;
// int threshold = 300;
// char com[20] = "rlfrrl";
// int index;
// int i;


// void setup() {
//   buzzer.playOn();
//   Serial.begin(9600);
//   Serial.println("Zumo sample Start!");
// }

// void loop() {
//   function();
// }


void function(void){
  index = 0;
  // button.waitForButton();
  while(1){
    // reflector();
    reflectances.update();
    if (reflectances.value(2) > threshold || reflectances.value(5) > threshold) {
      if(com[index]=='f'){
        while(1){
          reflectances.update();
          if(reflectances.value(2) < threshold && reflectances.value(5) < threshold){
            index += 1;
            break;
          }
        }
      }else{
        delay(950);
        motors.setSpeeds(0,0);
        turn = true;
      }
    }
    if (!turn){
      int error = reflectances.value(3) - reflectances.value(4);
      int correction = error / 7;
      motors.setSpeeds(
        speed - correction,
        speed + correction
      );

      // motors.setSpeeds(speed, speed);
    }else{
      // motors.setSpeeds(0,0);
      if (com[index] == '\0'){
        turn = false;
        break;
      }
      else if (com[index] == 'r'){
        motors.setSpeeds(100,-100);
        
        while(1){
          reflectances.update();
          if (reflectances.value(5) > threshold || reflectances.value(6) > threshold){
          break;
          }
        }
        while(1){
          reflectances.update();
          if(reflectances.value(3) > threshold ){
            break;
          }
        }
      }
      else if (com[index] == 'l'){
        motors.setSpeeds(-100,100);
        while(1){
          reflectances.update();
          if (reflectances.value(1) > threshold || reflectances.value(2) > threshold){
          break;
          }
        }
        while(1){
          reflectances.update();
          if(reflectances.value(4) > threshold){
            break;
          }
        }
      }
      // delay(1600); 


      motors.setSpeeds(0,0);

      turn = false;
      index += 1;
    }
  }
  buzzer.playOn();
  for(i=0;i<10;i++){
    led.on();
    delay(200);
    led.off();
    delay(200);
  }

}

//*********

// void correction(void){
//   while(1){
//     reflectances.update();
//     if(reflectances.value(1) > threshold)
//   }
// }

//*********

// void reflector(void){
//   Serial.print(reflectances.value(1));
//   Serial.print(',');
//   Serial.print(reflectances.value(2));
//   Serial.print(',');  
//   Serial.print(reflectances.value(3));
//   Serial.print(',');
//   Serial.print(reflectances.value(4));
//   Serial.print(',');  
//   Serial.print(reflectances.value(5));
//   Serial.print(',');  
//   Serial.print(reflectances.value(6));
//   Serial.print(',');    
//   Serial.println();
// }