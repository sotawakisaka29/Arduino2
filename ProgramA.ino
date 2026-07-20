#include <Wire.h>
#include <ZumoShieldN.h>

#define MAX_COMMAND 10

int speed = 50;
int threshold = 300;
int index = 0;
int i;

// const int width = 3;
// const int height = 3;

// struct Vec {
//   int x;
//   int y;
// };

// const int R[2][2] = {
//   {0,1},
//   {-1,0}
// };

// const int L[2][2] = {
//   {0, -1},
//   {1, 0}
// };

// const int F[2][2] = {
//   {1, 0},
//   {0, 1}
// };


char com[MAX_COMMAND];
int cmdIndex = 0;

void setup() {
  Serial.begin(9600);

  buzzer.playOn(); 
  Serial.println("Input Available");
  getCommand();

  Serial.println("Push button to start.");
  button.waitForButton();

  buzzer.playOn();
}

// void turnFunc(const int M[2][2], Vec *v){
//   int nx = M[0][0] * v->x + M[0][1] * v->y;
//   int ny = M[1][0] * v->x + M[1][1] * v->y;

//   v->x = nx;  //(v*).xと同じ
//   v->y = ny;  //(v*).yと同じ
// }

// bool judge(){
//   Vec pos = {0, 0};  // P0
//   Vec dir = {0, 1};  // r0

//   for(int i = 0; i < cmdIndex; i++){
//     switch(com[i])
//     {
//       case 'r':
//         turnFunc(R, &dir);  //&dirはdirが入ってるポインタ
//         break;

//       case 'l':
//         turnFunc(L, &dir);
//         break;

//       case 'f':
//         turnFunc(F, &dir);
//         break;
//     }

//     pos.x += dir.x;
//     pos.y += dir.y;

//     // ここにコースアウト条件を書く
//     if(pos.x < 0 || pos.x > width ||
//        pos.y < 0 || pos.y > height)
//     {
//       return false;
//     }
//   }

//   return true;
// }

void loop() {
  function();
}


void getCommand(void){
  cmdIndex = 0;

  while (1) {
    if (Serial.available() > 0) {
      char input = Serial.read();

      if (input == '\n' || input == '\r') {
        continue;
      }

      if (cmdIndex >= MAX_COMMAND) {
        cmdIndex = 0;
        Serial.println("Over Max Command. Delete All Command!");
      }

      if (input == 'd') {
        cmdIndex = 0;
        Serial.println("Delete All Command!");
      }
      else if (input == 'r' || input == 'l' || input == 'f') {

        // 一旦コマンドを追加
        com[cmdIndex++] = input;
        com[cmdIndex] = '\0';

        // 追加後の状態で判定
        if (!judge()) {
          Serial.println("ERROR : Course Out!");

          // 追加したコマンドを取り消す
          cmdIndex--;
          com[cmdIndex] = '\0';

          Serial.println("Re-Input Command!");
        }else {
          Serial.print("Current Command : ");
          Serial.println(com);
        }

      }
      else if (input == '.') {
        com[cmdIndex] = '\0';

        Serial.print("Command : ");
        Serial.println(com);
        break;
      }
      else {
        Serial.println("Wrong Command!");
      }
    }
  }
}


