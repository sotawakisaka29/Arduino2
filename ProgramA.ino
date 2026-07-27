#include <Wire.h>
#include <ZumoShieldN.h>

#define WIDTH 3
#define HEIGHT 2
#define MAX_COMMAND 10

// struct vec {
//   int x;
//   int y;
// };

// enum DIR {
//   NORTH,
//   EAST,
//   SOUTH,
//   WEST
// };

vec cur = {0, 0};      // 現在位置
vec goal;              // 目標位置

DIR dir = NORTH;       // 初期方位

char com[MAX_COMMAND];
int commandLength = 0;

// ProgramB.ino に実装
extern void runRoute(void);
extern void calibrateCompass(void);

//==================================================
// 座標差分
//==================================================

vec difference(vec current, vec destination) {
  vec dif;

  dif.x = destination.x - current.x;
  dif.y = destination.y - current.y;

  return dif;
}

//==================================================
// コマンド追加
//==================================================

void addCommand(char c)
{
  if (commandLength < MAX_COMMAND - 1)
  {
    com[commandLength++] = c;
    com[commandLength] = '\0';
  }
}

//==================================================
// 指定方向へ向くためのコマンド生成
//==================================================

void turnTo(DIR target)
{
  int diff = (target - dir + 4) % 4;

  if (diff == 1)
  {
    addCommand('r');
  }
  else if (diff == 2)
  {
    addCommand('r');
    addCommand('r');
  }
  else if (diff == 3)
  {
    addCommand('l');
  }

  dir = target;
}

//==================================================
// 最短経路生成
// （3×4グリッド・障害物無し）
//==================================================

void createRoute()
{
  commandLength = 0;
  com[0] = '\0';

  vec dif = difference(cur, goal);

  // X方向移動

  if (dif.x > 0)
  {
    turnTo(EAST);

    for (int i = 0; i < dif.x; i++)
    {
      addCommand('f');
    }
  }
  else if (dif.x < 0)
  {
    turnTo(WEST);

    for (int i = 0; i < -dif.x; i++)
    {
      addCommand('f');
    }
  }

  // Y方向移動

  if (dif.y > 0)
  {
    turnTo(NORTH);

    for (int i = 0; i < dif.y; i++)
    {
      addCommand('f');
    }
  }
  else if (dif.y < 0)
  {
    turnTo(SOUTH);

    for (int i = 0; i < -dif.y; i++)
    {
      addCommand('f');
    }
  }
}

//==================================================
// 目的地入力
//==================================================

void getGoal()
{
  Serial.println("Input Goal X (0-3)");

  while (!Serial.available());
  goal.x = Serial.parseInt();

  Serial.println(goal.x);

  Serial.println("Input Goal Y (0-2)");

  while (!Serial.available());
  goal.y = Serial.parseInt();

  Serial.println(goal.y);
}

//==================================================
// setup
//==================================================

void setup()
{
  Serial.begin(9600);

  imu.begin();

  // 地磁気用設定
  imu.configureForCompassHeading();

  reflectances.init();

  buzzer.playOn();

  Serial.println("===== Route Setting =====");

  getGoal();

  createRoute();

  Serial.print("Generated Command : ");
  Serial.println(com);

  Serial.println("Push button to calibrate compass and start.");
  button.waitForButton();

  calibrateCompass();

  Serial.println("Start running.");

  buzzer.playOn();
}

//==================================================
// loop
//==================================================

void loop()
{
  runRoute();
}