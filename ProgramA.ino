#include <Wire.h>
#include <ZumoShieldN.h>
#include "ProgramTypes.h"

#define MAX_TARGET 20

MODE mode = COMMAND_INPUT_MODE;

char targetLabels[MAX_TARGET + 1];
int targetCount = 0;

// ProgramB.ino に実装された運行関数
extern void runOperation(void);

//==================================================
// 地点名を座標に変換
//==================================================

bool labelToVec(char label, vec *p)
{
  switch (label)
  {
    case '0': p->x = 0; p->y = 0; return true;
    case '1': p->x = 1; p->y = 0; return true;
    case '2': p->x = 2; p->y = 0; return true;
    case '3': p->x = 3; p->y = 0; return true;
    case '4': p->x = 0; p->y = 1; return true;
    case '5': p->x = 1; p->y = 1; return true;
    case '6': p->x = 2; p->y = 1; return true;
    case '7': p->x = 3; p->y = 1; return true;
    case '8': p->x = 0; p->y = 2; return true;
    case '9': p->x = 1; p->y = 2; return true;

    case 'a':
    case 'A':
      p->x = 2;
      p->y = 2;
      return true;

    case 'b':
    case 'B':
      p->x = 3;
      p->y = 2;
      return true;
  }

  return false;
}

char normalizeLabel(char label)
{
  if (label == 'A') return 'a';
  if (label == 'B') return 'b';
  return label;
}

//==================================================
// コマンド入力モード
//==================================================

void clearTargets()
{
  targetCount = 0;
  targetLabels[0] = '\0';
}

void printInputPrompt()
{
  Serial.println(F("===== Command Input Mode ====="));
  Serial.println(F("Input intersection numbers (0-9, a, b)."));
  Serial.println(F("'*' : clear all commands"));
  Serial.println(F("'.' : finish input"));
  Serial.println(F("Example: 159b."));
}

void getCommand()
{
  clearTargets();
  printInputPrompt();

  while (true)
  {
    if (Serial.available() == 0)
    {
      continue;
    }

    char input = Serial.read();

    if (input == '\n' || input == '\r')
    {
      continue;
    }

    if (input == '*')
    {
      clearTargets();
      Serial.println(F("Command cleared."));
      continue;
    }

    if (input == '.')
    {
      if (targetCount == 0)
      {
        Serial.println(F("ERROR: Enter at least one destination."));
        continue;
      }

      break;
    }

    vec unused;
    if (!labelToVec(input, &unused))
    {
      Serial.println(F("ERROR: Unsupported command."));
      continue;
    }

    if (targetCount >= MAX_TARGET)
    {
      clearTargets();
      Serial.println(F("ERROR: Too many commands. All commands were cleared."));
      continue;
    }

    input = normalizeLabel(input);
    targetLabels[targetCount++] = input;
    targetLabels[targetCount] = '\0';

    Serial.print(F("Command: "));
    Serial.println(targetLabels);
  }

  Serial.print(F("Input command: "));
  Serial.println(targetLabels);
}

//==================================================
// Arduino setup / loop
//==================================================

void setup()
{
  Serial.begin(9600);

  imu.begin();
  imu.configureForCompassHeading();
  reflectances.init();

  buzzer.playOn();

  mode = COMMAND_INPUT_MODE;
  getCommand();

  mode = ROUTE_GENERATION_MODE;
}

void loop()
{
  if (mode == ROUTE_GENERATION_MODE)
  {
    // 入力完了後の経路生成・校正・走行はProgramBへ任せる。
    runOperation();
    mode = FINISHED_MODE;
  }
}
