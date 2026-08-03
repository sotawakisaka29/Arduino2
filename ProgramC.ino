#include <Wire.h>
#include <ZumoShieldN.h>
#include "ProgramTypes.h"

#define MAX_ROUTE_COMMAND 160

extern char targetLabels[];
extern int targetCount;
extern vec cur;
extern vec goal;
extern DIR dir;
extern MODE mode;
extern bool labelToVec(char label, vec *p);
extern void runOperation(void);

char com[MAX_ROUTE_COMMAND];
int commandLength = 0;
bool routeOverflow = false;

//==================================================
// 経路生成
//==================================================

bool addRouteCommand(char command)
{
  if (commandLength >= MAX_ROUTE_COMMAND - 1)
  {
    routeOverflow = true;
    return false;
  }

  com[commandLength++] = command;
  com[commandLength] = '\0';
  return true;
}

int directionX(DIR direction)
{
  if (direction == EAST) return 1;
  if (direction == WEST) return -1;
  return 0;
}

int directionY(DIR direction)
{
  if (direction == NORTH) return 1;
  if (direction == SOUTH) return -1;
  return 0;
}

void addTurnCommands(DIR target, DIR *planningDir)
{
  int diff = (target - *planningDir + 4) % 4;

  if (diff == 1)
  {
    addRouteCommand('r');
  }
  else if (diff == 2)
  {
    addRouteCommand('r');
    addRouteCommand('r');
  }
  else if (diff == 3)
  {
    addRouteCommand('l');
  }

  *planningDir = target;
}

void addMoves(DIR target, int count, DIR *planningDir)
{
  if (count <= 0 || routeOverflow)
  {
    return;
  }

  addTurnCommands(target, planningDir);

  for (int i = 0; i < count && !routeOverflow; i++)
  {
    addRouteCommand('f');
  }
}

void addXMoves(int dx, DIR *planningDir)
{
  if (dx > 0)
  {
    addMoves(EAST, dx, planningDir);
  }
  else if (dx < 0)
  {
    addMoves(WEST, -dx, planningDir);
  }
}

void addYMoves(int dy, DIR *planningDir)
{
  if (dy > 0)
  {
    addMoves(NORTH, dy, planningDir);
  }
  else if (dy < 0)
  {
    addMoves(SOUTH, -dy, planningDir);
  }
}

// 現在向いている方向に近い軸を先に選び、マンハッタン距離が
// 最短のまま旋回回数も少なくなるようにする。
void createRouteOneSection(vec start, vec destination, DIR *planningDir)
{
  int dx = destination.x - start.x;
  int dy = destination.y - start.y;

  if (dx == 0)
  {
    addYMoves(dy, planningDir);
    return;
  }

  if (dy == 0)
  {
    addXMoves(dx, planningDir);
    return;
  }

  int xPriority = directionX(*planningDir) * dx;
  int yPriority = directionY(*planningDir) * dy;

  if (xPriority > yPriority)
  {
    addXMoves(dx, planningDir);
    addYMoves(dy, planningDir);
  }
  else
  {
    addYMoves(dy, planningDir);
    addXMoves(dx, planningDir);
  }
}

bool makeRoute()
{
  commandLength = 0;
  com[0] = '\0';
  routeOverflow = false;

  // ロボットは交差点0の下からスタートするため、
  // 最初に交差点0まで進む接近用の直進を追加する。
  addRouteCommand('f');

  vec routePosition = cur;
  DIR planningDir = dir;

  for (int i = 0; i < targetCount && !routeOverflow; i++)
  {
    vec destination;
    labelToVec(targetLabels[i], &destination);

    createRouteOneSection(routePosition, destination, &planningDir);
    routePosition = destination;
  }

  if (routeOverflow)
  {
    commandLength = 0;
    com[0] = '\0';
    Serial.println(F("ERROR: Generated route is too long."));
    return false;
  }

  goal = routePosition;

  Serial.print(F("Generated route: "));
  Serial.println(com);
  Serial.print(F("Final goal: "));
  Serial.print(goal.x);
  Serial.print(F(","));
  Serial.println(goal.y);

  return true;
}

//==================================================
// ProgramA.ino から呼び出す経路生成関数
//==================================================

void runRouteGeneration()
{
  mode = ROUTE_GENERATION_MODE;
  Serial.println(F("===== Route Generation Mode ====="));

  if (!makeRoute())
  {
    return;
  }

  runOperation();
}
