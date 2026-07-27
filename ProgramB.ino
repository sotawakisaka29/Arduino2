#include <Wire.h>
#include <ZumoShieldN.h>
#include "ProgramTypes.h"

#define MAX_ROUTE_COMMAND 160

extern char targetLabels[];
extern int targetCount;
extern MODE mode;
extern bool labelToVec(char label, vec *p);

vec cur = {0, 0};
vec goal = {0, 0};
DIR dir = NORTH;

char com[MAX_ROUTE_COMMAND];
int commandLength = 0;
bool routeOverflow = false;

int threshold = 300;
int baseSpeed = 50;
int intersectionCenterDelayMs = 950;

float KpGap = 1.5;

const int TURN_SPEED = 100;
const float TURN_TOLERANCE_DEG = 1.0;
const float GYRO_TURN_ANGLE_DEG = 90.0;
const float GYRO_TURN_TOLERANCE_DEG = 2.0;
const unsigned long TURN_SETTLE_MS = 150;
const unsigned long TURN_TIMEOUT_MS = 8000;
const unsigned long BLOCK_TIMEOUT_MS = 15000;
const unsigned long GAP_TIMEOUT_MS = 5000;

int indexCmd = 0;
float baseAngle = 0.0;

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
// センサ・ライントレース
//==================================================

void lineTrace()
{
  reflectances.update();

  int error = reflectances.value(3) - reflectances.value(4);
  int correction = error / 7;

  motors.setSpeeds(
    baseSpeed - correction,
    baseSpeed + correction
  );
}

bool lineLost()
{
  reflectances.update();

  for (int i = 1; i <= 6; i++)
  {
    if (reflectances.value(i) > threshold)
    {
      return false;
    }
  }

  return true;
}

bool detectIntersection()
{
  reflectances.update();

  return (
    reflectances.value(2) > threshold ||
    reflectances.value(5) > threshold
  );
}

//==================================================
// 座標・方角
//==================================================

void updatePosition()
{
  switch (dir)
  {
    case NORTH: cur.y++; break;
    case EAST:  cur.x++; break;
    case SOUTH: cur.y--; break;
    case WEST:  cur.x--; break;
  }
}

bool goalCheck()
{
  return cur.x == goal.x && cur.y == goal.y;
}

float normalizeHeading(float heading)
{
  while (heading < 0.0)
  {
    heading += 360.0;
  }

  while (heading >= 360.0)
  {
    heading -= 360.0;
  }

  return heading;
}

float getHeading()
{
  return normalizeHeading(imu.averageCompassHeading());
}

void recordBaseHeading()
{
  baseAngle = getHeading();
  dir = NORTH;

  Serial.print(F("Start heading: "));
  Serial.println(baseAngle);
}

float headingForDirection(DIR target)
{
  return normalizeHeading(baseAngle + 90.0 * (int)target);
}

float shortestHeadingError(float target, float current)
{
  float error = target - current;

  if (error > 180.0)
  {
    error -= 360.0;
  }
  else if (error < -180.0)
  {
    error += 360.0;
  }

  return error;
}

bool rotateToDirection(DIR target, float gyroTargetAngle)
{
  float targetHeading = headingForDirection(target);
  unsigned long startedAt = millis();

  // 地磁気から絶対的な目標方位を決め、ジャイロで相対的に
  // 90度回転する。旋回中の終了判定にはジャイロを使用する。
  imu.turnSensorReset();

  while (true)
  {
    imu.turnSensorUpdate();

    float gyroAngle = imu.turnSensorAngleDegree();
    float gyroError = gyroTargetAngle - gyroAngle;

    if (abs(gyroError) <= GYRO_TURN_TOLERANCE_DEG)
    {
      break;
    }

    if (millis() - startedAt > TURN_TIMEOUT_MS)
    {
      motors.setSpeeds(0, 0);
      Serial.println(F("ERROR: Turn timeout."));
      return false;
    }

    // ジャイロは反時計回りが正。
    if (gyroError < 0.0)
    {
      motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
    }
    else
    {
      motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
    }
  }

  motors.setSpeeds(0, 0);

  // 停止後の惰性による回転もジャイロ角度へ反映する。
  unsigned long settleStartedAt = millis();
  while (millis() - settleStartedAt < TURN_SETTLE_MS)
  {
    imu.turnSensorUpdate();
  }

  // モーター停止後に、地磁気で絶対方位を確認する。
  float finalGyroAngle = imu.turnSensorAngleDegree();
  float finalHeading = getHeading();
  float compassError = shortestHeadingError(targetHeading, finalHeading);

  Serial.print(F("Turn gyro angle: "));
  Serial.println(finalGyroAngle);
  Serial.print(F("Turn compass error: "));
  Serial.println(compassError);

  if (abs(compassError) > TURN_TOLERANCE_DEG)
  {
    Serial.println(F("WARNING: Compass heading is outside tolerance."));
  }

  dir = target;
  return true;
}

bool turnRight()
{
  DIR target = (DIR)((dir + 1) % 4);
  return rotateToDirection(target, -GYRO_TURN_ANGLE_DEG);
}

bool turnLeft()
{
  DIR target = (DIR)((dir + 3) % 4);
  return rotateToDirection(target, GYRO_TURN_ANGLE_DEG);
}

//==================================================
// 欠線区間・1区間走行
//==================================================

bool gapDrive()
{
  unsigned long startedAt = millis();
  imu.turnSensorReset();

  while (lineLost())
  {
    if (millis() - startedAt > GAP_TIMEOUT_MS)
    {
      motors.setSpeeds(0, 0);
      Serial.println(F("ERROR: Line was not found."));
      return false;
    }

    imu.turnSensorUpdate();

    float yaw = imu.turnSensorAngleDegree();
    int correction = (int)(KpGap * (0.0 - yaw));

    motors.setSpeeds(
      baseSpeed - correction,
      baseSpeed + correction
    );
  }

  return true;
}

bool driveOneBlock(bool centerAndStop)
{
  unsigned long startedAt = millis();
  bool leftStartIntersection = false;

  while (true)
  {
    if (millis() - startedAt > BLOCK_TIMEOUT_MS)
    {
      motors.setSpeeds(0, 0);
      Serial.println(F("ERROR: Intersection timeout."));
      return false;
    }

    if (lineLost())
    {
      if (!gapDrive())
      {
        return false;
      }
    }

    bool intersection = detectIntersection();

    if (!leftStartIntersection)
    {
      if (intersection)
      {
        motors.setSpeeds(baseSpeed, baseSpeed);
      }
      else
      {
        leftStartIntersection = true;
        lineTrace();
      }

      continue;
    }

    if (intersection)
    {
      if (centerAndStop)
      {
        // 旋回前または最終地点では交差点中心付近まで進む。
        motors.setSpeeds(baseSpeed, baseSpeed);
        delay(intersectionCenterDelayMs);
        motors.setSpeeds(0, 0);
      }

      return true;
    }

    lineTrace();
  }
}

//==================================================
// 終了処理・メイン運行
//==================================================

void finishAction(bool success)
{
  motors.setSpeeds(0, 0);
  buzzer.playOn();

  if (success)
  {
    Serial.println(F("All route commands completed."));
  }
  else
  {
    Serial.println(F("Running stopped because of an error."));
  }

  for (int i = 0; i < 10; i++)
  {
    led.on();
    delay(200);
    led.off();
    delay(200);
  }
}

void runRoute()
{
  indexCmd = 0;
  bool success = true;
  bool approachingStart = true;

  while (com[indexCmd] != '\0')
  {
    // 旋回命令は現在の交差点で先に実行する。
    while (com[indexCmd] == 'r' || com[indexCmd] == 'l')
    {
      bool turned;

      if (com[indexCmd] == 'r')
      {
        turned = turnRight();
      }
      else
      {
        turned = turnLeft();
      }

      if (!turned)
      {
        success = false;
        break;
      }

      indexCmd++;
    }

    if (!success || com[indexCmd] == '\0')
    {
      break;
    }

    if (com[indexCmd] != 'f')
    {
      Serial.println(F("ERROR: Invalid generated route."));
      success = false;
      break;
    }

    // 次も直進なら停止せず交差線を通過する。
    // 旋回前と最終地点だけ交差点中心で停止する。
    bool centerAndStop = com[indexCmd + 1] != 'f';

    if (!driveOneBlock(centerAndStop))
    {
      success = false;
      break;
    }

    if (approachingStart)
    {
      // 最初のfはコース外から交差点0へ入るための命令なので、
      // グリッド内の現在座標は更新しない。
      approachingStart = false;
    }
    else
    {
      updatePosition();
    }

    indexCmd++;
  }

  if (success && !goalCheck())
  {
    Serial.println(F("ERROR: Final position does not match the goal."));
    success = false;
  }

  finishAction(success);
}

//==================================================
// 地磁気センサーのキャリブレーション
//==================================================

void calibrateCompass()
{
  Serial.println(F("Starting compass calibration."));

  imu.configureForCompassHeading();
  imu.doCompassCalibration();

  Serial.print(F("max.x: "));
  Serial.println(imu.m_max.x);
  Serial.print(F("max.y: "));
  Serial.println(imu.m_max.y);
  Serial.print(F("min.x: "));
  Serial.println(imu.m_min.x);
  Serial.print(F("min.y: "));
  Serial.println(imu.m_min.y);

  Serial.println(F("Compass calibration finished."));
}

//==================================================
// ProgramA.ino から呼び出す運行関数
//==================================================

void runOperation()
{
  mode = ROUTE_GENERATION_MODE;
  Serial.println(F("===== Route Generation Mode ====="));

  if (!makeRoute())
  {
    return;
  }

  Serial.println(F("Push button to calibrate the compass."));
  button.waitForButton();
  calibrateCompass();

  Serial.println(F("Keep the robot still for gap-driving gyro calibration."));
  imu.configureForTurnSensing();

  mode = RUN_MODE;
  Serial.println(F("===== Run Mode ====="));
  Serial.println(F("Push button to start."));
  button.waitForButton();

  // この向きをコース上の論理的な北方向として記録する。
  // 地磁気上の北がコースの上方向である必要はない。
  recordBaseHeading();

  buzzer.playOn();
  Serial.println(F("Start running."));

  runRoute();
}
