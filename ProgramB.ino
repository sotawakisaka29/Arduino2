#include <Wire.h>
#include <ZumoShieldN.h>
#include "ProgramTypes.h"

extern MODE mode;
extern char com[];

vec cur = {0, 0};
vec goal = {0, 0};
DIR dir = NORTH;

int threshold = 300;
int baseSpeed = 60;
int intersectionCenterDelayMs = 970;

float KpGap = 2.0;

const int TURN_SPEED = 120;
const float TURN_TOLERANCE_DEG = 1.0;
const float GYRO_TURN_ANGLE_DEG = 90.0;
const float GYRO_TURN_TOLERANCE_DEG = 3.0;
const unsigned long TURN_SETTLE_MS = 150;
const unsigned long TURN_TIMEOUT_MS = 8000;
const unsigned long BLOCK_TIMEOUT_MS = 15000;
const unsigned long GAP_TIMEOUT_MS = 5000;

int indexCmd = 0;
float baseAngle = 0.0;

//==================================================
// センサ・ライントレース
//==================================================

void lineTrace()
{
  reflectances.update();

  int error = reflectances.value(3) - reflectances.value(4);
  int correction = error / 5;

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
      // 出発側の交差点を抜けるまでは次の交差点として数えない。
      // ただし旋回終了直後からライン補正は停止させない。
      if (!intersection)
      {
        leftStartIntersection = true;
      }

      lineTrace();
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
