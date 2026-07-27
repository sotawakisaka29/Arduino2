#include <Wire.h>
#include <ZumoShieldN.h>

struct vec {
  int x;
  int y;
};

enum DIR {
  NORTH,
  EAST,
  SOUTH,
  WEST
};

extern char com[];
extern vec cur;
extern vec goal;
extern DIR dir;

int threshold = 300;

int indexCmd = 0;

int baseSpeed = 50;

float KpTrace = 0.4;
float KpTurn = 2.0;
float KpGap = 1.5;


// ライントレース


void lineTrace(){
    reflectances.update();

    int error =
        reflectances.value(3)
        - reflectances.value(4);

    int correction =
        (int)(KpTrace * error);

    motors.setSpeeds(
        baseSpeed - correction,
        baseSpeed + correction
    );
}


// ライン消失判定


bool lineLost(){
    reflectances.update();
    for(int i=1;i<=6;i++){
        if(reflectances.value(i) > threshold){
            return false;
        }
    }
    return true;
}


// 交差点判定


bool detectIntersection(){
    reflectances.update();
    if(
        reflectances.value(2) > threshold ||
        reflectances.value(5) > threshold
    ){
        return true;
    }
    return false;
}


// 現在座標更新


void updatePosition(){
    switch(dir){
        case NORTH:
            cur.y++;
            break;

        case SOUTH:
            cur.y--;
            break;

        case EAST:
            cur.x++;
            break;

        case WEST:
            cur.x--;
            break;
    }
}


// ゴール判定


bool goalCheck(){
    return (
        cur.x == goal.x &&
        cur.y == goal.y
    );
}


// 地磁気方位取得


float getHeading()
{
    float h = imu.averageCompassHeading();

    if (h < 0.0)
    {
        h += 360.0;
    }

    if (h >= 360.0)
    {
        h -= 360.0;
    }

    return h;
}


// 右旋回


void turnRight(){
    float startHeading =
        getHeading();

    float targetHeading =
        startHeading + 90.0;

    if(targetHeading >= 360.0){
        targetHeading -= 360.0;
    }

    while(1){
        float heading = getHeading();
        float error = targetHeading - heading;

        if(error > 180){
            error -= 360;
        }

        if(error < -180){
            error += 360;
        }

        if(abs(error) < 3){
            break;
        }

        int turnSpeed =
            KpTurn * error;

        motors.setSpeeds(
            turnSpeed,
            -turnSpeed
        );
    }

    motors.setSpeeds(0,0);
    switch(dir)
    {
        case NORTH: dir = EAST; break;
        case EAST:  dir = SOUTH; break;
        case SOUTH: dir = WEST; break;
        case WEST:  dir = NORTH; break;
    }
}


// 左旋回


void turnLeft(){
    float startHeading =
        getHeading();

    float targetHeading =
        startHeading - 90.0;

    if(targetHeading < 0.0){
        targetHeading += 360.0;
    }

    while(1){
        float heading =
            getHeading();

        float error =
            targetHeading - heading;

        if(error > 180){
            error -= 360;
        }

        if(error < -180){
            error += 360;
        }

        if(abs(error) < 3){
            break;
        }

        int turnSpeed =
            KpTurn * error;

        motors.setSpeeds(
            turnSpeed,
            -turnSpeed
        );
    }
    motors.setSpeeds(0,0);
    switch(dir){
        case NORTH: dir = WEST; break;
        case WEST:  dir = SOUTH; break;
        case SOUTH: dir = EAST; break;
        case EAST:  dir = NORTH; break;
    }
}


// 欠線区間走行


void gapDrive(){
    imu.turnSensorReset();
    while(lineLost()){
        imu.turnSensorUpdate();
        float yaw = imu.turnSensorAngleDegree();
        float error = 0.0 - yaw;
        int correction = KpGap * error;
        motors.setSpeeds(
            baseSpeed - correction,
            baseSpeed + correction
        );

        reflectances.update();

        if(
            reflectances.value(3) > threshold ||
            reflectances.value(4) > threshold
        ){
            break;
        }
    }
}


// 到着処理


void goalAction(){
    motors.setSpeeds(0,0);
    buzzer.playOn();
    for(int i=0;i<10;i++){
        led.on();
        delay(200);

        led.off();
        delay(200);
    }
}


// メイン運行


void runRoute(){
    indexCmd = 0;
    while(1){
        if(goalCheck()){
            goalAction();
            break;
        }
        reflectances.update();
        if(lineLost()){
            gapDrive();
        }

        if(!detectIntersection()){
            lineTrace();
            continue;
        }

        delay(300);

        updatePosition();

        if(goalCheck()){
            goalAction();
            break;
        }

        switch(com[indexCmd]){
            case 'f':
                indexCmd++;
                break;

            case 'r':
                turnRight();
                indexCmd++;
                break;

            case 'l':
                turnLeft();
                indexCmd++;
                break;

            case '\0':
                goalAction();
                return;
        }
    }

    while(1);
}

//キャリブレーション

void calibrateCompass()
{
    Serial.println("starting calibration");

    imu.configureForCompassHeading();

    imu.doCompassCalibration();

    Serial.print("max.x   ");
    Serial.println(imu.m_max.x);

    Serial.print("max.y   ");
    Serial.println(imu.m_max.y);

    Serial.print("min.x   ");
    Serial.println(imu.m_min.x);

    Serial.print("min.y   ");
    Serial.println(imu.m_min.y);

    Serial.println("calibration finished");
}