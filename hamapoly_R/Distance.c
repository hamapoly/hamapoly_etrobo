// 参考：https://qiita.com/TetsuroAkagawa
// 引用：https://qiita.com/TetsuroAkagawa/items/ba6190f08d26df7cc8ad

#include "Distance.h"

#define TIRE_DIAMETER 90.0  //タイヤ直径(約90mm *ETロボコンシミュレータの取扱説明書参照) -> (90.0mm *2020年ADVクラスのDENSOチームのモデル図に記載)

static float distance = 0.0;     //走行距離
static float distance4msL = 0.0; //左タイヤの4ms間の距離
static float distance4msR = 0.0; //右タイヤの4ms間の距離
static float pre_angleL, pre_angleR; // 左右モータ回転角度の過去値

static float angle4msL = 0.0; //左タイヤの4ms間の角度
static float angle4msR = 0.0; //右タイヤの4ms間の角度

/* 初期化関数 */
void Distance_init() {
    //各変数の値の初期化
    distance = 0.0;
    distance4msR = 0.0;
    distance4msL = 0.0;
    //モータ角度の過去値に現在値を代入
    pre_angleL = ev3_motor_get_counts(EV3_PORT_C);
    pre_angleR = ev3_motor_get_counts(EV3_PORT_B);
}

/* 距離更新（4ms間の移動距離を毎回加算している） */
void Distance_update(){
    float cur_angleL = ev3_motor_get_counts(EV3_PORT_C); //左モータ回転角度の現在値
    float cur_angleR = ev3_motor_get_counts(EV3_PORT_B);//右モータ回転角度の現在値
    float distance4ms = 0.0;        //4msの距離

    // 4ms間の走行距離 = ((円周率 * タイヤの直径) / 360) * (モータ角度過去値　- モータ角度現在値)
    distance4msL = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleL - pre_angleL);  // 4ms間の左モータ距離
    distance4msR = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleR - pre_angleR);  // 4ms間の右モータ距離
    distance4ms = (distance4msL + distance4msR) / 2.0; //左右タイヤの走行距離を足して割る
    distance += distance4ms;

    angle4msL = cur_angleL - pre_angleL;
    angle4msR = cur_angleR - pre_angleR;

    //モータの回転角度の過去値を更新
    pre_angleL = cur_angleL;
    pre_angleR = cur_angleR;
}

/* 走行距離を取得 */
float Distance_getDistance(){
    return distance;
}

/* 右タイヤの4ms間の距離を取得 */
float Distance_getDistance4msRight(){
    return distance4msR;
}

/* 左タイヤの4ms間の距離を取得 */
float Distance_getDistance4msLeft(){
    return distance4msL;
}

/* 右タイヤの4ms間の角度を取得 */
int Distance_getAngle4msLeft(){
    return roundf(angle4msL);
}

/* 左タイヤの4ms間の角度を取得 */
int Distance_getAngle4msRight(){ 
    return roundf(angle4msR);
}
