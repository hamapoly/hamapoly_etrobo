#ifndef _DISTANCE_H_
#define _DISTANCE_H_

#include "ev3api.h"
// #include "parameter.h"

/* 円周率 */
#define PI 3.14159265358

/* 初期化関数 */
void Distance_init();

/* 距離を更新 */
void Distance_update();

/* 走行距離を取得 */
float Distance_getDistance();

/* 右タイヤの4ms間の距離を取得 */
float Distance_getDistance4msRight();

/* 左タイヤの4ms間の距離を取得 */
float Distance_getDistance4msLeft();

/* 右タイヤの4ms間の角度を取得 */
int Distance_getAngle4msLeft();

/* 左タイヤの4ms間の角度を取得 */
int Distance_getAngle4msRight();

#endif