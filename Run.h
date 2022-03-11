#ifndef INCLUDED_Run_h_
#define INCLUDED_Run_h_

#include "ev3api.h"

/* 関数プロトタイプ宣言 */

// 走行ログ用の関数
extern void log_stamp(char *stamp); // app.cで定義した(staticでない)関数は宣言の必要は無いようですが、一応各区間のソースファイルで利用するのでここで宣言

// 初期化・値更新関数
void Run_init();    // 走行データを初期化
void Run_update();  // 走行データを更新

// 値取得関数
uint16_t Run_getRGB_R();
uint16_t Run_getRGB_G();
uint16_t Run_getRGB_B();
uint32_t Run_getTime();
int8_t   Run_getPower();
int8_t   Run_getPower_L();
int8_t   Run_getPower_R();
int16_t  Run_getTurn();
int16_t  Run_getAngle();
float    Run_getDistance();
float    Run_getDirection();
float    Run_getSpeed();

// 計測値更新用の関数群
//---------------------------------------------------------------------------------------------------------------------------------
void Run_updateMotor();
void Run_updateSpeed();
void Run_updateSamplingTime(uint16_t cur_value);

// 距離計測用関数群(引用：https://qiita.com/TetsuroAkagawa/items/ba6190f08d26df7cc8ad)
//---------------------------------------------------------------------------------------------------------------------------------
/* 初期化関数 */
void Run_initDistance();

/* 距離を更新 */
void Run_updateDistance();

/* 右タイヤの4ms間の距離を取得 */
float Run_getDistance4msRight();

/* 左タイヤの4ms間の距離を取得 */
float Run_getDistance4msLeft();

/* 右タイヤの4ms間の角度を取得 */
int Run_getAngle4msLeft();

/* 左タイヤの4ms間の角度を取得 */
int Run_getAngle4msRight();

// 方位計測用関数群(引用：https://qiita.com/TetsuroAkagawa/items/4e7de30523d9c7ec6241)
//---------------------------------------------------------------------------------------------------------------------------------
/* 初期化 */
void Run_initDirection();

 // 方位を更新
void Run_updateDirection();

#endif