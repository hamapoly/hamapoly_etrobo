#ifndef INCLUDED_Run_h_
#define INCLUDED_Run_h_

// #include "math.h"        Grid.hで記述
// #include "Distance"      Direction.hで記述
#include "Direction.h"
#include "Grid.h"

/* 関数プロトタイプ宣言 */

// 走行ログ用の関数
extern void log_stamp(char *stamp); // app.cで定義した(staticでない)関数は宣言の必要は無いようですが、一応各区間のソースファイルで利用するのでここで宣言

// 初期化・値更新関数
void Run_init();
void Run_update();

// 値取得関数
uint16_t getRGB_R();
uint16_t getRGB_G();
uint16_t getRGB_B();
int16_t  Run_getAngle();
int8_t   Run_getPower();
int16_t  Run_getTurn();
uint32_t Run_getTime();

// 返り値の最大・最小値を制限する関数
float   math_limit(float n, float min, float max);

// モーターの制御を行う関数(ev3_motor_steerの代替)
void    motor_ctrl(int8_t power, int16_t turn);

// アームの上下を制御する関数
void    arm_up(uint8_t power, bool_t loop);
void    arm_down(uint8_t power, bool_t loop);

// テールの開閉を制御する関数
void    tale_open(uint8_t power, bool_t loop);
void    tale_close(uint8_t power, bool_t loop);


// ラインを検知したらその場で停止する関数(引数loopで内部ループの有無を選択可能)
void    Run_setStop_Line(bool_t loop);

// 指定した距離に到達するまで、指定出力で移動または旋回する関数
void    Run_setDistance(int8_t power, int16_t turn, float distance);

// 指定した方位に到達するまで、指定出力で旋回または移動する関数
void    Run_setDirection(int8_t power, int16_t turn, float direction);

// 指定した距離に障害物を検知するまで、指定出力で前進または旋回する関数(引数distanceで追加条件として移動距離を設定可能)
void    Run_setDetection(int8_t power, int16_t turn, int16_t detection, float distance);


// PID初期化関数
void    Run_PID_init();

// PID制御関数(定数) * (センサ入力値 - 目標値)
int16_t Run_getTurn_sensorPID(uint16_t sensor_val, uint16_t target_val);


// 目標の出力値に到達するまで、指定量の出力値の増減を行い、その結果を返す関数
int8_t  Run_getPower_change(int8_t current_power, int8_t target_power, float change_rate);

// 目標の出力値に到達するまで、指定量の出力値の増減を行い、その結果を返す関数
int8_t  Run_getTurn_change(int8_t current_turn, int8_t target_turn, float change_rate);

// モーターの制御を加減速を伴って行う関数
void    motor_ctrl_alt(int8_t power, int16_t turn, float change_rate);


//パターン判別のためにサンプリングを行う関数
int8_t  sampling_sonic(void);

// サンプリングを用いた直進検知関数
int8_t  sampling_turn(int16_t turn);

#endif