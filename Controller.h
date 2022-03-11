#ifndef INCLUDED_Controller_h_
#define INCLUDED_Controller_h_

#include <math.h>
#include "Run.h"

/* 関数プロトタイプ宣言 */

// タスクとハンドラ
extern void shutdown_task(intptr_t exinf);  // Mainタスクに並行して(=Mainタスクのスリープ中に)実行される測定値書き込み関数

extern void datalog_cyc(intptr_t);          // 周期ハンドラによって5msごとに計測値の更新を行う関数

// 返り値の最大・最小値を制限する関数
float   Ctrl_math_limit(float n, float min, float max);

// モーターの制御を行う関数(ev3_motor_steerの代替)
void    Ctrl_motor_steer(int8_t power, int16_t turn);

// モーターの制御を加減速を伴って行う関数
void    Ctrl_motor_steer_alt(int8_t power, int16_t turn, float change_rate);


// アームの上下を制御する関数
void    Ctrl_arm_up(uint8_t power, bool_t loop);
void    Ctrl_arm_down(uint8_t power, bool_t loop);

// テールの開閉を制御する関数
void    Ctrl_tale_open(uint8_t power, bool_t loop);
void    Ctrl_tale_close(uint8_t power, bool_t loop);


// ラインを検知したらその場で停止する関数(引数loopで内部ループの有無を選択可能)
void    Ctrl_runStop_Line(bool_t loop);

// 指定した距離に到達するまで、指定出力で移動または旋回する関数
void    Ctrl_runDistance(int8_t power, int16_t turn, float distance);

// 指定した方位に到達するまで、指定出力で旋回または移動する関数
void    Ctrl_runDirection(int8_t power, int16_t turn, float direction);

// 指定した距離に障害物を検知するまで、指定出力で前進または旋回する関数(引数distanceで移動距離も条件として設定可能)
void    Ctrl_runDetection(int8_t power, int16_t turn, int16_t detection, float distance);


// PID初期化関数
void    Ctrl_initPID();

// PID制御関数(定数) * (センサ入力値 - 目標値)
int16_t Ctrl_getTurn_PID(uint16_t sensor_val, uint16_t target_val);


// 目標の出力値に到達するまで、指定量の出力値の増減を行い、その結果を返す関数
int8_t  Ctrl_getPower_Change(int8_t target_power, float change_rate);

// 目標の出力値に到達するまで、指定量の出力値の増減を行い、その結果を返す関数
int8_t  Ctrl_getTurn_Change(int8_t target_turn, float change_rate);


//パターン判別のためにサンプリングを行う関数
int8_t  sampling_sonic(void);

// サンプリングを用いた直進検知関数
int8_t  sampling_turn(int16_t turn);

#endif