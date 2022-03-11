#include "Controller.h"

// 英単語の省略表記についての参考サイト
// https://progeigo.org/learning/essential-words-600-plus/#abbreviation-70
// https://qiita.com/Ted-HM/items/7dde25dcffae4cdc7923
// https://qiita.com/mtanabe/items/48f9b2f5167d5a3b2d71

/* マクロ定義 */
// R/Lコースの変換
#define EDGE 1  // 1でLコース、-1でRコース

// PID用の定義
#define DELTA_T 0.004   // 処理周期(4msの場合)
// 下記のPID値が走行に与える影響については次のサイトが参考になります https://www.tsone.co.jp/blog/archives/889
#define KP      1.38    // sim_power100 1.68     //sim_power80-70 1.68     //実機_power50 1.38
#define KI      0.0     // sim_power100 0.47?    //sim_power80-70 0.00     //実機_power50 0.00
#define KD      0.15    // sim_power100 0.50     //sim_power80-70 0.30     //実機_power50 0.15

/* グローバル宣言 */
static int32_t diff[2] = {0, 0};    // PID制御の偏差用変数
static float integral = 0.0;        // PID制御の積分用変数

static int8_t input_power = 0;      // 現在のモーターへの入力値を保存
static int8_t input_turn = 0;      // 現在の旋回量を保存

/* 戻り値の最大・最小値を制限する関数 *************************************/
// n    : 制限したい値
// max  : 最大値
// min  : 最小値
//
// 戻り値 : 最大・最小値に調整された値
/***********************************************************************/
float Ctrl_math_limit(float n, float min, float max)
{
    if(n > max)
    {
        n = max;
    }
    else if(n < min)
    {
        n = min;
    }
    return n;
}

/* モーター制御関数 *************************************************************************************************************************/
// ev3_motor_steer関数の代替(ev3_motor_steer関数はetroboシミュレータ環境では非推奨となっているため)
// > 参照：https://github.com/ETrobocon/etrobo/wiki/api_ev3rt_on_athrill
//
// 使用グローバル変数
//  input_power : 現在のモーターへの入力値を保存.　実機でCtrl_getPower_Change関数を利用するために使用
//  input_turn  : 現在の旋回量を保存.　実機でCtrl_getTurn_Change関数を利用するために使用
//
// 引数
//  power    : モータの出力値．範囲：-100から+100．マイナスの値は後退．
//  turn     : ステアリングの度合い．範囲：-200から+200．マイナスの値は左への転回，プラスの値は右への転回になる．
//             具体的に言えば，このパラメータはこの左右モータの出力の差の度合いである．例えば， turn が+25である場合，
//             左モータの出力は power で，右モータの出力は power の75\%になり，ロボットは右へ転回する．(ev3_motor_steer関数より引用)
//
// 例       : Ctrl_motor_steer(100, 100)の場合 モーター出力は(左  100, 右    0) となり、右の車輪を軸に右方向に旋回する
//            Ctrl_motor_steer(100, 200)の場合 モーター出力は(左  100, 右 -100) となり、その場で右方向に旋回する
//            Ctrl_motor_steer( 50,  50)の場合 モーター出力は(左   50, 右   25) となり、右方向に曲がりつつ前進する
/******************************************************************************************************************************************/
void Ctrl_motor_steer(int8_t power, int16_t turn)
{
    turn = turn * EDGE;   // R/Lコースの変換処理

    input_power = power;    // 現在の入力値を記録
    input_turn  = turn;     // 現在の入力値を記録

    if(power < -100 || power > 100 || turn < -200 || turn > 200)    // 引数が許容範囲に収まっていない場合
    {
        power   = Ctrl_math_limit(power, -100, 100);                         // Ctrl_math_limit関数を利用して
        turn    = Ctrl_math_limit(turn, -200, 200);                          // 値を許容範囲に収めてから走行処理を行う
    }
    
    if(power != 0 && turn == 0)                                     // 前後進
    {
        ev3_motor_set_power(EV3_PORT_C, power);
        ev3_motor_set_power(EV3_PORT_B, power);
    }
    else if(turn > 0)                                               // 右旋回
    {
        ev3_motor_set_power(EV3_PORT_C, power);
        ev3_motor_set_power(EV3_PORT_B, power - (turn * power / 100)); // turnをpowerの比率に合わせる
    }
    else if(turn < 0)                                               // 左旋回
    {
        ev3_motor_set_power(EV3_PORT_C, power + (turn * power / 100));  // turnをpowerの比率に合わせる
        ev3_motor_set_power(EV3_PORT_B, power);
    }
    else                                                            // 引数(0, 0)で左右モーター停止
    {
        ev3_motor_stop(EV3_PORT_C, true);
        ev3_motor_stop(EV3_PORT_B, true);
    }
}

/* モーター制御関数alt *********************************************************************************************************************/
// 加減速機能付きのCtrl_motor_steer関数
/******************************************************************************************************************************************/
void Ctrl_motor_steer_alt(int8_t power, int16_t turn, float change_rate)
{
    power = Ctrl_getPower_Change(power, change_rate); // 出力調整

    Ctrl_motor_steer(power, turn);
}

//*****************************************************************************
// 関数名 : arm_up, arm_down
// 引数 : 無し
// 戻り値 : 無し
// 概要 : アームの上げ/下げを行う
// sim  : 初期角度 -56, 最大角度 40, 最低角度 -70
// 実機 : 初期角度 25, 最大角度 80, 最低角度 0
//*****************************************************************************
// アーム上昇制御関数
void Ctrl_arm_up(uint8_t power, bool_t loop)
{
    int32_t cur_angle = ev3_motor_get_counts(EV3_PORT_A);    // 現在のモーター角度
    int8_t cur_power = ev3_motor_get_power(EV3_PORT_A);      // 現在のモーター出力

    do
    {
        // printf("%d ", cur_angle);
        if(cur_angle < 20)                                     // 指定角度に到達していない場合
        {
            if(cur_power < power)                                   // 指定出力に到達していない場合
                ++cur_power;                                            // 出力増加
        }
        else                                                    // モーター角度が指定角度に到達した場合
        {
            if(cur_power > 0)                                       // モーター出力が0になっていない場合
            {
                --cur_power;                                            // 出力減少
            }
            else                                                    // モーター出力が0となった場合
            {
                ev3_motor_stop(EV3_PORT_A, true);                       // モーターを停止
                return;                                                 // 関数を終了
            }
        }
        ev3_motor_set_power(EV3_PORT_A, cur_power);             // アームモーターに出力を設定

        if(loop)                    // loopがtrueの場合
        {
            tslp_tsk(4 * 1000U);    /* 4msec周期起動 */
            cur_angle = ev3_motor_get_counts(EV3_PORT_A);    // 現在のモーター角度を更新
            cur_power = ev3_motor_get_power(EV3_PORT_A);     // 現在のモーター出力を更新
        }
    }
    while(loop);
}

// アーム下降制御関数
void Ctrl_arm_down(uint8_t power, bool_t loop)
{
    int32_t cur_angle = ev3_motor_get_counts(EV3_PORT_A);    // 現在のモーター角度
    int8_t cur_power = ev3_motor_get_power(EV3_PORT_A);      // 現在のモーター出力

    do
    {
        // printf("%d ", cur_angle);
        if(cur_angle > -5)                                             // 指定角度に到達していない場合
        {
            if(cur_power > power * -1)                                      // 指定出力に到達していない場合
                --cur_power;                                                    // 出力増加
        }
        else                                                            // モーター角度が指定角度に到達した場合
        {
            if(cur_power < 0)                                               // モーター出力が0になっていない場合
            {
                ++cur_power;                                                    // 出力減少
            }
            else                                                            // モーター出力が0となった場合
            {
                ev3_motor_stop(EV3_PORT_A, true);                               // モーターを停止
                return;                                                         // 関数を終了
            }
        }
        ev3_motor_set_power(EV3_PORT_A, cur_power);                     // アームモーターに出力を設定

        if(loop)                    // loopがtrueの場合
        {
            tslp_tsk(4 * 1000U);    /* 4msec周期起動 */
            cur_angle = ev3_motor_get_counts(EV3_PORT_A);    // 現在のモーター角度を更新
            cur_power = ev3_motor_get_power(EV3_PORT_A);     // 現在のモーター出力を更新
        }
    }
    while(loop);
}

//*****************************************************************************
// 関数名 : tale_up, tale_down
// 引数 : 無し
// 戻り値 : 無し
// 概要 : 尻尾の開閉を行う
// 初期角度 4, 最大角度 3896
//*****************************************************************************
// テール開制開開御関数
void Ctrl_tale_open(uint8_t power, bool_t loop)
{
    int32_t cur_angle = ev3_motor_get_counts(EV3_PORT_D);   // 現在のモーター角度
    int8_t cur_power = ev3_motor_get_power(EV3_PORT_D);     // 現在のモーター出力

    do
    {
        // printf("%d ", cur_angle);
        if(cur_angle < 3800)                                    // 指定角度に到達していない場合
        {
            if(cur_power < power)                                   // 指定出力に到達していない場合
                ++cur_power;                                            // 出力増加
        }
        else                                                    // モーター角度が指定角度に到達した場合
        {
            if(cur_power > 0)                                       // モーター出力が0になっていない場合
            {
                --cur_power;                                            // 出力減少
            }
            else                                                    // モーター出力が0となった場合
            {
                ev3_motor_stop(EV3_PORT_D, true);                       // モーターを停止
                return;                                                 // 関数を終了
            }
        }
        ev3_motor_set_power(EV3_PORT_D, cur_power);             // テールモーターに出力を設定

        if(loop)                    // loopがtrueの場合
        {
            tslp_tsk(4 * 1000U);    /* 4msec周期起動 */
            cur_angle = ev3_motor_get_counts(EV3_PORT_D);    // 現在のモーター角度を更新
            cur_power = ev3_motor_get_power(EV3_PORT_D);     // 現在のモーター出力を更新
        }
    }
    while(loop);
}

// テール閉制御関数
void Ctrl_tale_close(uint8_t power, bool_t loop)
{
    int32_t cur_angle = ev3_motor_get_counts(EV3_PORT_D);   // 現在のモーター角度
    int8_t cur_power = ev3_motor_get_power(EV3_PORT_D);     // 現在のモーター出力

    do
    {
        // printf("%d ", cur_angle);
        if(cur_angle > 200)                                             // 指定角度に到達していない場合
        {
            if(cur_power > power * -1)                                      // 指定出力に到達していない場合
                --cur_power;                                                    // 出力増加
        }
        else                                                            // モーター角度が指定角度に到達した場合
        {
            if(cur_power < 0)                                               // モーター出力が0になっていない場合
            {
                ++cur_power;                                                    // 出力減少
            }
            else                                                            // モーター出力が0となった場合
            {
                ev3_motor_stop(EV3_PORT_D, true);                               // モーターを停止
                return;                                                         // 関数を終了
            }
        }
        ev3_motor_set_power(EV3_PORT_D, cur_power);                     // アームモーターに出力を設定

        if(loop)                    // loopがtrueの場合
        {
            tslp_tsk(4 * 1000U);    /* 4msec周期起動 */
            cur_angle = ev3_motor_get_counts(EV3_PORT_D);   // 現在のモーター角度を更新
            cur_power = ev3_motor_get_power(EV3_PORT_D);    // 現在のモーター出力を更新
        }
    }
    while(loop);
}


/* 黒ラインを検知したらその場で停止する関数 ******************************************/
// do ~ while(0) > 参考：https://qiita.com/ymko/items/ae8e056a270558f7fbaf
//
// loop : true (関数は停止が完了してからリターン)，false (関数は停止を待たずにリターン)
/*********************************************************************************/
void Ctrl_runStop_Line(bool_t loop)
{
    do
    {
        if(Run_getRGB_R() < 60 && Run_getRGB_G() < 90 && Run_getRGB_B() < 90)  // 黒ラインを検知した場合
        {
            Ctrl_motor_steer(0, 0);     // 左右モーター停止
            return;                     // 関数を終了
        }

        if(loop)                    // ループ処理の場合
            tslp_tsk(4 * 1000U);    /* 4msec周期起動 */
    }
    while(loop);
}

/* 指定した距離に到達するまで、指定出力で移動または旋回する関数 *********************************/
// 加減速ありの場合は関数Ctrl_motor_steer_altを使用する
//
// 引数
//  power        : Ctrl_motor_steer関数のpower値(-100 ~ +100)
//  turn         : Ctrl_motor_steer関数のturn値(-200 ~ +200)
//  distance     : 移動する距離
/******************************************************************************************/
void Ctrl_runDistance(int8_t power, int16_t turn, float distance)
{
    float ref_distance = Run_getDistance();             // 処理開始時点での距離を取得

    if(power > 0 && distance > 0)                       // 前進の場合
    {
        while(1)                                            // モーターが停止するまでループ
        {
            if(Run_getDistance() >= (ref_distance + distance))  // 指定距離に到達した場合
            {
                Ctrl_motor_steer_alt(0, turn, 0.1);                 // モーターが停止するまで減速
                if(Run_getPower() == 0)                             // モーターが完全に停止した場合
                    return;                                             // 関数を終了
            }
            else                                                // 指定距離に到達していない場合
            {
                Ctrl_motor_steer_alt(power, turn, 0.1);             // 指定出力になるまで加速して走行
            }
            tslp_tsk(4 * 1000U); /* 4msec周期起動 */
        }
    }
    else if(power < 0 && distance < 0)                  // 後退の場合
    {
        while(1)                                            // モーターが停止するまでループ
        {
            if(Run_getDistance() <= (ref_distance + distance))  // 指定距離に到達した場合
            {
                Ctrl_motor_steer_alt(0, turn, 0.1);                 // モーターが停止するまで減速
                if(Run_getPower() == 0)                             // モーターが完全に停止した場合
                    return;                                             // 関数を終了
            }
            else                                                // 指定距離に到達していない場合
            {
                Ctrl_motor_steer_alt(power, turn, 0.1);             // 指定出力になるまで加速して走行
            }
            tslp_tsk(4 * 1000U); /* 4msec周期起動 */
        }
    }
    else                                                // 正しい引数が得られなかった場合
    {
        printf("argument out of range @ Ctrl_runDistance()\n"); // エラーメッセージを出して
        return;                                                 // 終了
    }
}

/* 指定した方位に到達するまで、指定出力で旋回または移動する関数 *********************************/
// 加減速ありの場合は関数Ctrl_motor_steer_altを使用する
//
// 引数
//  power        : Ctrl_motor_steer関数のpower値(-100 ~ +100)
//  turn         : Ctrl_motor_steer関数のturn値(-200 ~ +200)
//  direction    : 旋回する方位
/******************************************************************************************/
void Ctrl_runDirection(int8_t power, int16_t turn, float direction)
{
    float ref_direction = Run_getDirection();               // 処理開始時点での方位を取得
    
    direction = direction * EDGE;                           // R/Lコースの変換処理
    
    if(power != 0 && turn > 0 && direction > 0)             // 右旋回の場合
    {
        while(1)                                                // モーターが停止するまでループ
        {
            if(Run_getDirection() >= (ref_direction + direction))   // 指定方位に到達した場合
            {
                Ctrl_motor_steer_alt(0, turn, 0.1);                     // モーターが停止するまで減速
                if(Run_getPower() == 0)                                 // モーターが完全に停止した場合
                    return;                                                 // 関数を終了
            }
            else                                                    // 指定方位に到達していない場合
            {
                Ctrl_motor_steer_alt(power, turn, 0.1);                 // 指定出力になるまで加速して旋回
            }
            tslp_tsk(4 * 1000U); /* 4msec周期起動 */
        }
    }
    else if(power != 0 && turn < 0 && direction < 0)        // 左旋回の場合
    {
        while(1)                                                // モーターが停止するまでループ
        {
            if(Run_getDirection() <= (ref_direction + direction))   // 指定方位に到達した場合
            {
                Ctrl_motor_steer_alt(0, turn, 0.1);                     // モーターが停止するまで減速
                if(Run_getPower() == 0)                                 // モーターが完全に停止した場合
                    return;                                                 // 関数を終了
            }
            else                                                    // 指定方位に到達していない場合
            {
                Ctrl_motor_steer_alt(power, turn, 0.1);                 // 指定出力になるまで加速して旋回
            }
            tslp_tsk(4 * 1000U); /* 4msec周期起動 */
        }
    }
    else                                                    // 正しい引数が得られなかった場合
    {
        printf("argument out of range @ Ctrl_runDirection()\n");    // エラーメッセージを出して
        return;                                                     // 異常終了
    }
}

/* 指定した距離に障害物を検知するまで、指定出力で前進または旋回する関数 ***********************/
// 加減速ありの場合は関数Ctrl_motor_steer_altを使用する
//
// 引数
//  power        : Ctrl_motor_steer関数のpower値(-100 ~ +100)
//  turn         : Ctrl_motor_steer関数のturn値(-200 ~ +200)
//  detection    : 障害物を検知する距離
//  distance     : 障害物検知に加えて、指定の距離で停止する条件を追加する(0で無効)
/****************************************************************************************/
void Ctrl_runDetection(int8_t power, int16_t turn, int16_t detection, float distance)
{
    float ref_distance = Run_getDistance();                         // 処理開始時点での距離を取得
    
    if(power > 0 && distance == 0)                                      // 距離の指定がない場合
    {
        while(1)                                                            // モーターが停止するまでループ
        {
            if(ev3_ultrasonic_sensor_get_distance(EV3_PORT_3) <= detection)     // 障害物を検知した場合
            {
                Ctrl_motor_steer_alt(0, turn, 0.1);                                 // モーターが停止するまで減速
                if(Run_getPower() == 0)                                             // モーターが完全に停止した場合
                    return;                                                             // 関数を終了
            }
            else                                                                // 障害物を検知していない場合
            {
                Ctrl_motor_steer_alt(power, turn, 0.1);                                   // 指定出力になるまで加速して走行
            }
            tslp_tsk(4 * 1000U); /* 4msec周期起動 */
        }
    }
    else if(power > 0 && distance > 0)                              // 距離の指定がある場合
    {        
        while(1)                                                        // モーターが停止するまでループ
        {
            if(ev3_ultrasonic_sensor_get_distance(EV3_PORT_3) <= detection || Run_getDistance() >= (ref_distance + distance))
            {                                                               // 障害物を検知した場合、または指定距離に到達した場合
                Ctrl_motor_steer_alt(0, turn, 0.1);                             // モーターが停止するまで減速
                if(Run_getPower() == 0)                                         // モーターが完全に停止した場合
                    return;                                                         // 関数を終了
            }
            else                                                            // 障害物を未検知、かつ指定距離に到達していない場合
            {
                Ctrl_motor_steer_alt(power, turn, 0.1);                         // 指定出力になるまで加速して走行
            }
            tslp_tsk(4 * 1000U); /* 4msec周期起動 */
        }
    }
    else                                                            // 正しい引数が得られなかった場合
    {
        printf("argument out of range @ Ctrl_runDetection()\n");        // エラーメッセージを出して
        return;                                                         // 異常終了
    }
}


/* PID初期化関数 *********************************************************/
// PID用のグローバル変数の初期化
/************************************************************************/
void Ctrl_initPID()
{
    diff[0] = 0;
    integral = 0.0;
}

/* PID制御関数(定数) * (センサー入力値 - 目標値) **********************************************************/
// 参考：https://monoist.atmarkit.co.jp/mn/articles/1007/26/news083.html
//
// 使用グローバル変数
// int32_t  diff[2]    : 偏差を記録
// float    integral   : 積分を記録
//
// 引数
// sensor_val   : センサーの現在値
// taget_val    : センサーの目標値
//
// 戻り値       : Ctrl_motor_steer関数のturn値(-200 ~ +200)
/*******************************************************************************************************/
int16_t Ctrl_getTurn_PID(uint16_t sensor_val, uint16_t target_val)    // センサー値, センサーの目標値
{
    float p, i, d;

    diff[0] = diff[1];
    diff[1] = sensor_val - target_val;  // 偏差を取得
    integral += (diff[1] + diff[0]) / 2.0 * DELTA_T;

    p = KP * diff[1];
    i = KI * integral;
    d = KD * (diff[1] - diff[0]) / DELTA_T;

    return roundf(Ctrl_math_limit(p + i + d, -200.0, 200.0));    // 最大・最小値を制限し、四捨五入した値を返す
}


/* 目標の出力値に到達するまで、指定量の出力値の増減を行い、その結果を返す関数 *********************************************************************/
// 徐々に加速、減速を行えるようにするための関数。線形で示すと、通常の加減速は _|￣|_ であり、この関数で実現したい加減速は _／￣＼_　のような形。
//
// 使用グローバル変数
//  input_power :   実機では、入力した出力と実際の出力が異なることがあり、現在の出力を参照すると加減速が不安定になるため、代替として使用
//
// 引数
//  target_power  : 目標の出力値
//  change_rate   : 増減の変化量 *例として0.2とした場合、4ms(1周期)で出力値が0.2ずつ変化し、20ms経過すると出力値が 1 変化することになる
//
// 戻り値        : 指定量の加減速を行ったモーターの出力値(小数点以下の値はモーター制御の関数が対応していないため切り捨て)
/*******************************************************************************************************************************************/
int8_t Ctrl_getPower_Change(int8_t target_power, float change_rate)
{
    static float power = 0.0;               // 出力値を保持する変数
    int8_t current_power = input_power;     // 現在の入力値

    if(current_power < target_power)        // 現在値 < 目標値 の時
    {
        if(floorf(power) != current_power)      // 現在値が指定した変化量を超えて増減した場合の対策
            power = (float)current_power;           // 出力値を上書き

        power = power + change_rate;            // 変化量の分だけ増加
        return floorf(power);                   // 小数点以下を切り捨てて値を返す
    }
    else if(current_power > target_power)   // 現在値 > 目標値 の時
    {
        if(ceilf(power) != current_power)       // 現在値が指定した変化量を超えて増減した場合の対策
            power = (float)current_power;           // 出力値を上書き

        power = power - change_rate;            // 変化量の分だけ減少
        return ceilf(power);                    // 小数点以下を切り上げて値を返す
    }
    else                                    // 現在値が目標値に到達している場合
        return current_power;                   // 現在値をそのまま返す
}

/* 目標の出力値に到達するまで、指定量の出力値の増減を行い、その結果を返す関数 *********************************************************************/
int8_t Ctrl_getTurn_Change(int8_t target_turn, float change_rate)
{
    static float turn = 0.0;            // 出力値を保持する変数
    int8_t current_turn = input_turn;   // 現在の入力値

    if(current_turn < target_turn)      // 現在値 < 目標値 の時
    {
        if(floorf(turn) != current_turn)    // 現在値が指定した変化量を超えて増減した場合の対策
            turn = (float)current_turn;         // 出力値を上書き

        turn = turn + change_rate;          // 変化量の分だけ増加
        return floorf(turn);                // 小数点以下を切り捨てて値を返す
    }
    else if(current_turn > target_turn) // 現在値 > 目標値 の時
    {
        if(ceilf(turn) != current_turn)     // 現在値が指定した変化量を超えて増減した場合の対策
            turn = (float)current_turn;         // 出力値を上書き

        turn = turn - change_rate;          // 変化量の分だけ減少
        return ceilf(turn);                 // 小数点以下を切り上げて値を返す
    }
    else                                // 現在値が目標値に到達している場合
        return current_turn;                // 現在値をそのまま返す
}

/* サンプリングを用いたパターン判別関数*******************************************************************************************************/
// 説明: パターン判別のためにサンプリングを１００回行う。
//       過半数以上の障害物検知をした場合のみパターンAと判別する。
/******************************************************************************************************************************************/
int8_t sampling_sonic(void)
{
    int loop;
    int sampling_cnt = 0;
    int8_t pattern;

    for(loop=0;loop<100;loop++)                                                 // サンプリングを１００回行う
    {
        if(ev3_ultrasonic_sensor_get_distance(EV3_PORT_3) <= 25)
            sampling_cnt++;
        tslp_tsk(4 * 1000U); /* 4msec周期起動 */
    }

    if(sampling_cnt >= 80)                                                      // サンプリングを基にパターン判別を行う
        pattern = 1;
    else
        pattern = 0;

    return pattern;
}

/* サンプリングを用いた直進検知関数***********************************************/
int8_t sampling_turn(int16_t turn)
{
    static uint8_t flag = 0;
    static uint8_t cnt = 0;
    static int16_t sampling_data[100] = {0};

    uint8_t i = 0;
    int16_t avg = 0;

    if(cnt < 100)
    {
        if(turn < 0)
            sampling_data[cnt] = turn * (-1);
        else
            sampling_data[cnt] = turn;

        cnt++;

        if(cnt == 99)
        {
            cnt = 0;
            flag = 1;
        }
    }

    for(i = 0; i < 100; i++)
    {
        avg += sampling_data[i];
    }
    avg = avg / 100;

    if(flag == 1 && avg < 7)
        return 1;
    else
        return 0;
}