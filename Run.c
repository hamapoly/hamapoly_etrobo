#include "Run.h"

// Rコース用の変換処理

/* マクロ定義 */
#define DELTA_T 0.004   // 処理周期(4msの場合)
// 下記のPID値が走行に与える影響については次のサイトが参考になります https://www.tsone.co.jp/blog/archives/889
#define KP      1.68    // power100_1.68
#define KI      0.0     // power100_0.47?
#define KD      0.3     // power100_0.50

/* グローバル変数 */    // static宣言されたグローバル変数の範囲(スコープ)は、宣言した.cファイル内に限定される
static const sensor_port_t
    color_sensor    = EV3_PORT_2,
    sonar_sensor    = EV3_PORT_3,
    gyro_sensor     = EV3_PORT_4;

static const motor_port_t
    left_motor      = EV3_PORT_C,
    right_motor     = EV3_PORT_B,
    arm_motor       = EV3_PORT_A,
    tale_motor      = EV3_PORT_D;

static rgb_raw_t rgb;
static int8_t   run_power = 0;
static int16_t  run_turn = 0;
static int16_t  run_angle = 0;
static uint32_t run_time = 0;
static SYSTIM   sampling_time = 0;
static float    speed = 0;

static int32_t diff[2] = {0, 0};    // PID制御用(カラーセンサー)
static float integral = 0.0;

/* 関数 */

void Run_init(void)
{
    run_time = 0;
}

void Run_update(void)
{
    ++run_time;                                         // 走行時間を加算
    ev3_color_sensor_get_rgb_raw(color_sensor, &rgb);   // RGB値を更新
    run_angle = ev3_gyro_sensor_get_angle(gyro_sensor); // 位置角(傾き)を更新
}

uint16_t getRGB_R(void){    // カラーセンサーのR値を取得
    return rgb.r;
}

uint16_t getRGB_G(void){    // カラーセンサーのG値を取得
    return rgb.g;
}

uint16_t getRGB_B(void){    // カラーセンサーのB値を取得
    return rgb.b;
}

int8_t Run_getPower(void){  // モーター出力を取得(motor_ctrlでモーターを制御している必要がある)
    return run_power;
}

int16_t Run_getTurn(void){  // 旋回値を取得(motor_ctrlでモーターを制御している必要がある)
    return run_turn;
}

int16_t Run_getAngle(void){ // 位置角(傾き)を取得
    return run_angle;
}

uint32_t Run_getTime(void){ // 走行時間を取得(5ms単位) <- 周期ハンドラによって5msごとに更新されるため
    return run_time;
}

SYSTIM Run_getSamplingTime(void){    // センサのサンプリング周期を取得
    return sampling_time;
}

float Run_getSpeed(void){           // 100[ms]ごとの速度を取得
    return speed;
}

/* 返り値の最大・最小値を制限する関数 *************************************/
// n    : 制限したい値
// max  : 最大値
// min  : 最小値
//
// 返り値 : 最大・最小値に調整された値
/***********************************************************************/
float math_limit(float n, float min, float max)
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
// power    : モータの出力値．範囲：-100から+100．マイナスの値は後退．
// turn     : ステアリングの度合い．範囲：-200から+200．マイナスの値は左への転回，プラスの値は右への転回になる．
//             具体的に言えば，このパラメータはこの左右モータの出力の差の度合いである．例えば， turn が+25である場合，
//             左モータの出力は power で，右モータの出力は power の75\%になり，ロボットは右へ転回する．(ev3_motor_steer関数より引用)
//
// 例       : motor_ctrl(100, 100)の場合 モーター出力は(左  100, 右    0) となり、右の車輪を軸に右方向に旋回する
//            motor_ctrl(100, 200)の場合 モーター出力は(左  100, 右 -100) となり、その場で右方向に旋回する
//            motor_ctrl( 50,  50)の場合 モーター出力は(左   50, 右   25) となり、右方向に曲がりつつ前進する
/******************************************************************************************************************************************/
void motor_ctrl(int8_t power, int16_t turn)
{
    // turn = turn * -1;   // Rコース用の変換処理

    run_power = power;  // 計測用の変数を更新
    run_turn = turn;    // 計測用の変数を更新

    if(power < -100 || power > 100 || turn < -200 || turn > 200)    // 引数が許容範囲に収まっていない場合
    {
        power   = math_limit(power, -100, 100);                         // math_limit関数を利用して
        turn    = math_limit(turn, -200, 200);                          // 値を許容範囲に収めてから走行処理を行う
    }
    
    if(power != 0 && turn == 0)                                     // 前後進
    {
        ev3_motor_set_power(left_motor, power);
        ev3_motor_set_power(right_motor, power);
    }
    else if(turn > 0)                                               // 右旋回
    {
        ev3_motor_set_power(left_motor, power);
        ev3_motor_set_power(right_motor, power - (turn * power / 100)); // turnをpowerの比率に合わせる
    }
    else if(turn < 0)                                               // 左旋回
    {
        ev3_motor_set_power(left_motor, power + (turn * power / 100));  // turnをpowerの比率に合わせる
        ev3_motor_set_power(right_motor, power);
    }
    else                                                            // 引数(0, 0)で左右モーター停止
    {
        ev3_motor_stop(left_motor, true);
        ev3_motor_stop(right_motor, true);
    }
}

//*****************************************************************************
// 関数名 : arm_up, arm_down
// 引数 : 無し
// 返り値 : 無し
// 概要 : アームの上げ/下げを行う
// 初期角度 -56, 最大角度 40, 最低角度 -70
//*****************************************************************************
// アーム上昇制御関数
void arm_up(uint8_t power, bool_t loop)
{
    int32_t cur_angle = ev3_motor_get_counts(arm_motor);    // 現在のモーター角度
    int8_t cur_power = ev3_motor_get_power(arm_motor);      // 現在のモーター出力

    do
    {
        // printf("%d ", cur_angle);
        if(cur_angle < -20)                                     // 指定角度に到達していない場合
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
                ev3_motor_stop(arm_motor, true);                        // モーターを停止
                return;                                                 // 関数を終了
            }
        }
        ev3_motor_set_power(arm_motor, cur_power);              // アームモーターに出力を設定

        if(loop)                    // loopがtrueの場合
        {
            tslp_tsk(4 * 1000U);    /* 4msec周期起動 */
            cur_angle = ev3_motor_get_counts(arm_motor);    // 現在のモーター角度を更新
            cur_power = ev3_motor_get_power(arm_motor);     // 現在のモーター出力を更新
        }
    }
    while(loop);
}

// アーム下降制御関数
void arm_down(uint8_t power, bool_t loop)
{
    int32_t cur_angle = ev3_motor_get_counts(arm_motor);    // 現在のモーター角度
    int8_t cur_power = ev3_motor_get_power(arm_motor);      // 現在のモーター出力

    do
    {
        // printf("%d ", cur_angle);
        if(cur_angle > -47)                                             // 指定角度に到達していない場合
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
                ev3_motor_stop(arm_motor, true);                                // モーターを停止
                return;                                                         // 関数を終了
            }
        }
        ev3_motor_set_power(arm_motor, cur_power);                      // アームモーターに出力を設定

        if(loop)                    // loopがtrueの場合
        {
            tslp_tsk(4 * 1000U);    /* 4msec周期起動 */
            cur_angle = ev3_motor_get_counts(arm_motor);    // 現在のモーター角度を更新
            cur_power = ev3_motor_get_power(arm_motor);     // 現在のモーター出力を更新
        }
    }
    while(loop);
}

//*****************************************************************************
// 関数名 : tale_up, tale_down
// 引数 : 無し
// 返り値 : 無し
// 概要 : 尻尾の開閉を行う
// 初期角度 4, 最大角度 3896
//*****************************************************************************
// テール開制開開御関数
void tale_open(uint8_t power, bool_t loop)
{
    int32_t cur_angle = ev3_motor_get_counts(tale_motor);   // 現在のモーター角度
    int8_t cur_power = ev3_motor_get_power(tale_motor);     // 現在のモーター出力

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
                ev3_motor_stop(tale_motor, true);                       // モーターを停止
                return;                                                 // 関数を終了
            }
        }
        ev3_motor_set_power(tale_motor, cur_power);             // テールモーターに出力を設定

        if(loop)                    // loopがtrueの場合
        {
            tslp_tsk(4 * 1000U);    /* 4msec周期起動 */
            cur_angle = ev3_motor_get_counts(tale_motor);    // 現在のモーター角度を更新
            cur_power = ev3_motor_get_power(tale_motor);     // 現在のモーター出力を更新
        }
    }
    while(loop);
}

// テール閉制御関数
void tale_close(uint8_t power, bool_t loop)
{
    int32_t cur_angle = ev3_motor_get_counts(tale_motor);   // 現在のモーター角度
    int8_t cur_power = ev3_motor_get_power(tale_motor);     // 現在のモーター出力

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
                ev3_motor_stop(tale_motor, true);                               // モーターを停止
                return;                                                         // 関数を終了
            }
        }
        ev3_motor_set_power(tale_motor, cur_power);                     // アームモーターに出力を設定

        if(loop)                    // loopがtrueの場合
        {
            tslp_tsk(4 * 1000U);    /* 4msec周期起動 */
            cur_angle = ev3_motor_get_counts(tale_motor);   // 現在のモーター角度を更新
            cur_power = ev3_motor_get_power(tale_motor);    // 現在のモーター出力を更新
        }
    }
    while(loop);
}


/* 黒ラインを検知したらその場で停止する関数 ******************************************/
// do ~ while(0) > 参考：https://qiita.com/ymko/items/ae8e056a270558f7fbaf
//
// loop : true (関数は停止が完了してからリターン)，false (関数は停止を待たずにリターン)
/*********************************************************************************/
void Run_setStop_Line(bool_t loop)
{
    do
    {
        ev3_color_sensor_get_rgb_raw(color_sensor, &rgb);
        if(rgb.r < 60 && rgb.g < 90 && rgb.b < 90)  // 黒ラインを検知した場合
        {
            motor_ctrl(0, 0);                           // 左右モーター停止
            return;                                     // 関数を終了
        }

        if(loop)                    // ループ処理の場合
            tslp_tsk(4 * 1000U);    /* 4msec周期起動 */
    }
    while(loop);
}

/* 指定した距離に到達するまで、指定出力で移動または旋回する関数 *********************************/
// power        : motor_ctrl関数のpower値(-100 ~ +100)
// turn         : motor_ctrl関数のturn値(-200 ~ +200)
// distance     : 移動する距離
/******************************************************************************************/
void Run_setDistance(int8_t power, int16_t turn, float distance)
{
    Distance_update();                                          // 距離を更新
    float ref_distance = Distance_getDistance();                // 処理開始時点での距離を取得

    if(power > 0 && distance > 0)                               // 前進の場合
    {
        while(1)                                                    // モーターが停止するまでループ
        {
            Distance_update();                                          // 距離を更新
            if(Distance_getDistance() >= (ref_distance + distance))     // 指定距離に到達した場合
            {
                motor_ctrl_alt(0, turn, 0.1);                               // モーターが停止するまで減速
                if(run_power == 0)                                          // モーターが完全に停止した場合
                    return;                                                     // 関数を終了
            }
            else                                                        // 指定距離に到達していない場合
            {
                motor_ctrl_alt(power, turn, 0.1);                           // 指定出力になるまで加速して走行
            }
            tslp_tsk(4 * 1000U); /* 4msec周期起動 */
        }
    }
    else if(power < 0 && distance < 0)                          // 後退の場合
    {
        while(1)                                                    // モーターが停止するまでループ
        {
            Distance_update();                                          // 距離を更新
            if(Distance_getDistance() <= (ref_distance + distance))     // 指定距離に到達した場合
            {
                motor_ctrl_alt(0, turn, 0.1);                               // モーターが停止するまで減速
                if(run_power == 0)                                          // モーターが完全に停止した場合
                    return;                                                     // 関数を終了
            }
            else                                                        // 指定距離に到達していない場合
            {
                motor_ctrl_alt(power, turn, 0.1);                               // 指定出力になるまで加速して走行
            }
            tslp_tsk(4 * 1000U); /* 4msec周期起動 */
        }
    }
    else                                                        // 正しい引数が得られなかった場合
    {
        printf("argument out of range @ Run_setDistance()\n");  // エラーメッセージを出して
        exit(1);                                                // 異常終了
    }
}

/* 指定した方位に到達するまで、指定出力で旋回または移動する関数 *********************************/
// power        : motor_ctrl関数のpower値(-100 ~ +100)
// turn         : motor_ctrl関数のturn値(-200 ~ +200)
// direction    : 旋回する方位
/******************************************************************************************/
void Run_setDirection(int8_t power, int16_t turn, float direction)
{
    Distance_update();                                              // 距離を更新
    Direction_update();                                             // 方位を更新
    float ref_direction = Direction_getDirection();                 // 処理開始時点での方位を取得
    
    // direction = direction * -1; // Rコース用の変換処理
    
    if(power != 0 && turn > 0 && direction > 0)                     // 右旋回の場合
    {
        while(1)                                                        // モーターが停止するまでループ
        {
            Distance_update();                                              // 距離を更新
            Direction_update();                                             // 方位を更新
            if(Direction_getDirection() >= (ref_direction + direction))     // 指定方位に到達した場合
            {
                motor_ctrl_alt(0, turn, 0.1);                                   // モーターが停止するまで減速
                if(run_power == 0)                                              // モーターが完全に停止した場合
                    return;                                                         // 関数を終了
            }
            else                                                            // 指定方位に到達していない場合
            {
                motor_ctrl_alt(power, turn, 0.1);                               // 指定出力になるまで加速して旋回
            }
            tslp_tsk(4 * 1000U); /* 4msec周期起動 */
        }
    }
    else if(power != 0 && turn < 0 && direction < 0)                // 左旋回の場合
    {
        while(1)                                                        // モーターが停止するまでループ
        {
            Distance_update();                                              // 距離を更新
            Direction_update();                                             // 方位を更新
            if(Direction_getDirection() <= (ref_direction + direction))     // 指定方位に到達した場合
            {
                motor_ctrl_alt(0, turn, 0.1);                                   // モーターが停止するまで減速
                if(run_power == 0)                                              // モーターが完全に停止した場合
                    return;                                                         // 関数を終了
            }
            else                                                            // 指定方位に到達していない場合
            {
                motor_ctrl_alt(power, turn, 0.1);                               // 指定出力になるまで加速して旋回
            }
            tslp_tsk(4 * 1000U); /* 4msec周期起動 */
        }
    }
    else                                                        // 正しい引数が得られなかった場合
    {
        printf("argument out of range @ Run_setDirection()\n"); // エラーメッセージを出して
        exit(1);                                                // 異常終了
    }
}

/* 指定した距離に障害物を検知するまで、指定出力で前進または旋回する関数 ***********************/
// power        : motor_ctrl関数のpower値(-100 ~ +100)
// turn         : motor_ctrl関数のturn値(-200 ~ +200)
// detection    : 障害物を検知する距離
// distance     : 障害物検知に加えて、指定の距離で停止する条件を追加する(0で無効)
/****************************************************************************************/
void Run_setDetection(int8_t power, int16_t turn, int16_t detection, float distance)
{
    Distance_update();                                                  // 距離を更新
    float ref_distance = Distance_getDistance();                        // 処理開始時点での距離を取得
    
    if(power > 0 && distance == 0)                                      // 距離の指定がない場合
    {
        while(1)                                                            // モーターが停止するまでループ
        {
            if(ev3_ultrasonic_sensor_get_distance(sonar_sensor) <= detection)   // 障害物を検知した場合
            {
                motor_ctrl_alt(0, turn, 0.1);                                       // モーターが停止するまで減速
                if(run_power == 0)                                                  // モーターが完全に停止した場合
                    return;                                                             // 関数を終了
            }
            else                                                                // 障害物を検知していない場合
            {
                motor_ctrl_alt(power, turn, 0.1);                                   // 指定出力になるまで加速して走行
            }
            tslp_tsk(4 * 1000U); /* 4msec周期起動 */
        }
    }
    else if(power > 0 && distance > 0)                                  // 距離の指定がある場合
    {        
        while(1)                                                            // モーターが停止するまでループ
        {
            Distance_update();                                                  // 距離を更新
            if(ev3_ultrasonic_sensor_get_distance(sonar_sensor) <= detection || Distance_getDistance() >= (ref_distance + distance))
            {                                                                   // 障害物を検知した場合、または指定距離に到達した場合
                motor_ctrl_alt(0, turn, 0.1);                                       // モーターが停止するまで減速
                if(run_power == 0)                                                  // モーターが完全に停止した場合
                    return;                                                             // 関数を終了
            }
            else                                                                // 障害物を未検知、かつ指定距離に到達していない場合
            {
                motor_ctrl_alt(power, turn, 0.1);                                   // 指定出力になるまで加速して走行
            }
            tslp_tsk(4 * 1000U); /* 4msec周期起動 */
        }
    }
    else                                                        // 正しい引数が得られなかった場合
    {
        printf("argument out of range @ Run_setDetection()\n"); // エラーメッセージを出して
        exit(1);                                                // 異常終了
    }
}


/* PID初期化関数 *********************************************************/
// （　＾ω＾）・・・
/************************************************************************/
void Run_PID_init()
{
    diff[0] = 0;
    integral = 0.0;
}

/* PID制御関数(定数) * (センサー入力値 - 目標値) **********************************************************/
// 参考：https://monoist.atmarkit.co.jp/mn/articles/1007/26/news083.html
//
// sensor_val   : センサーの現在値
// taget_val    : センサーの目標値
//
// 返り値       : motor_ctrl関数のturn値(-200 ~ +200)
/*******************************************************************************************************/
int16_t Run_getTurn_sensorPID(uint16_t sensor_val, uint16_t target_val)    // センサー値, センサーの目標値
{
    float p, i, d;

    diff[0] = diff[1];
    diff[1] = sensor_val - target_val;  // 偏差を取得
    integral += (diff[1] + diff[0]) / 2.0 * DELTA_T;

    p = KP * diff[1];
    i = KI * integral;
    d = KD * (diff[1] - diff[0]) / DELTA_T;

    return roundf(math_limit(p + i + d, -200.0, 200.0));    // 最大・最小値を制限し、四捨五入した値を返す
}


/* 目標の出力値に到達するまで、指定量の出力値の増減を行い、その結果を返す関数 *********************************************************************/
// 徐々に加速、減速を行えるようにするための関数。線形で示すと、通常の加減速は _|￣|_ であり、この関数で実現したい加減速は _／￣＼_　のような形。
//
// current_power : 現在の出力値
// target_power  : 目標の出力値
// change_rate   : 増減の変化量 *例として0.2とした場合、4ms(1周期)で出力値が0.2ずつ変化し、20ms経過すると出力値が 1 変化することになる
//
// 返り値        : 指定量の加減速を行ったモーターの出力値(小数点以下の値はモーターが対応していないため切り捨て)
/*******************************************************************************************************************************************/
int8_t Run_getPower_change(int8_t current_power, int8_t target_power, float change_rate)
{
    static float power = 0;                 // 出力値を保持する変数

    if(current_power < target_power)        // 現在値 < 目標値 の時
    {
        if(floorf(power) != current_power)      // 現在値が指定した変化量を超えて増減した場合の対策
            power = current_power;                  // 出力値を更新

        power = power + change_rate;            // 変化量の分だけ増加
        return floorf(power);                   // 小数点以下を切り捨てて値を返す
    }
    else if(current_power > target_power)       // 現在値 > 目標値 の時
    {
        if(ceilf(power) != current_power)       // 現在値が指定した変化量を超えて増減した場合の対策
            power = current_power;                  // 出力値を更新

        power = power - change_rate;            // 変化量の分だけ減少
        return ceilf(power);                    // 小数点以下を切り上げて値を返す
    }
    else                                    // 現在値が目標値に到達している場合
        return current_power;                   // 現在値をそのまま返す
}

/* 目標の出力値に到達するまで、指定量の出力値の増減を行い、その結果を返す関数 *********************************************************************/
int8_t Run_getTurn_change(int8_t current_turn, int8_t target_turn, float change_rate)
{
    static float turn = 0;                 // 出力値を保持する変数

    if(current_turn < target_turn)        // 現在値 < 目標値 の時
    {
        if(floorf(turn) != current_turn)      // 現在値が指定した変化量を超えて増減した場合の対策
            turn = current_turn;                  // 出力値を更新

        turn = turn + change_rate;            // 変化量の分だけ増加
        return floorf(turn);                   // 小数点以下を切り捨てて値を返す
    }
    else if(current_turn > target_turn)       // 現在値 > 目標値 の時
    {
        if(ceilf(turn) != current_turn)       // 現在値が指定した変化量を超えて増減した場合の対策
            turn = current_turn;                  // 出力値を更新

        turn = turn - change_rate;            // 変化量の分だけ減少
        return ceilf(turn);                    // 小数点以下を切り上げて値を返す
    }
    else                                    // 現在値が目標値に到達している場合
        return current_turn;                   // 現在値をそのまま返す
}

/* モーター制御関数alt *********************************************************************************************************************/
// 加減速機能付きのmotor_ctrl関数
// 使用方法に制限あり
// 悪い例：motor_ctrl_alt(50, 0, 1);   ←のように１度の処理周期内に連続で記述すると動作に問題が生じる為,if文などで実行タイミングを別々にする必要がある
//        motor_ctrl_alt(0, 0, 1);
/******************************************************************************************************************************************/
void motor_ctrl_alt(int8_t power, int16_t turn, float change_rate)
{
    // turn = turn * -1;   // Rコース用の変換処理

    power = Run_getPower_change(run_power, power, change_rate); // 出力調整
    run_power = power;  // 計測用の変数を更新
    run_turn = turn;    // 計測用の変数を更新

    if(power < -100 || power > 100 || turn < -200 || turn > 200)    // 引数が許容範囲に収まっていない場合
    {
        power   = math_limit(power, -100, 100);                         // math_limit関数を利用して
        turn    = math_limit(turn, -200, 200);                          // 値を許容範囲に収めてから走行処理を行う
    }
    
    if(power != 0 && turn == 0)                                     // 前後進
    {
        ev3_motor_set_power(left_motor, power);
        ev3_motor_set_power(right_motor, power);
    }
    else if(turn > 0)                                               // 右旋回
    {
        ev3_motor_set_power(left_motor, power);
        ev3_motor_set_power(right_motor, power - (turn * power / 100)); // turnをpowerの比率に合わせる
    }
    else if(turn < 0)                                               // 左旋回
    {
        ev3_motor_set_power(left_motor, power + (turn * power / 100));  // turnをpowerの比率に合わせる
        ev3_motor_set_power(right_motor, power);
    }
    else                                                            // 引数(0, 0)で左右モーター停止
    {
        ev3_motor_stop(left_motor, true);
        ev3_motor_stop(right_motor, true);
    }
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
        if(ev3_ultrasonic_sensor_get_distance(sonar_sensor) <= 25)
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



/* センサのサンプリング周期計測関数***********************************************/
void sensor_update(uint16_t now_value)
{
    static SYSTIM pre_time = 0, now_time = 0;
    static uint32_t pre_value = 0;

    if(pre_value != now_value)
    {
        pre_value = now_value;
        get_tim(&now_time);
        sampling_time = now_time - pre_time;
        get_tim(&pre_time);
    }
}

/* 速度計測関数***********************************************/
void speed_update(void)
{
    static uint32_t pre_time = 0, now_time = 0;
    float pre_distance = 0.0;

    now_time = Run_getTime();

    if((now_time - pre_time) >= 20)
    {
        speed = (Distance_getDistance() - pre_distance);
        pre_time = now_time;
        pre_distance = Distance_getDistance();
    }
}