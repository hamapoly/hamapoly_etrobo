#include "Run.h"

/* マクロ定義 */
#define PI 3.14159265358    // 円周率
#define TREAD 145.0         //車体トレッド幅(約140.0mm *ETロボコンシミュレータの取扱説明書参照) -> (150.0mm *2020年ADVクラスのDENSOチームのモデル図に記載)
#define TIRE_DIAMETER 100.0 //タイヤ直径(約90mm *ETロボコンシミュレータの取扱説明書参照) -> (90.0mm *2020年ADVクラスのDENSOチームのモデル図に記載)

/* グローバル宣言 */
typedef struct running_data{    // 走行データ用の構造体
    rgb_raw_t   rgb;
    int8_t      power_L;
    int8_t      power_R;
    int8_t      power;
    int16_t     turn;
    int16_t     angle;
    float       speed; 
    float       distance;
    float       direction;
    SYSTIM      cycle_time;
    uint32_t    time;
}run_data_t;

static run_data_t run;

/* 関数 */

// 走行データの初期化(累積する値のみ)
void Run_init(void)
{
    run.time = 0;
    Run_initDistance();
    Run_initDirection();
}

// データ更新
void Run_update(void)
{
    if(run.time < 480000) run.time++;                       // 走行時間を加算(5ms周期の場合、最大240秒まで) *ログに記録するときに周期を掛ける
    ev3_color_sensor_get_rgb_raw(EV3_PORT_2, &run.rgb);   // RGB値を更新
    run.angle = ev3_gyro_sensor_get_angle(EV3_PORT_4);     // 位置角(傾き)を更新
    Run_updateMotor();          // モーター出力値を更新
    Run_updateDistance();       // 走行距離を更新
    Run_updateDirection();      // 走行方位を更新
    Run_updateSpeed();          // 走行速度を更新
    // Run_updateCycleTime();   // 更新周期を更新    
}

// 走行データ取得用関数群
//---------------------------------------------------------------------------------------------------------------------------------
uint16_t    Run_getRGB_R(void)      { return run.rgb.r; }       // カラーセンサーのR値を取得
uint16_t    Run_getRGB_G(void)      { return run.rgb.g; }       // カラーセンサーのG値を取得
uint16_t    Run_getRGB_B(void)      { return run.rgb.b; }       // カラーセンサーのB値を取得
uint32_t    Run_getTime(void)       { return run.time; }        // 走行時間を取得(5ms単位) <- 周期ハンドラによって5msごとに更新されるため
int8_t      Run_getPower(void)      { return run.power; }       // モーター出力を取得
int8_t      Run_getPower_L(void)    { return run.power_L; }     // Lモーター出力を取得
int8_t      Run_getPower_R(void)    { return run.power_R; }     // Rモーター出力を取得
int16_t     Run_getTurn(void)       { return run.turn; }        // 旋回値を取得
int16_t     Run_getAngle(void)      { return run.angle; }       // 位置角(傾き)を取得
float       Run_getDistance(void)   { return run.distance; }    // 走行距離を取得
float       Run_getDirection(void)  { return run.direction; }   // 走行方位を取得(右回転が正転)
float       Run_getSpeed(void)      { return run.speed; }       // 走行速度を取得(100ms毎の速度)
// SYSTIM      Run_getCycleTime(void)  { return run.cycle_time }   // 指定された値の更新周期を取得

// 計測値更新用の関数群
//---------------------------------------------------------------------------------------------------------------------------------
/* モーター出力計測関数 */
void Run_updateMotor(void)
{
    run.power_L = ev3_motor_get_power(EV3_PORT_C);      // 右モーターの出力値を更新
    run.power_R = ev3_motor_get_power(EV3_PORT_B);     // 左モーターの出力値を更新
    if(run.power_L == run.power_R)                      // 直進時のモーター出力と旋回量を更新
    {
        run.power = run.power_L;
        run.turn  = 0;
    }
    else if(run.power_L > run.power_R)                  // 右旋回時のモーター出力と旋回量を更新
    {
        run.power = run.power_L;
        run.turn  = (int16_t)((run.power_L - run.power_R) * 100) / run.power_L;
    }
    else if(run.power_L < run.power_R)                  // 左旋回時のモーター出力と旋回量を更新
    {
        run.power = run.power_R;
        run.turn  = (int16_t)((run.power_R - run.power_L) * 100) / run.power_R * -1;
    }
}

/* 速度計測関数(100ms毎の速度) */
void Run_updateSpeed(void)
{
    static uint32_t pre_time = 0;
    static float pre_distance = 0.0;
    
    if((Run_getTime() - pre_time) >= 20)
    {
        run.speed = (Run_getDistance() - (float)pre_distance);
        pre_time = Run_getTime();
        pre_distance = Run_getDistance();
    }
}

/* 指定された値の更新周期計測関数 */
void Run_updateCycleTime(uint16_t cur_value)
{
    static SYSTIM pre_time = 0, cur_time = 0;
    static uint32_t pre_value = 0;

    if(pre_value != cur_value)
    {
        pre_value = cur_value;
        get_tim(&cur_time);
        run.cycle_time = cur_time - pre_time;
        get_tim(&pre_time);
    }
}

// 距離計測用の関数群(引用：https://qiita.com/TetsuroAkagawa/items/ba6190f08d26df7cc8ad)
//---------------------------------------------------------------------------------------------------------------------------------
/* 距離計測用グローバル変数 */
static float distance4msL = 0.0; //左タイヤの4ms間の距離
static float distance4msR = 0.0; //右タイヤの4ms間の距離
static float pre_angleL, pre_angleR; // 左右モータ回転角度の過去値

static float angle4msL = 0.0; //左タイヤの4ms間の角度
static float angle4msR = 0.0; //右タイヤの4ms間の角度

/* 初期化関数 */
void Run_initDistance() {
    //各変数の値の初期化
    run.distance = 0.0;
    distance4msR = 0.0;
    distance4msL = 0.0;
    //モータ角度の過去値に現在値を代入
    pre_angleL = ev3_motor_get_counts(EV3_PORT_C);
    pre_angleR = ev3_motor_get_counts(EV3_PORT_B);
}

/* 距離更新（4ms間の移動距離を毎回加算している） */
void Run_updateDistance(){
    float cur_angleL = ev3_motor_get_counts(EV3_PORT_C);    //左モータ回転角度の現在値
    float cur_angleR = ev3_motor_get_counts(EV3_PORT_B);   //右モータ回転角度の現在値
    float distance4ms = 0.0;        //4msの距離

    // 4ms間の走行距離 = ((円周率 * タイヤの直径) / 360) * (モータ角度過去値　- モータ角度現在値)
    distance4msL = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleL - pre_angleL);  // 4ms間の左モータ距離
    distance4msR = ((PI * TIRE_DIAMETER) / 360.0) * (cur_angleR - pre_angleR);  // 4ms間の右モータ距離
    distance4ms = (distance4msL + distance4msR) / 2.0; //左右タイヤの走行距離を足して割る
    run.distance += distance4ms;

    angle4msL = cur_angleL - pre_angleL;
    angle4msR = cur_angleR - pre_angleR;

    //モータの回転角度の過去値を更新
    pre_angleL = cur_angleL;
    pre_angleR = cur_angleR;
}

/* 右タイヤの4ms間の距離を取得 */
float Run_getDistance4msRight(){
    return distance4msR;
}

/* 左タイヤの4ms間の距離を取得 */
float Run_getDistance4msLeft(){
    return distance4msL;
}

/* 右タイヤの4ms間の角度を取得 */
int Run_getAngle4msLeft(){
    return roundf(angle4msL);
}

/* 左タイヤの4ms間の角度を取得 */
int Run_getAngle4msRight(){ 
    return roundf(angle4msR);
}

// 方位計測用の関数群(引用：https://qiita.com/TetsuroAkagawa/items/4e7de30523d9c7ec6241)
//---------------------------------------------------------------------------------------------------------------------------------
 /* 初期化 */
void Run_initDirection(){
    run.direction = 0.0;
}

/* 方位を更新 */
void Run_updateDirection(){
    //(360 / (2 * 円周率 * 車体トレッド幅)) * (右進行距離 - 左進行距離)
    run.direction += (360.0 / (2.0 * PI * TREAD)) * (Run_getDistance4msLeft() - Run_getDistance4msRight());
}
