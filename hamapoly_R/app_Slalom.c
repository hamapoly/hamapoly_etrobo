#include "app_Slalom.h"

/* マクロ定義 */

/* グローバル変数 */
static const sensor_port_t
    color_sensor    = EV3_PORT_2,
    sonar_sensor    = EV3_PORT_3,
    gyro_sensor     = EV3_PORT_4;

static const motor_port_t
    left_motor      = EV3_PORT_C,
    right_motor     = EV3_PORT_B;

/* 構造体 */
typedef enum {
    START,          // 段差の手前で段差を上る準備
    UP_STAIRS,      // 段差を上る
    MOVE_1,         // 2つ目のペットボトル手前まで移動
    MOVE_2,         // 3つ目のペットボトル手前まで移動
    BRANCH,         // 4つ目のペットボトル手前まで移動して配置パターンを判断する
    PATTERN_A,      // 配置パターンAの場合の移動処理
    PATTERN_B,      // 配置パターンBの場合の移動処理
    LINETRACE,      // ラインに復帰する
    END             // 次のタスクへ移行
    } RUN_STATE;

static RUN_STATE r_state = START;

/* メイン関数 */
void Slalom_task()
{
    /* ローカル変数 ******************************************************************************************/
    rgb_raw_t rgb;

    float temp = 0.0;       // 距離、方位の一時保存用

    float distance = 0.0;   // 走行距離

    int8_t flag = 0;        // 便利なflag
    int8_t edge = 0;        // 1 でラインの左側をトレース、-1 で右側をトレース

    int8_t power = 0;       // モーターの出力値を格納する変数(-100 ~ +100)
    int16_t turn = 0;       // モーターによる旋回量を格納する変数(-200 ~ +200)

    /* 初期化処理 ********************************************************************************************/
    // 別ソースコード内の計測用static変数を初期化する(初期化を行わないことで、以前の区間から値を引き継ぐことができる)
    Distance_init();    // 距離を初期化
    Direction_init();   // 方位を初期化

    // Run_init();         // 走行時間を初期化

    /**
    * Main loop ****************************************************************************************************************************************
    */

    temp = Distance_getDistance();  // 指定距離ライントレースのため、処理開始時点の距離を取り置き

    while(1)
    {
        /* 値の更新 **********************************************************************************************/
        Distance_update();                      // 走行距離を更新
        distance = Distance_getDistance();      // 走行距離を取得
        
        ev3_color_sensor_get_rgb_raw(color_sensor, &rgb);   // RGBを取得
        /********************************************************************************************************/

        // ここに処理を記述
        if(flag == 1)   // 終了フラグを確認
            return;     // 関数終了

        switch(r_state)
        {
            case START: // 壁にアームを押し付けて方位を調整、後退してアームを上げる **************

                if(Distance_getDistance() < temp + 180)              // 指定距離に到達していない場合
                {
                    turn = Run_getTurn_sensorPID(rgb.r, 60);    // PID制御で旋回量を算出
                    motor_ctrl(15, turn);                       // 指定出力とPIDでライントレース走行
                }
                else                            // 指定距離に到達した場合
                {
                    motor_ctrl(0, 0);           // モーター停止
                    tslp_tsk(200 * 1000U);      // 待機

                    motor_ctrl(-10, 0);         // 指定出力で後退
                    tslp_tsk(400 * 1000U);      // 待機

                    motor_ctrl(0, 0);           // モーター停止
                    arm_up(30, true);           // アームを上げる
                    r_state = UP_STAIRS;
                }
                break;

            case UP_STAIRS: // 段差を上る ***************************************
                if(-3 < ev3_gyro_sensor_get_angle(gyro_sensor) && ev3_gyro_sensor_get_angle(gyro_sensor) < 3)
                {                           // 傾きが検知されない場合
                    motor_ctrl(20, 0);          // 指定出力で前進
                }
                else                        // 傾きを検知した場合
                {
                    motor_ctrl(30, 0);      // 指定出力で前進
                    tslp_tsk(200 * 1000U);  // 待機
                    if(-3 > ev3_gyro_sensor_get_angle(gyro_sensor) || ev3_gyro_sensor_get_angle(gyro_sensor) > 3)
                    {                           // 傾きを検出した場合
                        motor_ctrl(33, 0);          // 指定出力で前進
                    }
                    arm_down(30, true);     // アームをおろす

                    r_state = MOVE_1;
                    temp = Distance_getDistance();  // 指定距離ライントレースのため、処理開始時点の距離を取り置き
                }
                break;
                
            case MOVE_1: // 2つ目のペットボトル手前まで移動 ************************************
                if( Distance_getDistance() < temp + 180)    // 指定距離内に障害物を検知するか、指定距離を走りきるまで
                {
                    turn = Run_getTurn_sensorPID(rgb.r, 51);      // PID制御で旋回量を算出
                    motor_ctrl(13, turn);                         // ライントレース
                }
                else
                {
                    if(Run_getPower() != 0)                   // モーターが停止していない場合
                    {
                        motor_ctrl_alt(0, 0, 0.1);              // 減速してモーター停止
                    }
                    else                                    // モーターが停止した場合
                    {
                        Run_setDirection(10, 200, 40);           // 右旋回

                        Slalom_run(10, -65, 165);

                        Run_setDetection(10, 0, 6, 0);          // 障害物を検知するまで前進

                        r_state = MOVE_2;
                    }
                }
                break;

            case MOVE_2: // 3つ目のペットボトル手前まで移動 ************************************

                Slalom_run(20, -85, 100);                // 左旋回
                
                Slalom_run(20, 80, 160);                // 右旋回

                Run_setDetection(10, 0, 9, 0);          // 障害物を検知するまで前進

                Slalom_run(20, -65, 90);

                Slalom_run(20, 25, 30);

                r_state = BRANCH;
                break;

            case BRANCH: // 4つ目のペットボトル手前まで移動して配置パターンを判断する ************

                if(sampling_sonic())                      // 走行体正面の障害物の有無を検知
                    r_state = PATTERN_A;                                    // 正面にペットボトルがあればPATTERN_Aへ分岐
                else
                    r_state = PATTERN_B;                                    // 正面にペットボトルがなければPATTERN_Bへ分岐
                break;

            case PATTERN_A: // **********************************************************
                Run_setDetection(10, 10, 8, 100);        // 障害物を検知する、または指定距離走るまで前進

                Slalom_run(20, 80, 100);

                Slalom_run(20, 0, 120);

                Slalom_run(20, 42, 60);

                arm_up(60, true);                       // アームを上げる

                ev3_gyro_sensor_reset(gyro_sensor);     // ジャイロセンサーの初期化
                while(-3.5 < ev3_gyro_sensor_get_angle(gyro_sensor) && ev3_gyro_sensor_get_angle(gyro_sensor) < 3.5)
                {                                       // 傾きを検知するまでループ
                    motor_ctrl(20, 30);                     // 右曲がりに前進
                    tslp_tsk(4 * 1000U);                    /* 4msec周期起動 */
                }
                tslp_tsk(500 * 1000U);                  // 待機

                arm_down(40, true);                     // アームを下げる

                motor_ctrl(20, 50);                     // 右曲がりに前進
                Run_setStop_Line(true);                 // ラインを検知したら停止
                Slalom_run(20, -100, 50);        // 左旋回

                edge = 1;                               // ラインの右側をトレースするように設定
                r_state = LINETRACE;
                break;
                
            case PATTERN_B: // **************************************************************

                Slalom_run(20, 0, 70);

                Slalom_run(20, 85, 230);

                Slalom_run(20, 0, 80);

                arm_up(60, true);                       // アームを上げる

                Slalom_run(20, -80, 90);

                ev3_gyro_sensor_reset(gyro_sensor);     // ジャイロセンサーの初期化
                while(-3.5 < ev3_gyro_sensor_get_angle(gyro_sensor) && ev3_gyro_sensor_get_angle(gyro_sensor) < 3.5)
                {                                       // 傾きを検知するまでループ
                    motor_ctrl(20, -50);                    // 左曲がりに前進
                    tslp_tsk(4 * 1000U);                    /* 4msec周期起動 */
                }
                tslp_tsk(500 * 1000U);                  // 待機

                arm_down(40, true);                     // アームを下げる

                motor_ctrl(20, -60);                    // 左曲がりに前進
                Run_setStop_Line(true);                 // ラインを検知したら停止
                Slalom_run(20, 110, 80);          // 右旋回

                edge = 1;                              // ラインの左側をトレースするように設定
                r_state = LINETRACE;
                break;

            case LINETRACE: // **********************************************************
                turn = Run_getTurn_sensorPID(rgb.r, 60);    // PID制御で旋回量を算出(Line.cを参照)
                motor_ctrl(10, turn * edge);                // ライントレース
                
                if(flag == 2 && rgb.r < 65 && rgb.g < 75 && rgb.b < 95) // 黒ラインを検知
                {
                    motor_ctrl(15, 0);                          // 前進しながら
                    r_state = END;                              // 最後の処理に移る
                }

                if(rgb.r < 75 && rgb.g < 95 && rgb.b > 120)     // 青ラインを検知
                {
                    flag = 1;                                   // フラグを立てて上のif条件を解除する
                }

                break;

            case END: // **************************************************************
                if(ev3_ultrasonic_sensor_get_distance(sonar_sensor) < 5)
                {
                    motor_ctrl(0, 0);                       // ガレージの壁を検知して停車
                    flag = 1;                               // 終了フラグを立てる
                }
                break;

            default:
                break;
        }
        tslp_tsk(4 * 1000U); /* 4msec周期起動 */
    }
    /**
    * Main loop END ************************************************************************************************************************************
    */
}


void Slalom_run(int8_t power, int16_t turn, float distance)
{
    Distance_update();                                          // 距離を更新
    float ref_distance = Distance_getDistance();                // 処理開始時点での距離を取得
    if(power > 0 && distance > 0)                               // 前進の場合
    {
        while(1)                                                    // モーターが停止するまでループ
        {
            motor_ctrl(power, turn);
            Distance_update();                                          // 距離を更新
            if(Distance_getDistance() >= (ref_distance + distance))     // 指定距離に到達した場合
            {
                return;                                                     // 関数を終了
            }
            tslp_tsk(4 * 1000U); /* 4msec周期起動 */
        }
    }
    else if(power < 0 && distance < 0)                          // 後退の場合
    {
        while(1)                                                    // モーターが停止するまでループ
        {
            motor_ctrl(power, turn);
            Distance_update();                                          // 距離を更新
            if(Distance_getDistance() <= (ref_distance + distance))     // 指定距離に到達した場合
            {
                return;                                                     // 関数を終了
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