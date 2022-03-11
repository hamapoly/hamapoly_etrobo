#include "app_Slalom.h"

/* マクロ定義 */

/* グローバル変数 */
static const sensor_port_t
    sonar_sensor    = EV3_PORT_3,
    gyro_sensor     = EV3_PORT_4;

/* メイン関数 */
void section_Slalom()
{
    /* ローカル変数 ******************************************************************************************/
    float temp = 0.0;       // 距離、方位の一時保存用

    int8_t flag = 0;        // 便利なflag
    int8_t edge = 0;        // 左コース走行時、1 でラインの左側をトレース、-1 で右側をトレース

    int8_t power = 0;       // モーターの出力値を格納する変数(-100 ~ +100)
    int16_t turn = 0;       // モーターによる旋回量を格納する変数(-200 ~ +200)

    /* 列挙 */
    enum {
        START,          // 段差の手前で段差を上る準備
        UP_STAIRS,      // 段差を上る
        MOVE_1,         // 2つ目のペットボトル手前まで移動
        MOVE_2,         // 3つ目のペットボトル手前まで移動
        BRANCH,         // 4つ目のペットボトル手前まで移動して配置パターンを判断する
        PATTERN_A,      // 配置パターンAの場合の移動処理
        PATTERN_B,      // 配置パターンBの場合の移動処理
        LINETRACE,      // ラインに復帰する
        END             // 次のタスクへ移行
    } r_state = START;

    /* 初期化処理 */
    ev3_gyro_sensor_reset(gyro_sensor);     // ジャイロセンサーの初期化
    Run_init();         // 走行データを初期化
    Ctrl_initPID();     // PIDの値を初期化

    temp = Run_getDistance();  // 指定距離ライントレースのため、処理開始時点の距離を取り置き

    /**
    * Main loop ****************************************************************************************************************************************
    */
    while(1)
    {
        if(flag == 1)   // 終了フラグを確認
            return;     // 関数終了

        switch(r_state)
        {
            case START: // 壁にアームを押し付けて方位を調整、後退してアームを上げる **************

                if(Run_getDistance() < temp + 50)     // 指定距離に到達していない場合
                {
                    turn = Ctrl_getTurn_PID(Run_getRGB_R(), 60);    // PID制御で旋回量を算出
                    Ctrl_motor_steer(15, turn);                       // 指定出力とPIDでライントレース走行
                }
                else                                        // 指定距離に到達した場合
                {
                    Ctrl_motor_steer(0, 0);                           // モーター停止
                    tslp_tsk(200 * 1000U);                      // 待機

                    Ctrl_motor_steer(-10, 0);                         // 指定出力で後退
                    tslp_tsk(400 * 1000U);                      // 待機

                    Ctrl_motor_steer(0, 0);                           // モーター停止
                    Ctrl_arm_up(100, true);                           // アームを上げる
                    r_state = UP_STAIRS;                        // 状態を遷移する
                }
                break;

            case UP_STAIRS: // 段差を上る ***************************************
                if(-3 < Run_getAngle() && Run_getAngle() < 3)
                {                                           // 傾きが検知されない場合
                    Ctrl_motor_steer(25, 0);                          // 指定出力で前進
                }
                else                                        // 傾きを検知した場合
                {
                    Ctrl_motor_steer(30, 0);                          // 指定出力で前進
                    tslp_tsk(200 * 1000U);                      // 待機
                    if(-3 > Run_getAngle() || Run_getAngle() > 3)
                    {                                           // 傾きを検出した場合
                        Ctrl_motor_steer(33, 0);                          // 指定出力で前進
                    }
                    Ctrl_arm_down(30, true);                         // アームをおろす

                    r_state = MOVE_1;                           // 状態を遷移する
                    temp = Run_getDistance();              // 指定距離ライントレースのため、処理開始時点の距離を取り置き
                }
                break;
                
            case MOVE_1: // 2つ目のペットボトル手前まで移動 ************************************
                if( Run_getDistance() < temp + 180)        // 指定距離内に障害物を検知するか、指定距離を走りきるまで
                {
                    turn = Ctrl_getTurn_PID(Run_getRGB_R(), 51);        // PID制御で旋回量を算出
                    Ctrl_motor_steer(13, turn);                           // ライントレース
                }
                else                                            // 指定距離内に障害物を検知したか、指定距離を走りきった場合
                {
                    if(Run_getPower() != 0)                         // モーターが停止していない場合
                    {
                        Ctrl_motor_steer_alt(0, 0, 0.1);                      // 減速してモーター停止
                    }
                    else                                            // モーターが停止した場合
                    {
                        Ctrl_runDirection(10, 200, 40);                 // 右旋回

                        Slalom_run(10, -65, 165);                       // 左旋回

                        Ctrl_runDetection(10, 0, 6, 0);                 // 障害物を検知するまで前進

                        r_state = MOVE_2;                               // 状態を遷移する
                    }
                }
                break;

            case MOVE_2: // 3つ目のペットボトル手前まで移動 ************************************

                Slalom_run(20, -85, 100);                       // 左旋回
                
                Slalom_run(20, 80, 160);                        // 右旋回

                Ctrl_runDetection(10, 0, 9, 0);                 // 障害物を検知するまで前進

                Slalom_run(20, -65, 90);                        // 左旋回

                Slalom_run(20, 25, 30);                         // 右旋回

                r_state = BRANCH;                               // 状態を遷移する
                break;

            case BRANCH: // 4つ目のペットボトル手前まで移動して配置パターンを判断する ************

                if(sampling_sonic())                            // 走行体正面の障害物の有無を検知
                    r_state = PATTERN_A;                            // 正面にペットボトルがあればPATTERN_Aへ分岐
                else                                            
                    r_state = PATTERN_B;                            // 正面にペットボトルがなければPATTERN_Bへ分岐
                break;

            case PATTERN_A: // **********************************************************
                Ctrl_runDetection(10, 10, 8, 100);              // 障害物を検知する、または指定距離走るまで前進

                Slalom_run(20, 80, 100);                        //右旋回

                Slalom_run(20, 0, 120);                         //前進

                Slalom_run(20, 42, 60);                         //右旋回

                Ctrl_arm_up(60, true);                               // アームを上げる

                ev3_gyro_sensor_reset(gyro_sensor);             // ジャイロセンサーの初期化
                while(-3.5 < Run_getAngle() && Run_getAngle() < 3.5)
                {                                               // 傾きを検知するまでループ
                    Ctrl_motor_steer(20, 30);                             // 右曲がりに前進
                    tslp_tsk(4 * 1000U);                            /* 4msec周期起動 */
                }
                tslp_tsk(500 * 1000U);                          // 待機

                Ctrl_arm_down(40, true);                             // アームを下げる

                Ctrl_motor_steer(20, 50);                             // 右曲がりに前進
                Ctrl_runStop_Line(true);                        // ラインを検知したら停止
                Slalom_run(20, -100, 50);                       // 左旋回

                edge = 1;                                       // ラインの左側をトレースするように設定
                r_state = LINETRACE;                            // 状態を遷移する
                break;
                
            case PATTERN_B: // **************************************************************

                Slalom_run(20, 0, 70);                          // 前進

                Slalom_run(20, 85, 230);                        // 右旋回

                Slalom_run(20, 0, 80);                          // 前進

                Ctrl_arm_up(60, true);                               // アームを上げる

                Slalom_run(20, -80, 90);                        // 左旋回

                ev3_gyro_sensor_reset(gyro_sensor);             // ジャイロセンサーの初期化
                while(-3.5 < Run_getAngle() && Run_getAngle() < 3.5)
                {                                               // 傾きを検知するまでループ
                    Ctrl_motor_steer(20, -50);                            // 左曲がりに前進
                    tslp_tsk(4 * 1000U);                            /* 4msec周期起動 */
                }
                tslp_tsk(500 * 1000U);                          // 待機

                Ctrl_arm_down(40, true);                             // アームを下げる

                Ctrl_motor_steer(20, -60);                            // 左曲がりに前進
                Ctrl_runStop_Line(true);                        // ラインを検知したら停止
                Slalom_run(20, 110, 80);                        // 右旋回

                edge = 1;                                       // ラインの左側をトレースするように設定
                r_state = LINETRACE;                            // 状態を遷移する
                break;

            case LINETRACE: // **********************************************************
                turn = Ctrl_getTurn_PID(Run_getRGB_R(), 60);        // PID制御で旋回量を算出(Line.cを参照)
                Ctrl_motor_steer(10, turn * edge);                    // ライントレース
                
                if(flag == 2 && Run_getRGB_R() < 65 && Run_getRGB_G() < 75 && Run_getRGB_B() < 95) 
                {                                               // 黒ラインを検知
                    Ctrl_motor_steer(15, 0);                              // 前進しながら
                    r_state = END;                                  // 最後の処理に移る
                }

                if(Run_getRGB_R() < 75 && Run_getRGB_G() < 95 && Run_getRGB_B() > 120)     // 青ラインを検知
                {
                    flag = 1;                                       // フラグを立てて上のif条件を解除する
                }

                break;

            case END: // **************************************************************
                if(ev3_ultrasonic_sensor_get_distance(sonar_sensor) < 5)
                {                                           // ガレージの壁を検知した場合
                    Ctrl_motor_steer(0, 0);                       // モーター停止
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

//*****************************************************************************
// 関数名 : Slalpm_run
// power        : Ctrl_motor_steer関数のpower値(-100 ~ +100)
// turn         : Ctrl_motor_steer関数のturn値(-200 ~ +200)
// distance     : 移動する距離
// 概要 : 加減速なしで指定距離走行を行う(スラローム区間のスムーズな走行を実現するため)
//*****************************************************************************
void Slalom_run(int8_t power, int16_t turn, float distance)
{
    float ref_distance = Run_getDistance();                // 処理開始時点での距離を取得
    if(power > 0 && distance > 0)                               // 前進の場合
    {
        while(1)                                                    // モーターが停止するまでループ
        {
            Ctrl_motor_steer(power, turn);
            if(Run_getDistance() >= (ref_distance + distance))     // 指定距離に到達した場合
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
            Ctrl_motor_steer(power, turn);
            if(Run_getDistance() <= (ref_distance + distance))     // 指定距離に到達した場合
            {
                return;                                                     // 関数を終了
            }
            tslp_tsk(4 * 1000U); /* 4msec周期起動 */
        }
    }
    else                                                        // 正しい引数が得られなかった場合
    {
        printf("argument out of range @ Slalom_run()\n");       // エラーメッセージを出して
        exit(1);                                                // 異常終了
    }
}