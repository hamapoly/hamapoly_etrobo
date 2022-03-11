#include "app_Linetrace.h"

/* マクロ定義 */
#define MOTOR_POWER     50  // モーターの出力値(-100 ~ +100)
#define PID_TARGET_VAL  74  // PID制御におけるセンサRun_getRGB_R()の目標値 *参考 : https://qiita.com/pulmaster2/items/fba5899a24912517d0c5

/* 関数 */
void section_Linetrace()
{
    /* ローカル変数 */

    float temp = 0.0;   // 距離、方位の一時保存用

    int8_t flag = 0;
    int8_t flag_line[] = {0, 0, 0, 0};
    int8_t power = MOTOR_POWER;

    int16_t turn = 0;

    char message[30];

    /* 列挙 */
    enum {
        START,
        MOVE,
        CURVE_1,
        CURVE_2,
        CURVE_Z,
        CURVE_4,
        LINETRACE,
        END,
        GOAL_LINE
    } line_state = LINETRACE;

    /* 初期化処理 */
    Run_init();         // 走行時間を初期化
    Ctrl_initPID();     // PIDの値を初期化

    /**
    * Main loop ****************************************************************************************************************************************
    */
    while(1)
    {
        if(flag == 1)   // 終了フラグを確認
            return;     // 関数終了

        // linetrace test----
        // while(Run_getDistance() < 3000)
        // {
        //     power = Ctrl_getPower_Change(70, 0.5);
        //     turn = Ctrl_getTurn_PID(Run_getRGB_R(), 65);
        //     Ctrl_motor_steer(power, turn);
        //     tslp_tsk(4 * 1000U);
        // }
        // end-----

        switch(line_state)
        {
            case START: // スタート後の走行処理 *****************************************************
                line_state = MOVE;

                break;

            case MOVE: // 通常走行 *****************************************************************
                Ctrl_motor_steer_alt(100, 0, 0.2);                        // 指定出力になるまで加速して走行

                if(Run_getDistance() > 1850 && flag_line[0] == 0)
                {                                                   // 指定距離に到達した場合かつフラグが立っていない場合
                    line_state = CURVE_1;                                  //状態を遷移する
                }
                else if(Run_getDistance() > 2900 && flag_line[1] == 0)
                {                                                   // 指定距離に到達した場合かつフラグが立っていない場合
                    line_state = CURVE_2;                                  //状態を遷移する
                }
                else if(Run_getDistance() > 3750 && flag_line[2] == 0)
                {                                                   // 指定距離に到達した場合かつフラグが立っていない場合
                    line_state = CURVE_Z;                                  //状態を遷移する
                }
                else if(Run_getDistance() > 5750 && flag_line[3] == 0)
                {                                                   // 指定距離に到達した場合かつフラグが立っていない場合
                    line_state = CURVE_4;                                  //状態を遷移する
                }
                // else if(Run_getDistance() > 8500)
                // {
                //     Ctrl_motor_steer(50, 0);
                //     if(Run_getRGB_R() < 60 && Run_getRGB_G() < 90 && Run_getRGB_B() < 90)
                //     {
                //         line_state = LINETRACE;
                //     }
                // }
            
                break;

            case CURVE_1: // カーブ１走行 *****************************************************************
                if(Run_getDirection() > -80)                  // 指定角度に到達するまで
                {
                    Ctrl_motor_steer(100, -50);                               //左旋回
                }
                else                                                // 指定角度に到達した場合
                {
                    flag_line[0] = 1;                                   //フラグを立てる
                    line_state = MOVE;                                     //状態を遷移する
                }

                break;

            case CURVE_2: // カーブ2走行 *****************************************************************
                if(Run_getDirection() > -220)                 // 指定角度に到達するまで
                {
                    Ctrl_motor_steer(100, -65);                               //左旋回
                }
                else                                                // 指定角度に到達した場合
                {
                    flag_line[1] = 1;                                   //フラグを立てる
                    line_state = MOVE;                                     //状態を遷移する
                }

                break;

            case CURVE_Z: // カーブZ字走行 *****************************************************************                
                if(Run_getDistance() < 4300 && Run_getDirection() < -170)
                {                                                   // 指定距離・角度に到達するまで
                    Ctrl_motor_steer(100, 65);                                // 右旋回
                }
                else if(Run_getDistance() < 4400)              //指定距離に到達するまで
                {
                    Ctrl_motor_steer(100, 0);                                 // 前進
                }
                else if(Run_getDistance() < 4800 && Run_getDirection() < -40)
                {                                                   // 指定距離・角度に到達するまで
                    Ctrl_motor_steer(100, 70);                                // 右旋回
                }
                else if(Run_getDistance() < 4900)              // 指定距離に到達するまで
                {
                    Ctrl_motor_steer(100, 0);                                 // 前進
                }
                else if(Run_getDirection() > -155)            // 指定角度に到達するまで
                {
                    Ctrl_motor_steer(100, -70);                               // 左旋回
                }
                else                                                // 指定角度に到達した場合
                {
                    flag_line[2] = 1;                                   // フラグを立てる
                    line_state = MOVE;                                     // 状態を遷移する
                }

                break;

            case CURVE_4: // カーブ4走行 *****************************************************************

                if(Run_getDistance() < 6100 && Run_getDirection() > -230)
                {                                                   // 指定距離・角度に到達するまで
                   Ctrl_motor_steer(100, -70);                                // 左旋回
                }
                else if(Run_getDistance() < 6200)              // 指定距離に到達するまで
                {
                    Ctrl_motor_steer(100, 0);                                 // 前進
                }
                else if(Run_getDirection() < -90)             // 指定角度に到達するまで
                {
                    Ctrl_motor_steer(100, 55);                                // 右旋回
                }
                else                                                // 指定角度に到達した場合
                {
                    Ctrl_motor_steer(100, 18);                                // 右旋回
                    if(Run_getRGB_R() < 60 && Run_getRGB_G() < 90 && Run_getRGB_B() < 90)      // 黒ラインを検知した場合
                    {
                        flag_line[3] = 1;                               //フラグを立てる
                        line_state = LINETRACE;                            //状態を遷移する
                    }
                }

                break;

            case LINETRACE:

                turn = Ctrl_getTurn_PID(Run_getRGB_R(), PID_TARGET_VAL);    // PID制御で旋回量を算出

                if(-50 < turn && turn < 50)             // 旋回量が少ない場合
                    Ctrl_motor_steer_alt(80, turn, 0.5);       // 加速して走行
                else                                    // 旋回量が多い場合
                    Ctrl_motor_steer_alt(60, turn, 0.5);          // 減速して走行

                if(Run_getRGB_R() < 65 && Run_getRGB_G() < 90 && Run_getRGB_B() > 70 && Run_getDistance() > 11000)    // 2つ目の青ラインを検知  Run_getDistance() > 11000
                {
                    temp = Run_getDistance();  // 検知時点でのdistanceを仮置き
                    log_stamp("\n\n\tBlue detected\n\n\n");
                    line_state = END;
                    Ctrl_motor_steer(0,0);
                    Ctrl_arm_down(100, true);
                }

                // Run_getAngle() = ev3_gyro_sensor_get_angle(gyro_sensor);
                // sprintf(message, "ANGLE:%d          ",Run_getAngle());
                // ev3_lcd_draw_string(message, 0,10);

                break;

            case END: // 青ラインを検知したら減速 **************************************************
                
                if(Run_getDistance() < temp + 100)   // 指定距離進むまで
                    power = Ctrl_getPower_Change(30, 1);  // 指定出力になるように減速
                else                        // 減速が終了
                    flag = 1;               // メインループ終了フラグ

                turn = Ctrl_getTurn_PID(Run_getRGB_R(), 55);
                Ctrl_motor_steer(power, turn);    // PID制御で走行

                break;

            case GOAL_LINE:
                Ctrl_motor_steer(0, 0);

                break;

            default: // **************************************************************************
                break;
        }
        tslp_tsk(4 * 1000U); /* 4msec周期起動 */
    }
    /**
    * Main loop END ************************************************************************************************************************************
    */
}