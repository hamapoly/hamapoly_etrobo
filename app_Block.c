#include "app_Block.h"

/* マクロ定義 */

/* グローバル変数 */
static const sensor_port_t
    sonar_sensor    = EV3_PORT_3;

/* 関数 */
void section_Block()
{
    /* ローカル変数 */
    float temp = 0.0;       // 走行距離、方位の一時保存用

    int8_t flag = 0;

    int8_t power = 0;   // モーターの出力値を格納する変数(-100 ~ +100)
    int16_t turn = 0;   // モーターによる旋回量を格納する変数(-200 ~ +200)

    /* 列挙 */
    enum {
        PRE,
        START,
        MOVE,
        CURVE,
        LINE,
        LINE_2,
        RETURN,
        END
    } r_state = PRE;

    /* 初期化処理 */
    Run_init();         // 走行データを初期化
    Ctrl_initPID();     // PIDの値を初期化

    /**
    * Main loop ****************************************************************************************************************************************
    */
    while(1)
    {
        if(flag == 1)   // 終了フラグを確認
            break;      // メインループ終了

        switch(r_state)
        {
            case PRE: // 区間単体での練習用case *************************************************
                turn = Ctrl_getTurn_PID(Run_getRGB_R(), 64);    // PID制御を用いて旋回値を取得
                Ctrl_motor_steer(20, turn);                       // 指定出力で走行

                if(Run_getRGB_R() < 75 && Run_getRGB_G() < 95 && Run_getRGB_B() > 120) // 青色検知
                {
                    temp = Run_getDistance();
                    turn = 0;
                    r_state = START;
                }

                break;
            case START: // ********************************************************************
                if(Run_getDirection() < 40)
                {
                    turn = Ctrl_getTurn_Change(73, 0.5);
                    Ctrl_motor_steer_alt(80, turn, 0.5);
                }
                else
                {
                    turn = Ctrl_getTurn_Change(0, 0.5);
                    Ctrl_motor_steer(60, turn);
                }

                if(turn == 0 && Run_getDistance() > 200)
                    r_state = MOVE;

                break;

            case MOVE: // *********************************************************************
                if(Run_getRGB_R() > 90 && Run_getRGB_G() > 90 && Run_getRGB_B() < 30)  // 黄色検知
                {
                    log_stamp("\n\n\tYellow detected\n\n\n");
                    r_state = CURVE;
                }
                else if(Run_getDistance() > temp + 1000)              // もしくは指定距離に到達した場合
                {
                    log_stamp("\n\n\tReached ditance\n\n\n");
                    r_state = CURVE;
                }

                break;

            case CURVE:   // ********************************************************************
                Ctrl_motor_steer_alt(50, 27, 0.1);                // 指定速度まで減速しつつ右曲がりに前進

                if(Run_getRGB_R() < 60 && Run_getRGB_G() < 60 && Run_getRGB_B() < 60)  // 黒色検知
                {
                    Ctrl_motor_steer(0, 0);                           // モーター停止
                    tslp_tsk(300 * 1000U);                      // 待機
                    Ctrl_runDirection(20, 200, 30);              // 右旋回
                    r_state = LINE;
                }

                break;

            case LINE:  // ********************************************************************
                turn = Ctrl_getTurn_PID(Run_getRGB_R(), 64);    // PID制御を用いて旋回値を取得
                Ctrl_motor_steer_alt(20, turn * -1, 0.5);         // 加速しつつライントレース走行

                if(Run_getRGB_R() > 75 && Run_getRGB_G() < 40 && Run_getRGB_B() < 50)  //赤色検知
                {
                    log_stamp("\n\n\tRed detected\n\n\n");
                    turn = 0;
                    temp = Run_getDistance();
                    r_state = RETURN;
                }

                break;

            case RETURN:   // ********************************************************************
                if(Run_getDistance() < temp + 1100)      //1170
                {
                    if(Run_getDirection() < 240)         //250
                    {
                        turn = Ctrl_getTurn_Change(34, 0.3);       //42, 04
                        Ctrl_motor_steer_alt(50, turn, 0.4);
                    }
                    else
                    {
                        turn = Ctrl_getTurn_Change(0, 0.4);
                        Ctrl_motor_steer(50, turn);
                    }
                }
                else                                // 指定距離に到達した場合
                {
                    
                    if(Run_getDirection() < 320)
                    {
                        turn = Ctrl_getTurn_Change(20, 0.3);           //90, 0.4
                        Ctrl_motor_steer_alt(15, turn, 0.1);        // 減速して右曲がりに走行
                    }
                    else
                    {
                        Ctrl_motor_steer(15, 0);
                    }

                    if( Run_getRGB_R() < 60 && Run_getRGB_G() < 60 && Run_getRGB_B() < 60)         // 黒色検知
                    {                    
                        Ctrl_motor_steer(0,0);
                        tslp_tsk(300 * 1000U);  // 待機
                        Ctrl_runDirection(20, 200, 20);
                        r_state = END;
                        log_stamp("\n\n\nlinetrace\n\n\n");
                    }
                    else if(Run_getRGB_R() < 75 && Run_getRGB_G() < 95 && Run_getRGB_B() > 120)    // 青色検知
                    {
                        Ctrl_motor_steer(0,0);
                        tslp_tsk(300 * 1000U);  // 待機
                        Ctrl_runDirection(20, 200, 20);
                        r_state = END;
                        log_stamp("\n\n\nlinetrace\n\n\n");
                    }
                }
                break;

            case END:   // ********************************************************************
                
                if(sampling_turn(turn))
                {
                    Ctrl_motor_steer(20, 0);
                    
                    if(ev3_ultrasonic_sensor_get_distance(sonar_sensor) <= 4)
                    {
                        Ctrl_motor_steer(0, 0);                       // ガレージの壁を検知して停車
                        flag = 1;                               // 終了フラグ
                    }
                }
                else
                {
                    turn = Ctrl_getTurn_PID(Run_getRGB_R(), 48);    // PID制御を用いて旋回値を取得
                    Ctrl_motor_steer_alt(10, turn * -1, 0.5);         // 加速しつつライントレース走行
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