#include "app_Block.h"

/* マクロ定義 */

/* グローバル変数 */
static const sensor_port_t
    color_sensor    = EV3_PORT_2,
    sonar_sensor    = EV3_PORT_3;

/* 構造体 */
typedef enum {
    PRE,
    START,
    MOVE,
    CURVE,
    LINE,
    LINE_2,
    RETURN,
    END
    } RUN_STATE;

static RUN_STATE r_state = PRE;

/* 関数 */
void Block_task()
{
    /* ローカル変数 ******************************************************************************************/
    rgb_raw_t rgb;

    float temp = 0.0;       // 走行距離、方位の一時保存用

    float distance = 0.0;   // 走行距離
    float direction = 0.0;  // 方位

    int8_t flag = 0;
    int8_t edge = 0;    // 1 でラインの左側をトレース、-1 で右側をトレース

    int8_t power = 0;   // モーターの出力値を格納する変数(-100 ~ +100)
    int16_t turn = 0;   // モーターによる旋回量を格納する変数(-200 ~ +200)

    /* 初期化処理 ********************************************************************************************/
    // 別ソースコード内の計測用static変数を初期化する(初期化を行わないことで、以前の区間から値を引き継ぐことができる)
    Distance_init();    // 距離を初期化
    Direction_init();   // 方位を初期化

    Run_init();         // 走行時間を初期化
    Run_PID_init();     // PIDの値を初期化

    /**
    * Main loop ****************************************************************************************************************************************
    */
    while(1)
    {
        /* 値の更新 **********************************************************************************************/
        Distance_update();
        Direction_update();

        distance = Distance_getDistance();      // 走行距離を取得
        direction = Direction_getDirection();   // 方位を取得

        ev3_color_sensor_get_rgb_raw(color_sensor, &rgb);   //カラーセンサーの値を取得して 構造体"rgb" に格納
        /********************************************************************************************************/

        // ここに処理を記述
        if(flag == 1)   // 終了フラグを確認
            break;      // メインループ終了

        switch(r_state)
        {
            case PRE: // 区間単体での練習用case *************************************************
                turn = Run_getTurn_sensorPID(rgb.r, 64);    // PID制御を用いて旋回値を取得
                motor_ctrl(20, turn);                       // 指定出力で走行

                if(rgb.r < 75 && rgb.g < 95 && rgb.b > 120) // 青色検知
                {
                    temp = Distance_getDistance();
                    turn = 0;
                    r_state = START;
                }

                break;
            case START: // ********************************************************************
                if(direction < 40)
                {
                    turn = Run_getTurn_change(turn, 73, 0.5);
                    motor_ctrl_alt(80, turn, 0.5);
                }
                else
                {
                    turn = Run_getTurn_change(turn, 0, 0.5);
                    motor_ctrl(60, turn);
                }

                if(turn == 0 && distance > 200)
                    r_state = MOVE;

                break;

            case MOVE: // *********************************************************************
                if(rgb.r > 90 && rgb.g > 90 && rgb.b < 30)  // 黄色検知
                {
                    log_stamp("\n\n\tYellow detected\n\n\n");
                    r_state = CURVE;
                }
                else if(distance > temp + 1000)              // もしくは指定距離に到達した場合
                {
                    log_stamp("\n\n\tReached ditance\n\n\n");
                    r_state = CURVE;
                }

                break;

            case CURVE:   // ********************************************************************
                motor_ctrl_alt(50, 27, 0.1);                // 指定速度まで減速しつつ右曲がりに前進

                if(rgb.r < 60 && rgb.g < 60 && rgb.b < 60)  // 黒色検知
                {
                    motor_ctrl(0, 0);                           // モーター停止
                    tslp_tsk(300 * 1000U);                      // 待機
                    Run_setDirection(20, 200, 30);              // 右旋回
                    r_state = LINE;
                }

                break;

            case LINE:  // ********************************************************************
                turn = Run_getTurn_sensorPID(rgb.r, 64);    // PID制御を用いて旋回値を取得
                motor_ctrl_alt(20, turn * -1, 0.5);         // 加速しつつライントレース走行

                if(rgb.r > 75 && rgb.g < 40 && rgb.b < 50)  //赤色検知
                {
                    log_stamp("\n\n\tRed detected\n\n\n");
                    turn = 0;
                    temp = Distance_getDistance();
                    r_state = RETURN;
                }

                break;

            case RETURN:   // ********************************************************************
                if(distance < temp + 1100)      //1170
                {
                    if(direction < 240)         //250
                    {
                        turn = Run_getTurn_change(turn, 34, 0.3);       //42, 04
                        motor_ctrl_alt(50, turn, 0.4);
                    }
                    else
                    {
                        turn = Run_getTurn_change(turn, 0, 0.4);
                        motor_ctrl(50, turn);
                    }
                }
                else                                // 指定距離に到達した場合
                {
                    
                    if(direction < 320)
                    {
                        turn = Run_getTurn_change(turn, 20, 0.3);           //90, 0.4
                        motor_ctrl_alt(15, turn, 0.1);        // 減速して右曲がりに走行
                    }
                    else
                    {
                        motor_ctrl(15, 0);
                    }

                    if( rgb.r < 60 && rgb.g < 60 && rgb.b < 60)         // 黒色検知
                    {                    
                        motor_ctrl(0,0);
                        tslp_tsk(300 * 1000U);  // 待機
                        Run_setDirection(20, 200, 20);
                        r_state = END;
                        log_stamp("\n\n\nlinetrace\n\n\n");
                    }
                    else if(rgb.r < 75 && rgb.g < 95 && rgb.b > 120)    // 青色検知
                    {
                        motor_ctrl(0,0);
                        tslp_tsk(300 * 1000U);  // 待機
                        Run_setDirection(20, 200, 20);
                        r_state = END;
                        log_stamp("\n\n\nlinetrace\n\n\n");
                    }
                }
                break;

            case END:   // ********************************************************************
                
                if(sampling_turn(turn))
                {
                    motor_ctrl(20, 0);
                    
                    if(ev3_ultrasonic_sensor_get_distance(sonar_sensor) <= 4)
                    {
                        motor_ctrl(0, 0);                       // ガレージの壁を検知して停車
                        flag = 1;                               // 終了フラグ
                    }
                }
                else
                {
                    turn = Run_getTurn_sensorPID(rgb.r, 50);    // PID制御を用いて旋回値を取得
                    motor_ctrl_alt(10, turn * -1, 0.5);         // 加速しつつライントレース走行
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
