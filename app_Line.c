#include "app_Line.h"

/* マクロ定義 */
#define MOTOR_POWER     80  // モーターの出力値(-100 ~ +100)
#define PID_TARGET_VAL  64  // PID制御におけるセンサrgb.rの目標値 *参考 : https://qiita.com/pulmaster2/items/fba5899a24912517d0c5

/* グローバル変数 */
static const sensor_port_t
    color_sensor    = EV3_PORT_2;

/* 構造体 */
typedef enum {
    START,
    LINETRACE,
    END
    } RUN_STATE;

static RUN_STATE r_state = START;

/* メイン関数 */
void Line_task()
{
    /* ローカル変数 ******************************************************************************************/
    rgb_raw_t rgb;

    float temp = 0.0;   // 距離、方位の一時保存用

    int8_t flag = 0;
    int8_t flag_line[] = {0, 0, 0, 0};
    int8_t power = MOTOR_POWER;

    int16_t turn = 0;

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
        // 周期ハンドラによる取得値も利用できるが、精度を求める場合は使用直前に値を更新した方が良いと思われる
        
        Distance_update();
        Direction_update();
        /********************************************************************************************************/

        if(flag == 1)   // 終了フラグを確認
            return;     // 関数終了

        switch(r_state)
        {
            case START: // スタート後の走行処理 *****************************************************
                
                r_state = LINETRACE;

                break;

            case LINETRACE:
                
                turn = Run_getTurn_sensorPID(rgb.r, PID_TARGET_VAL);    // PID制御で旋回量を算出

                if(-50 < turn && turn < 50)             // 旋回量が少ない場合
                    motor_ctrl_alt(80, turn, 0.5);       // 加速して走行
                else                                    // 旋回量が多い場合
                    motor_ctrl_alt(50, turn, 0.5);          // 減速して走行

                if(rgb.r < 75 && rgb.g < 95 && rgb.b > 120 && Distance_getDistance() > 9000)    // 2つ目の青ラインを検知
                {
                    temp = Distance_getDistance();  // 検知時点でのdistanceを仮置き
                    log_stamp("\n\n\tBlue detected\n\n\n");
                    r_state = END;
                }

                break;

            case END: // 青ラインを検知したら減速 **************************************************
                if(Distance_getDistance() < temp + 200)   // 指定距離進むまで
                    power = Run_getPower_change(power, 30, 1);  // 指定出力になるように減速
                else                        // 減速が終了
                    flag = 1;               // メインループ終了フラグ

                turn = Run_getTurn_sensorPID(rgb.r, PID_TARGET_VAL);
                motor_ctrl(power, turn);    // PID制御で走行

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
