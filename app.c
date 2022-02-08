/**
 ******************************************************************************
 ** ファイル名 : app.c
 **
 ** 概要 : 二輪差動型ライントレースロボットのTOPPERS/HRP3用Cサンプルプログラム
 **
 ** 注記 : sample_c4 (sample_c3にBluetooth通信リモートスタート機能を追加)
 ******************************************************************************
 **/

#include "ev3api.h"
#include "app.h"
#include "etroboc_ext.h"

/* 追加：ヘッダファイル */
/*************************************************************************************************************************************************/
// ソースファイルの分割について                ：https://dev.toppers.jp/trac_user/ev3pf/wiki/UserManual *(2.2. ソースファイルの追加)
// 複数ディレクトリへの分割について(できなかった)：https://dev.toppers.jp/trac_user/ev3pf/wiki/FAQ *(Q：アプリケーションのソースコードを複数のディレクトリに分けて管理するには~)
#include "app_Block.h"
#include "app_Slalom.h"
/*************************************************************************************************************************************************/

/* APIについて */
// ev3のAPI：https://www.toppers.jp/ev3pf/EV3RT_C_API_Reference/index.html
// APIのソースコードは hrp3 > sdk > common > ev3api > src　を参照

#if defined(BUILD_MODULE)
    #include "module_cfg.h"
#else
    #include "kernel_cfg.h"
#endif

#define DEBUG

#if defined(DEBUG)
    #define _debug(x) (x)
#else
    #define _debug(x)
#endif

#if defined(MAKE_BT_DISABLE)
    static const int _bt_enabled = 0;
#else
    static const int _bt_enabled = 1;
#endif

/**
 * シミュレータかどうかの定数を定義します
 */
#if defined(MAKE_SIM)
    static const int _SIM = 1;
#elif defined(MAKE_EV3)
    static const int _SIM = 0;
#else
    static const int _SIM = 0;
#endif

/**
 * 左コース/右コース向けの設定を定義します
 * デフォルトは左コース(ラインの右エッジをトレース)です
 */
#if defined(MAKE_RIGHT)
    static const int _LEFT = 0;
    #define _EDGE -1
#else
    static const int _LEFT = 1;
    #define _EDGE 1
#endif

/**
 * センサー、モーターの接続を定義します
 */
static const sensor_port_t
    touch_sensor    = EV3_PORT_1,
    color_sensor    = EV3_PORT_2,
    sonar_sensor    = EV3_PORT_3,
    gyro_sensor     = EV3_PORT_4;

static const motor_port_t
    left_motor      = EV3_PORT_C,
    right_motor     = EV3_PORT_B,
    /* 追加：接続定義 *****************************************************************************************/
    arm_motor       = EV3_PORT_A,
    tale_motor      = EV3_PORT_D;
    /********************************************************************************************************/

static int      bt_cmd = 0;     /* Bluetoothコマンド 1:リモートスタート */
static FILE     *bt = NULL;     /* Bluetoothファイルハンドル */

/* 追加：グローバル変数,構造体 */
/*************************************************************************************************************************************************/
static FILE *outputfile;    // 出力ストリーム

rgb_raw_t rgb;
int16_t  run_angle = 0;

static int8_t logflag = 0;

//全体統括用の構造体
typedef enum {
    LINE,   // ライントレース区間
    SLALOM, // スラローム区間
    BLOCK,  // ブロック搬入区間
    GOAL    // タスク終了
} TASK_STATE;

//ライントレース区間用の構造体
typedef enum {
    START,
    MOVE,
    CURVE_1,
    CURVE_2,
    CURVE_Z,
    CURVE_4,
    LINETRACE,
    END,
    GOAL_LINE
    } RUN_STATE_LINE;
    
//変更テスト
static TASK_STATE t_state = LINE;
static RUN_STATE_LINE line_state = LINETRACE;
/*************************************************************************************************************************************************/

/* 下記のマクロは個体/環境に合わせて変更する必要があります */
/* sample_c1マクロ */
#define LIGHT_WHITE  23         /* 白色の光センサ値 */
#define LIGHT_BLACK  0          /* 黒色の光センサ値 */
/* sample_c2マクロ */
#define SONAR_ALERT_DISTANCE 30 /* 超音波センサによる障害物検知距離[cm] */
/* sample_c4マクロ */
//#define DEVICE_NAME     "ET0"  /* Bluetooth名 sdcard:\ev3rt\etc\rc.conf.ini LocalNameで設定 */
//#define PASS_KEY        "1234" /* パスキー    sdcard:\ev3rt\etc\rc.conf.ini PinCodeで設定 */
#define CMD_START         '1'    /* リモートスタートコマンド */

/* LCDフォントサイズ */
#define CALIB_FONT (EV3_FONT_SMALL)
#define CALIB_FONT_WIDTH (6/*TODO: magic number*/)
#define CALIB_FONT_HEIGHT (8/*TODO: magic number*/)

/* マクロ定義 */
#define MOTOR_POWER     40  // モーターの出力値(-100 ~ +100)
#define PID_TARGET_VAL  74  // PID制御におけるセンサrgb.rの目標値 *参考 : https://qiita.com/pulmaster2/items/fba5899a24912517d0c5


/* 関数プロトタイプ宣言 */
static int sonar_alert(void);
static void _syslog(int level, char* text);
static void _log(char* text);
//static void tail_control(signed int angle);
//static void backlash_cancel(signed char lpwm, signed char rpwm, int32_t *lenc, int32_t *renc);

/* 追加：関数プロトタイプ宣言 */
/*************************************************************************************************************************************************/
static void log_open(char* filename);

void Line_task();

// void log_stamp(char *stamp);     // Run.hでextern宣言
// extern宣言の記述について：https://www.khstasaba.com/?p=849
/*************************************************************************************************************************************************/

/* メインタスク */
void main_task(intptr_t unused)
{
    //signed char forward;      /* 前後進命令 */
    //signed char turn;         /* 旋回命令 */
    //signed char pwm_L, pwm_R; /* 左右モーターPWM出力 */

    /* 追加：ローカル変数 *************************************************************************************/

    /********************************************************************************************************/

    /* LCD画面表示 */
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);

    _log("HackEV sample_c4");
    if (_LEFT)  _log("Left course:");
    else        _log("Right course:");

    /* センサー入力ポートの設定 */
    ev3_sensor_config(sonar_sensor, ULTRASONIC_SENSOR);
    ev3_sensor_config(color_sensor, COLOR_SENSOR);
    // ev3_color_sensor_get_reflect(color_sensor); /* 反射率モード */
    ev3_sensor_config(touch_sensor, TOUCH_SENSOR);

    /* モーター出力ポートの設定 */
    ev3_motor_config(left_motor, LARGE_MOTOR);
    ev3_motor_config(right_motor, LARGE_MOTOR);

    /* 追加：入力ポートの設定 *********************************************************************************/
    ev3_color_sensor_get_rgb_raw(color_sensor, &rgb);   /* RGBモード */
    ev3_sensor_config(gyro_sensor, GYRO_SENSOR);    // ジャイロセンサー
    
    ev3_motor_config(arm_motor, LARGE_MOTOR);       // 前部のアーム
    ev3_motor_config(tale_motor, MEDIUM_MOTOR);     // 後部の尻尾
    /********************************************************************************************************/

    if (_bt_enabled)
    {
        /* Open Bluetooth file */
        bt = ev3_serial_open_file(EV3_SERIAL_BT);
        assert(bt != NULL);

        /* Bluetooth通信タスクの起動 */
        act_tsk(BT_TASK);
    }

    ev3_led_set_color(LED_ORANGE); /* 初期化完了通知 */

    _log("Go to the start, ready?");
    if (_SIM)   _log("Hit SPACE bar to start");
    else        _log("Tap Touch Sensor to start");

    if (_bt_enabled)
    {
        fprintf(bt, "Bluetooth Remote Start: Ready.\n", EV3_SERIAL_BT);
        fprintf(bt, "send '1' to start\n", EV3_SERIAL_BT);
    }

    /* スタート待機 */
    while(1)
    {
        //tail_control(TAIL_ANGLE_STAND_UP); /* 完全停止用角度に制御 */

        if (bt_cmd == 1)
        {
            break; /* リモートスタート */
        }

        if (ev3_touch_sensor_is_pressed(touch_sensor) == 1)
        {
            break; /* タッチセンサが押された */
        }

        tslp_tsk(10 * 1000U); /* 10msecウェイト */
    }

    /* 走行モーターエンコーダーリセット */
    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);

    ev3_led_set_color(LED_GREEN); /* スタート通知 */

    /* 追加：初期化 ******************************************************************************************/
    ev3_gyro_sensor_reset(gyro_sensor);     // ジャイロセンサーの初期化
    Run_init();                             // 走行時間を初期化
    Run_PID_init();

    /* 追加：タスク・周期ハンドラの起動 ************************************************************************/
    // act_tsk(LOGFILE_TASK);   // タスク
    sta_cyc(CYC_MEASURE_TSK);   // 周期ハンドラ
    /********************************************************************************************************/

    /**
    * Main loop ***************************************************************************************************************************************
    */
    while(1)    // 注意点：シミュレータ画面右上のリセットボタンを押しても、プログラムの処理状態はリセットされないようです。
    {
        if (ev3_button_is_pressed(BACK_BUTTON)) break;  // 走行体の画面左下のボタンを押すとMain loopが終了？

        // ライントレース区間   ：シミュレータ設定位置 [X = 3.0, Y = 0.0, Z = -16.35, R = 90]
        // スラローム区間       ：シミュレータ設定位置 [X = 10.5, Y = 0.0, Z = 15.6, R = 90]
        // ブロック搬入区間     ：シミュレータ設定位置 [X = 左エッジ24.3 | 右エッジ24.1, Y = 0.0, Z = 8.0, R = 180]
        switch(t_state)
        {
            case LINE:
                log_open("Log_Line.txt");    // txtファイル出力処理

                arm_up(100, true);
                Line_task();                // スタート直後からタスク開始 -> スラローム手前の青ラインを検知してタスク終了

                //t_state = SLALOM;           // スラローム区間へ移行
                break;

            case SLALOM:
                log_open("Log_Slalom.txt");  // txtファイル出力処理

                Slalom_task();              // ライントレース区間終了直後からタスク開始 -> スラローム板を降りた後、ラインに復帰してタスク終了

                t_state = BLOCK;            // ブロック搬入区間へ移行
                break;

            case BLOCK:
                log_open("Log_Block.txt");   // txtファイル出力処理

                Block_task();               // スラローム区間終了直後からタスク開始 -> ブロックを運搬しつつ、ガレージに停車してタスク終了

                t_state = GOAL;             // 終了処理へ移行
                break;

            case GOAL:
                ev3_motor_stop(left_motor, true);
                ev3_motor_stop(right_motor, true);

                break;

            default:
                break;
        }
        tslp_tsk(4 * 1000U); /* 4msec周期起動 */

        // logflag = 0;        // ファイル書き込み停止フラグ(周期ハンドラ用)
        // fclose(outputfile); // txtファイル出力終了
    }
    /**
    * Main loop END ***********************************************************************************************************************************
    */

    /* 追加：タスク・周期ハンドラの終了 ************************************************************************/
    // ter_tsk(LOGFILE_TASK);   // タスク
    stp_cyc(CYC_MEASURE_TSK);   // 周期ハンドラ
    /********************************************************************************************************/

    ev3_motor_stop(left_motor, false);
    ev3_motor_stop(right_motor, false);

    if (_bt_enabled)
    {
        ter_tsk(BT_TASK);
        fclose(bt);
    }

    ext_tsk();
}

//*****************************************************************************
// 関数名 : sonar_alert
// 引数 : 無し
// 返り値 : 1(障害物あり)/0(障害物無し)
// 概要 : 超音波センサによる障害物検知
//*****************************************************************************
static int sonar_alert(void)
{
    static unsigned int counter = 0;
    static int alert = 0;

    signed int distance;

    if (++counter == 40/4) /* 約40msec周期毎に障害物検知  */
    {
        /*
         * 超音波センサによる距離測定周期は、超音波の減衰特性に依存します。
         * NXTの場合は、40msec周期程度が経験上の最短測定周期です。
         * EV3の場合は、要確認
         */
        distance = ev3_ultrasonic_sensor_get_distance(sonar_sensor);
        if ((distance <= SONAR_ALERT_DISTANCE) && (distance >= 0))
        {
            alert = 1; /* 障害物を検知 */
        }
        else
        {
            alert = 0; /* 障害物無し */
        }
        counter = 0;
    }

    return alert;
}

//*****************************************************************************
// 関数名 : bt_task
// 引数 : unused
// 返り値 : なし
// 概要 : Bluetooth通信によるリモートスタート。 Tera Termなどのターミナルソフトから、
//       ASCIIコードで1を送信すると、リモートスタートする。
//*****************************************************************************
void bt_task(intptr_t unused)
{
    while(1)
    {
        if (_bt_enabled)
        {
            uint8_t c = fgetc(bt); /* 受信 */
            switch(c)
            {
            case '1':
                bt_cmd = 1;
                break;
            default:
                break;
            }
            fputc(c, bt); /* エコーバック */
        }
    }
}

//*****************************************************************************
// 関数名 : _syslog
// 引数 :   int   level - SYSLOGレベル
//          char* text  - 出力文字列
// 返り値 : なし
// 概要 : SYSLOGレベルlebelのログメッセージtextを出力します。
//        SYSLOGレベルはRFC3164のレベル名をそのまま（ERRだけはERROR）
//        `LOG_WARNING`の様に定数で指定できます。
//*****************************************************************************
static void _syslog(int level, char* text){
    static int _log_line = 0;
    if (_SIM)
    {
        syslog(level, text);
    }
    else
    {
        ev3_lcd_draw_string(text, 0, CALIB_FONT_HEIGHT*_log_line++);
    }
}

//*****************************************************************************
// 関数名 : _log
// 引数 :   char* text  - 出力文字列
// 返り値 : なし
// 概要 : SYSLOGレベルNOTICEのログメッセージtextを出力します。
//*****************************************************************************
static void _log(char *text){
    _syslog(LOG_NOTICE, text);
}

/* 追加：関数 */
/*************************************************************************************************************************************************/

// 引数filenameに入力した文字列のファイルを書き込み用にオープンする関数
    // 参考：https://ylb.jp/2006b/proc/fileio/fileoutput.html   https://9cguide.appspot.com/17-01.html
    // 出力先は \\wsl$\Ubuntu-20.04\home\ユーザー名\etrobo\hrp3\sdk\workspace\simdist\hamapoly\__ev3rtfs
    // vscode左側フォルダ欄の"hrp3"から探して右クリック→"Reveal in Explorer"または"ダウンロード"(メモ帳推奨)
static void log_open(char *filename)
{
    outputfile = fopen(filename, "w");  // ファイルを書き込み用にオープン
    if(outputfile == NULL)              // オープンに失敗した場合
    {
        printf("cannot open\n");            // エラーメッセージを出して
        exit(1);                            // 異常終了
    }
    fprintf(outputfile, "R\tG\tB\tDistance\tDirection\tAngle\tPower\tTurn\tTime\n");     // データの項目名をファイルに書き込み

    logflag = 1;    // ファイル書き込みフラグ
}

// 引数stampに入力した文字列をログに出力する関数
void log_stamp(char *stamp)
{
    fprintf(outputfile, stamp);
}

// Mainタスクのスリープ(tslp_tsk)中に実行される測定値書き込み関数 *現状main_task内のtslp_tskにしか反応していないと思われるため停止中
    // tslp_tsk等、サービスコールについて：https://monozukuri-c.com/itron-servicecall/
void logfile_task(intptr_t unused)
{
    if(logflag == 1)    // ファイル書き込みフラグを確認
    {
        fprintf(outputfile, "%d\t%d\t%d\t%8.3f\t%9.1f\t%4d\t%4d\t%4d\t%6dms\n", // txtファイル書き込み処理
         getRGB_R(),
         getRGB_G(),
         getRGB_B(),
         Distance_getDistance(),    // 走行距離を取得
         Direction_getDirection(),  // 方位を取得(右旋回が正転)
         Run_getAngle(),
         Run_getPower(),
         Run_getTurn(),
        Run_getTime() * 5);

        logflag = 0;    // ファイル書き込み停止フラグ
    }
}

// 周期ハンドラによって5msごとに計測値の更新を行う関数 *4ms以下にするとtimescaleが1を下回ることがある
    // タスク・周期ハンドラについて(各種計測値の更新などに利用)：https://qiita.com/koushiro/items/22a10c7dd451291fd95b , https://qiita.com/yamanekko/items/7ddb6029820d3cfbd583
    // 上記機能APIの名称・仕様と変更点                        ：https://dev.toppers.jp/trac_user/ev3pf/wiki/FAQ *(Q：周期的な処理を追加するためには~ Q：タスクの優先度を変更するには~)
    // もっと詳しいやつ                                       ：https://www.tron.org/ja/page-722/
    // CRE_CYCの記述については workspace > periodic-task を参考
void measure_task(intptr_t unused)
{
    static int8_t flag = 0;
    static int32_t cur_angle = 0;    // 現在のモーター角度

    Run_update();       // 時間、RGB値、位置角度を更新
    Distance_update();  // 距離を更新
    Direction_update(); // 方位を更新
    cur_angle = ev3_motor_get_counts(arm_motor);

    // logflag = 1;        // ファイル書き込みフラグ

    if(logflag == 1)    // ファイル書き込みフラグを確認
    {
        fprintf(outputfile, "%d\t%d\t%d\t%8.3f\t%9.1f\t%4d\t%4d\t%4d\t%6dms\t%d\n", // txtファイル書き込み処理
         getRGB_R(),
         getRGB_G(),
         getRGB_B(),
         Distance_getDistance(),    // 走行距離を取得
         Direction_getDirection(),  // 方位を取得(右旋回が正転)
         Run_getAngle(),
         Run_getPower(),
         Run_getTurn(),
        Run_getTime() * 5,
         cur_angle);
    }
    
    if(flag <= 60){
        flag++;
    }
    if(ev3_touch_sensor_is_pressed(touch_sensor) == 1 && flag >= 60)
    {
        logflag = 0;
        fclose(outputfile); // txtファイル出力終了
        motor_ctrl(0, 0);
        t_state = GOAL;
        line_state = GOAL_LINE;
    }
}

//*****************************************************************************
// 関数名 : Line_task
// 引数 :   なし
// 返り値 : なし
// 概要 : ライントレース区間の走行プログラム
//*****************************************************************************
void Line_task()
{
    /* ローカル変数 ******************************************************************************************/

    float temp = 0.0;   // 距離、方位の一時保存用

    int8_t flag = 0;
    int8_t flag_line[] = {0, 0, 0, 0};
    int8_t power = MOTOR_POWER;

    int16_t turn = 0;

    char message[30];

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
        ev3_color_sensor_get_rgb_raw(color_sensor, &rgb);   // RGB値を更新
        Distance_update();
        Direction_update();
        /********************************************************************************************************/

        if(flag == 1)   // 終了フラグを確認
            return;     // 関数終了

        switch(line_state)
        {
            case START: // スタート後の走行処理 *****************************************************
                line_state = MOVE;

                break;

            case MOVE: // 通常走行 *****************************************************************
                motor_ctrl_alt(100, 0, 0.2);                        // 指定出力になるまで加速して走行

                if(Distance_getDistance() > 1850 && flag_line[0] == 0)
                {                                                   // 指定距離に到達した場合かつフラグが立っていない場合
                    line_state = CURVE_1;                                  //状態を遷移する
                }
                else if(Distance_getDistance() > 2900 && flag_line[1] == 0)
                {                                                   // 指定距離に到達した場合かつフラグが立っていない場合
                    line_state = CURVE_2;                                  //状態を遷移する
                }
                else if(Distance_getDistance() > 3750 && flag_line[2] == 0)
                {                                                   // 指定距離に到達した場合かつフラグが立っていない場合
                    line_state = CURVE_Z;                                  //状態を遷移する
                }
                else if(Distance_getDistance() > 5750 && flag_line[3] == 0)
                {                                                   // 指定距離に到達した場合かつフラグが立っていない場合
                    line_state = CURVE_4;                                  //状態を遷移する
                }
                // else if(Distance_getDistance() > 8500)
                // {
                //     motor_ctrl(50, 0);
                //     if(rgb.r < 60 && rgb.g < 90 && rgb.b < 90)
                //     {
                //         line_state = LINETRACE;
                //     }
                // }
            
                break;

            case CURVE_1: // カーブ１走行 *****************************************************************
                if(Direction_getDirection() > -80)                  // 指定角度に到達するまで
                {
                    motor_ctrl(100, -50);                               //左旋回
                }
                else                                                // 指定角度に到達した場合
                {
                    flag_line[0] = 1;                                   //フラグを立てる
                    line_state = MOVE;                                     //状態を遷移する
                }

                break;

            case CURVE_2: // カーブ2走行 *****************************************************************
                if(Direction_getDirection() > -220)                 // 指定角度に到達するまで
                {
                    motor_ctrl(100, -65);                               //左旋回
                }
                else                                                // 指定角度に到達した場合
                {
                    flag_line[1] = 1;                                   //フラグを立てる
                    line_state = MOVE;                                     //状態を遷移する
                }

                break;

            case CURVE_Z: // カーブZ字走行 *****************************************************************                
                if(Distance_getDistance() < 4300 && Direction_getDirection() < -170)
                {                                                   // 指定距離・角度に到達するまで
                    motor_ctrl(100, 65);                                // 右旋回
                }
                else if(Distance_getDistance() < 4400)              //指定距離に到達するまで
                {
                    motor_ctrl(100, 0);                                 // 前進
                }
                else if(Distance_getDistance() < 4800 && Direction_getDirection() < -40)
                {                                                   // 指定距離・角度に到達するまで
                    motor_ctrl(100, 70);                                // 右旋回
                }
                else if(Distance_getDistance() < 4900)              // 指定距離に到達するまで
                {
                    motor_ctrl(100, 0);                                 // 前進
                }
                else if(Direction_getDirection() > -155)            // 指定角度に到達するまで
                {
                    motor_ctrl(100, -70);                               // 左旋回
                }
                else                                                // 指定角度に到達した場合
                {
                    flag_line[2] = 1;                                   // フラグを立てる
                    line_state = MOVE;                                     // 状態を遷移する
                }

                break;

            case CURVE_4: // カーブ4走行 *****************************************************************

                if(Distance_getDistance() < 6100 && Direction_getDirection() > -230)
                {                                                   // 指定距離・角度に到達するまで
                   motor_ctrl(100, -70);                                // 左旋回
                }
                else if(Distance_getDistance() < 6200)              // 指定距離に到達するまで
                {
                    motor_ctrl(100, 0);                                 // 前進
                }
                else if(Direction_getDirection() < -90)             // 指定角度に到達するまで
                {
                    motor_ctrl(100, 55);                                // 右旋回
                }
                else                                                // 指定角度に到達した場合
                {
                    motor_ctrl(100, 18);                                // 右旋回
                    if(rgb.r < 60 && rgb.g < 90 && rgb.b < 90)      // 黒ラインを検知した場合
                    {
                        flag_line[3] = 1;                               //フラグを立てる
                        line_state = LINETRACE;                            //状態を遷移する
                    }
                }

                break;

            case LINETRACE:

                turn = Run_getTurn_sensorPID(rgb.r, PID_TARGET_VAL);    // PID制御で旋回量を算出

                if(-50 < turn && turn < 50)             // 旋回量が少ない場合
                    motor_ctrl_alt(power, turn, 0.5);       // 加速して走行
                else                                    // 旋回量が多い場合
                    motor_ctrl_alt(power, turn, 0.5);          // 減速して走行

                if(rgb.r < 65 && rgb.g < 90 && rgb.b > 70 && Distance_getDistance() > 11000)    // 2つ目の青ラインを検知  Distance_getDistance() > 11000
                {
                    temp = Distance_getDistance();  // 検知時点でのdistanceを仮置き
                    log_stamp("\n\n\tBlue detected\n\n\n");
                    line_state = END;
                    motor_ctrl(0,0);
                    arm_down(100, true);
                }

                // run_angle = ev3_gyro_sensor_get_angle(gyro_sensor);
                // sprintf(message, "ANGLE:%d          ",run_angle);
                // ev3_lcd_draw_string(message, 0,10);

                break;

            case END: // 青ラインを検知したら減速 **************************************************
                
                if(Distance_getDistance() < temp + 100)   // 指定距離進むまで
                    power = Run_getPower_change(10, 30, 1);  // 指定出力になるように減速
                else                        // 減速が終了
                    flag = 1;               // メインループ終了フラグ

                turn = Run_getTurn_sensorPID(rgb.r, 55);
                motor_ctrl(power, turn);    // PID制御で走行

                break;

            case GOAL_LINE:
                ev3_motor_stop(left_motor, true);
                ev3_motor_stop(right_motor, true);

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