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

// 追記箇所-------------------------------------------------------------
// ソースファイルの分割について                ：https://dev.toppers.jp/trac_user/ev3pf/wiki/UserManual *(2.2. ソースファイルの追加)
// 複数ディレクトリへの分割について(できなかった)：https://dev.toppers.jp/trac_user/ev3pf/wiki/FAQ *(Q：アプリケーションのソースコードを複数のディレクトリに分けて管理するには~)
#include "Run.h"
#include "Controller.h"
#include "app_Linetrace.h"
#include "app_Slalom.h"
#include "app_Block.h"
// 追記終了-------------------------------------------------------------

/* APIについて */
// ev3のAPI：https://www.toppers.jp/ev3pf/EV3RT_C_API_Reference/index.html
// APIのソースコードは hrp3 > sdk > common > ev3api > src　を参照
// tslp_tsk等、サービスコールについて：https://monozukuri-c.com/itron-servicecall/

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
    // 追記箇所-------------------------------------------------------------
    arm_motor       = EV3_PORT_A,
    tale_motor      = EV3_PORT_D;
    // 追記終了-------------------------------------------------------------

static int      bt_cmd = 0;     /* Bluetoothコマンド 1:リモートスタート */
static FILE     *bt = NULL;     /* Bluetoothファイルハンドル */

// 追記箇所-------------------------------------------------------------
static FILE *outputfile;    // 出力ストリーム

static int8_t logflag = 0;

uint8_t cnt_cyc = 0;    // 周期ハンドラのタッチセンサ終了処理用
// 追記終了-------------------------------------------------------------

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

/* 関数プロトタイプ宣言 */
static int sonar_alert(void);
static void _syslog(int level, char* text);
static void _log(char* text);
//static void tail_control(signed int angle);
//static void backlash_cancel(signed char lpwm, signed char rpwm, int32_t *lenc, int32_t *renc);

// 追記箇所-------------------------------------------------------------
static void log_open(char* filename);

// void log_stamp(char *stamp);     // Run.hでextern宣言
// extern宣言の記述について：https://www.khstasaba.com/?p=849
// 追記終了-------------------------------------------------------------

/* メインタスク */
void main_task(intptr_t unused)
{
    //signed char forward;      /* 前後進命令 */
    //signed char turn;         /* 旋回命令 */
    //signed char pwm_L, pwm_R; /* 左右モーターPWM出力 */

    // 追記箇所-------------------------------------------------------------
    enum {
        LINETRACE,  // ライントレース区間
        SLALOM,     // スラローム区間
        BLOCK,      // ブロック搬入区間 + ガレージ停車
        GOAL        // タスク終了
    } t_state = LINETRACE;
    // 追記終了-------------------------------------------------------------

    /* LCD画面表示 */
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);

    _log("HackEV sample_c4");
    if (_LEFT)  _log("Left course:");
    else        _log("Right course:");

    /* センサー入力ポートの設定 */
    ev3_sensor_config(sonar_sensor, ULTRASONIC_SENSOR);
    ev3_sensor_config(color_sensor, COLOR_SENSOR);
    // ev3_color_sensor_get_reflect(color_sensor); /* 反射率モード */   // RGBモードを使用するため無効化
    ev3_sensor_config(touch_sensor, TOUCH_SENSOR);

    /* モーター出力ポートの設定 */
    ev3_motor_config(left_motor, LARGE_MOTOR);
    ev3_motor_config(right_motor, LARGE_MOTOR);

    // 追記箇所-------------------------------------------------------------
    // ポートの設定
    ev3_sensor_config(gyro_sensor, GYRO_SENSOR);    // ジャイロセンサー
    
    ev3_motor_config(arm_motor, LARGE_MOTOR);       // 前部のアーム
    ev3_motor_config(tale_motor, MEDIUM_MOTOR);     // 後部の尻尾
    // 追記終了-------------------------------------------------------------

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

    // 追記箇所-------------------------------------------------------------
    // 初期化処理
    ev3_gyro_sensor_reset(gyro_sensor);     // ジャイロセンサーの初期化
    Run_init();                             // 走行時間を初期化
    Ctrl_initPID();                          // PID用変数の初期化

    // タスク,ハンドラ起動処理
    // act_tsk(SHUTDOWN_TASK);     // タスク
    sta_cyc(CYC_DATALOG_TSK);   // 周期ハンドラ
    // 追記終了-------------------------------------------------------------

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
            case LINETRACE:
                log_open("Log_Linetrace.txt");   // txtファイル出力処理

                Ctrl_arm_up(100, true);     // 実機用
                section_Linetrace();        // スタート直後からタスク開始 -> スラローム手前の青ラインを検知してタスク終了

                t_state = SLALOM;           // スラローム区間へ移行
                break;

            case SLALOM:
                log_open("Log_Slalom.txt"); // txtファイル出力処理

                section_Slalom();           // ライントレース区間終了直後からタスク開始 -> スラローム板を降りた後、ラインに復帰してタスク終了

                t_state = BLOCK;            // ブロック搬入区間へ移行
                break;

            case BLOCK:
                log_open("Log_Block.txt");  // txtファイル出力処理

                section_Block();            // スラローム区間終了直後からタスク開始 -> ブロックを運搬しつつ、ガレージに停車してタスク終了

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

        logflag = 0;        // ファイル書き込み停止フラグ(周期ハンドラ用)
        fclose(outputfile); // txtファイル出力終了
    }
    /**
    * Main loop END ***********************************************************************************************************************************
    */

    // 追記箇所-------------------------------------------------------------
    // タスク,ハンドラ終了処理
    // ter_tsk(SHUTDOWN_TASK);     // タスク
    stp_cyc(CYC_DATALOG_TSK);   // 周期ハンドラ
    // 追記終了-------------------------------------------------------------

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

// 追記箇所-----------------------------------------------------------------------------------------------------------------------------------------

// 引数filenameに入力した文字列のファイルを書き込み用にオープンする関数
    // 参考：https://ylb.jp/2006b/proc/fileio/fileoutput.html   https://9cguide.appspot.com/17-01.html
    // 出力先は \\wsl$\Ubuntu-20.04\home\ユーザー名\etrobo\hrp3\sdk\workspace\simdist\hamapoly\__ev3rtfs
    // vscode左側フォルダ欄の"hrp3"から探して右クリック→"Reveal in Explorer"または"ダウンロード"(メモ帳推奨)
    // *生成されたtxtファイルを削除すると次に実行したときにファイルが生成されなくなることがあった
static void log_open(char *filename)
{
    outputfile = fopen(filename, "w");  // ファイルを書き込み用にオープン
    if(outputfile == NULL)              // オープンに失敗した場合
    {
        printf("cannot open\n");            // エラーメッセージを出して
        exit(1);                            // 異常終了
    }
    fprintf(outputfile, "R\tG\tB\tDistance\tDirection\tAngle\tPower_L\tPower_R\tTime\n");     // データの項目名をファイルに書き込み

    logflag = 1;    // ファイル書き込みフラグ
}

// 引数stampに入力した文字列をログに出力する関数
void log_stamp(char *stamp)
{
    fprintf(outputfile, stamp);
}

// タッチセンサ押下でプログラムを終了するタスク
void shutdown_task(intptr_t unused)
{
    static uint8_t cnt = 0;

    while(1)
    {
        if(!ev3_touch_sensor_is_pressed(touch_sensor) && cnt > 50)
        {
            ter_tsk(MAIN_TASK);                 // mainタスク終了

            log_stamp("\n\n\tShutdown\n\n\n");
            logflag = 0;                        // ファイル書き込みoff
            fclose(outputfile);                 // txtファイル出力終了

            stp_cyc(CYC_DATALOG_TSK);           // 周期ハンドラ停止

            ev3_motor_stop(left_motor, false);  // 停車
            ev3_motor_stop(right_motor, false);

            ext_tsk();
        }
        
        if(ev3_touch_sensor_is_pressed(touch_sensor) && cnt < 100)
            cnt++;
        else if(!ev3_touch_sensor_is_pressed(touch_sensor))
            cnt = 0;

        tslp_tsk(8 * 1000U); /* 8msec周期起動 */
    }
}

// 5msごとに計測値の更新を行う周期ハンドラ (*シミュレータの場合、4ms以下の周期起動にするとtimescaleが1を下回ることがある)
    // タスク・周期ハンドラについて(各種計測値の更新などに利用)：https://qiita.com/koushiro/items/22a10c7dd451291fd95b , https://qiita.com/yamanekko/items/7ddb6029820d3cfbd583
    // 上記機能APIの名称・仕様と変更点                        ：https://dev.toppers.jp/trac_user/ev3pf/wiki/FAQ *(Q：周期的な処理を追加するためには~ Q：タスクの優先度を変更するには~)
    // もっと詳しいやつ                                       ：https://www.tron.org/ja/page-722/
    // CRE_CYCの記述については workspace > periodic-task を参考
void datalog_cyc(intptr_t unused)
{
    int32_t cur_angle = ev3_motor_get_counts(arm_motor);    // 現在のモーター角度

    Run_update();       // 時間、RGB値、位置角度を更新

    if(logflag == 1)    // ファイル書き込みフラグを確認
    {
        fprintf(outputfile, "%d\t%d\t%d\t%8.3f\t%9.1f\t%4d\t%4d\t%4d\t%6dms\t%d\n", // txtファイル書き込み処理
        Run_getRGB_R(),
        Run_getRGB_G(),
        Run_getRGB_B(),
        Run_getDistance(),
        Run_getDirection(),
        Run_getAngle(),
        Run_getPower_L(),
        Run_getPower_R(),
        Run_getTime() * 5,
        cur_angle
        );
    }

    // タッチセンサによる停止処理(実機でタスクが機能しない問題を解決できていないため、タッチセンサで実機を停止させたい場合はこの処理をコメント解除する)
    if(!ev3_touch_sensor_is_pressed(touch_sensor) && cnt_cyc > 50)
    {
        ter_tsk(MAIN_TASK);                 // mainタスク終了

        log_stamp("\n\n\tShutdown\n\n\n");
        logflag = 0;                        // ファイル書き込みoff
        fclose(outputfile);                 // txtファイル出力終了

        ev3_motor_stop(left_motor, false);  // 停車
        ev3_motor_stop(right_motor, false);

        stp_cyc(CYC_DATALOG_TSK);           // 周期ハンドラ停止
    }
    if(ev3_touch_sensor_is_pressed(touch_sensor) && cnt_cyc < 100)
        cnt_cyc++;
    else if(!ev3_touch_sensor_is_pressed(touch_sensor))
        cnt_cyc = 0;
}

// 追記終了-----------------------------------------------------------------------------------------------------------------------------------------