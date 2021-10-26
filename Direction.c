// 参考：https://qiita.com/TetsuroAkagawa
// 引用：https://qiita.com/TetsuroAkagawa/items/4e7de30523d9c7ec6241

#include "Direction.h"

#define TREAD 150.0 //車体トレッド幅(約140.0mm *ETロボコンシミュレータの取扱説明書参照) -> (150.0mm *2020年ADVクラスのDENSOチームのモデル図に記載)

static float direction = 0.0; //現在の方位

 /* 初期化 */
void Direction_init(){
    direction = 0.0;
}

 /* 方位を取得(右旋回が正転) */
float Direction_getDirection(){
    return direction;
}

/* 方位を更新 */
void Direction_update(){
    //(360 / (2 * 円周率 * 車体トレッド幅)) * (右進行距離 - 左進行距離)
    direction += (360.0 / (2.0 * PI * TREAD)) * (Distance_getDistance4msLeft() - Distance_getDistance4msRight());
}