#ifndef _GRID_H_
#define _GRID_H_

#include "math.h"
#include "Distance.h"

/* 初期化関数 */
void Grid_init();
/* 座標aから座標bまでの移動距離を設定する関数 */
void Grid_setDistance(int aX, int aY, int bX, int bY);
/* 座標aから座標bまでの移動距離を取得する関数 */
float Grid_getDistance();

/* 目標座標の方位を設定する関数 */
void Grid_setDirection(int aX, int aY, int bX, int bY);
/* 目標座標の方位を取得する関数 */
float Grid_getDirection();

#endif