/*
 * system_config.h
 *
 *  Created on: 2019年3月30日
 *      Author: Henry Zhu
 */

#ifndef CONFIGURE_CONFIGS_SYSTEM_CONFIG_H_
#define CONFIGURE_CONFIGS_SYSTEM_CONFIG_H_

#include <QMainWindow>

#define APA_DEBUG(t) (qDebug(t))

/****************Task Shedule Priority Level******************/
#define TIME_5MS_TASK (1)
#define CAN0_TASK     (2)
#define CAN1_TASK     (3)
#define CAN2_TASK     (4)

/****************System Configure******************/
//#define CHANGAN
#define BORUI
//#define DONG_FENG_E70

#define SIMULATION 0
/********************是否使用超声波避障使能按钮***********************/
#define ULTRASONIC_COLLISION_ENABLE  ( 1 ) // 超声避障使能按钮

#define MV_2PI           ( 6.283185307179586476925286766559  )
#define MV_PI            ( 3.1415926535897932384626433832795 )
#define MV_3PI4          ( 2.3561944901923449288469825374596 )
#define MV_PI2           ( 1.5707963267948966192313216916398 )
#define MV_PI4           ( 0.7853981633974483096156608458198 )

#define MV_2PI_SQRT_INV  ( 0.3989422804014326779399460599343 )
#define MV_PI_SQRT_INV   ( 0.5641895835477562869480794515607 )
#define MV_PI_SQRT       ( 1.7724538509055160272981674833411 )
#endif /* CONFIGURE_CONFIGS_SYSTEM_CONFIG_H_ */
