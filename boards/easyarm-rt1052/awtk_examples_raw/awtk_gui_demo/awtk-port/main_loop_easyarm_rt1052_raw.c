/**
 * file:   main_loop_stm32_raw.c
 * author: li xianjing <xianjimli@hotmail.com>
 * brief:  main loop for stm32
 *
 * copyright (c) 2018 - 2018 Guangzhou ZHIYUAN Electronics Co.,Ltd. 
 *
 * this program is distributed in the hope that it will be useful,
 * but without any warranty; without even the implied warranty of
 * merchantability or fitness for a particular purpose.  see the
 * license file for more details.
 *
 */

/**
 * history:
 * ================================================================
 * 2018-05-11 li xianjing <xianjimli@hotmail.com> created
 *
 */
#include "fsl_common.h"
#include "fsl_elcdif.h"
#include "fsl_debug_console.h"


#include "base/g2d.h"
#include "base/idle.h"
#include "base/timer.h"
#include "lcd/lcd_mem.h"
#include "tkc/mem.h"
#include "base/lcd.h"
#include "lcd/lcd_mem_bgr565.h"
#include "main_loop/main_loop_simple.h"

extern uint8_t *online_fb_addr;
extern uint8_t *offline_fb_addr;


extern int BOARD_Touch_Poll(int *pX, int *pY, int *pPressFlg);

uint8_t platform_disaptch_input(main_loop_t* loop) {
	int iX = 0;
	int iY = 0;
    int iPressflg = 0;
	
    if (BOARD_Touch_Poll(&iX, &iY, &iPressflg)) {
        main_loop_post_pointer_event(loop, (iPressflg ? TRUE : FALSE), iX, iY);
    }
	
  return 0;
}

lcd_t* platform_create_lcd(wh_t w, wh_t h) {
  return lcd_mem_bgr565_create_double_fb(w, h, online_fb_addr, offline_fb_addr);
}


#include "main_loop/main_loop_raw.inc"

