/*******************************************************************************
 * i2c.h - definitions for the i2c-bus interface			     	
 *
 * The Clear BSD License
 * Copyright (C) 2019 Damon Zhang
 * All rights reserved.
 *
 * Author : Damon Zhang
 * Website: https://damon-yun.github.io/blog.github.io/
 * E-mail : damonzhang92@gmail.com
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 * that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Modification History
 * - 1.01 19-05-15  dmaon, modified
 * - 1.00 19-05-11  damon, first implementation
 *
 * note
 *    如需观察串口打印的调试信息，需要将 PIO0_0 引脚连接 PC 串口的 TXD，
 *    PIO0_4 引脚连接 PC 串口的 RXD。
 *
 *****************************************************************************/

/*!
 * @file MIMXRT1062.h
 * @version 1.2
 * @date 2019-04-29
 * @brief CMSIS Peripheral Access Layer for MIMXRT1062
 *
 * CMSIS Peripheral Access Layer for MIMXRT1062
 */
 
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_pxp.h"
#include "fsl_cache.h"
//awtk include
#include "base/g2d.h"
#include "base/pixel.h"
/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.g2d"
#endif

/*******************************************************************************
 * Configurations
 ******************************************************************************/

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* External XTAL (OSC) clock frequency. */

/*******************************************************************************
 * Private Code
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
ret_t g2d_fill_rect(bitmap_t *fb, rect_t* dst, color_t c)
{
    return RET_NOT_IMPL;
    
    uint8_t* fb_data = NULL;
    uint32_t out_addr = 0;
    uint8_t  out_pixsize = 2;
    pxp_output_buffer_config_t outputBufferConfig = {0};
    pxp_ps_buffer_config_t     psBufferConfig = {0};
    
    fb_data = bitmap_lock_buffer_for_write(fb);
    
    if (c.rgba.a < 0xFE) {
        return RET_NOT_IMPL;
    }
    
    PXP_Reset(PXP);
    /* Disable CSC1, it is enabled by default. */
    PXP_EnableCsc1(PXP, false); 
    
    /* Output config. */
    if (fb->format == BITMAP_FMT_BGR565) {
        outputBufferConfig.pixelFormat    = kPXP_OutputPixelFormatRGB565;
        out_pixsize = 2;
    } else if (fb->format == BITMAP_FMT_BGR888) {
        outputBufferConfig.pixelFormat    = kPXP_OutputPixelFormatRGB888;
        out_pixsize = 4;
    }

    /* calculate PS start addr */
    out_addr = (uint32_t)fb_data + (fb->w * (dst->y) + dst->x) * out_pixsize;    
    outputBufferConfig.interlacedMode = kPXP_OutputProgressive;
    outputBufferConfig.buffer0Addr    = out_addr;
    outputBufferConfig.buffer1Addr    = 0;
    outputBufferConfig.pitchBytes     = fb->w * out_pixsize;
    outputBufferConfig.width          = dst->w;
    outputBufferConfig.height         = dst->h;

    PXP_SetOutputBufferConfig(PXP, &outputBufferConfig);
    
    /* Fill Back Ground Color configure. */
    PXP_SetProcessSurfaceBackGroundColor(PXP, (uint32_t)c.color & (0xFFFFFF));
    
    /* Disable PS. */
    PXP_SetProcessSurfacePosition(PXP, 0xFFFFU, 0xFFFFU, 0U, 0U);
    
    /* Disable AS. */
    PXP_SetAlphaSurfacePosition(PXP, 0xFFFFU, 0xFFFFU, 0U, 0U);
    /* Start PXP. */
    PXP_Start(PXP);

    /* Wait for process complete. */
    while (!(kPXP_CompleteFlag & PXP_GetStatusFlags(PXP)))
    {
    }
    PXP_ClearStatusFlags(PXP, kPXP_CompleteFlag);
    L1CACHE_InvalidateDCacheByRange((uint32_t)out_addr, ((dst->h - 1) * fb->w + dst->w) * out_pixsize);
    
    bitmap_unlock_buffer(fb);
    
    return RET_OK;
}

ret_t g2d_copy_image(bitmap_t* dst, bitmap_t* src, rect_t* src_r, xy_t dx, xy_t dy)
{
//    return RET_NOT_IMPL;
    
    uint8_t* src_p = NULL;
    uint8_t* dst_p = NULL;
    uint8_t* src_data = NULL;
    uint8_t* dst_data = NULL;
    uint32_t bpp = bitmap_get_bpp(dst);
    uint32_t dst_line_length = bitmap_get_line_length(dst);
    uint32_t src_line_length = bitmap_get_line_length(src);
    return_value_if_fail(dst != NULL && src != NULL && src_r != NULL, RET_BAD_PARAMS);
    return_value_if_fail(dst->format == src->format, RET_BAD_PARAMS);
    return_value_if_fail(src_r->w != 0 && src_r->h != 0, RET_BAD_PARAMS);
    
    src_data = bitmap_lock_buffer_for_read(src);
    dst_data = bitmap_lock_buffer_for_write(dst);
    return_value_if_fail(src_data != NULL && dst_data != NULL, RET_BAD_PARAMS);


    src_p = (uint8_t*)(src_data) + src_r->y * src_line_length + src_r->x * bpp;
    dst_p = (uint8_t*)(dst_data) + dy * dst_line_length + dx * bpp;
    
    L1CACHE_CleanDCacheByRange((uint32_t)src_p, ((src_r->h - 1) * src->w + src_r->w) * bpp);   
    L1CACHE_CleanDCacheByRange((uint32_t)dst_p, ((src_r->h - 1) * src->w + src_r->w) * bpp);   
    
    pxp_output_buffer_config_t outputBufferConfig = {0};
    pxp_ps_buffer_config_t     psBufferConfig = {0};    
    pxp_output_pixel_format_t  output_pixel_format;
    pxp_ps_pixel_format_t      ps_pixel_format;
    uint32_t out_pixsize, ps_pixsize;
    
    if (src->format == BITMAP_FMT_BGR565) {
        ps_pixsize = 2;
        ps_pixel_format = kPXP_PsPixelFormatRGB565;
    }  else {
        ps_pixsize = 4;
        ps_pixel_format = kPXP_PsPixelFormatRGB888;
    }
    if (dst->format == BITMAP_FMT_BGR565) {
        out_pixsize = 2;
        output_pixel_format = kPXP_OutputPixelFormatRGB565;
    }  else {
        out_pixsize = 4;
        output_pixel_format = kPXP_OutputPixelFormatRGB888;
    }


    PXP_Reset(PXP);
    /* Disable CSC1, it is enabled by default. */
    PXP_EnableCsc1(PXP, false);
    /* PS configure. */
    psBufferConfig.bufferAddr = (uint32_t )src_p;
    psBufferConfig.bufferAddrU = 0;
    psBufferConfig.bufferAddrV = 0;
    psBufferConfig.pitchBytes = src->w * ps_pixsize;
    psBufferConfig.pixelFormat = ps_pixel_format;
    PXP_SetProcessSurfaceBufferConfig(PXP, &psBufferConfig);
    PXP_SetProcessSurfacePosition(PXP, 0, 0, src_r->w - 1, src_r->h - 1);
    
    /* Disable AS. */
    PXP_SetAlphaSurfacePosition(PXP, 0xFFFFU, 0xFFFFU, 0U, 0U);
    
    /* Output config. */
    outputBufferConfig.pixelFormat    = output_pixel_format;
    outputBufferConfig.interlacedMode = kPXP_OutputProgressive;
    outputBufferConfig.buffer0Addr    = (uint32_t)dst_p;
    outputBufferConfig.buffer1Addr    = 0;
    outputBufferConfig.pitchBytes     = dst->w * out_pixsize;
    outputBufferConfig.width          = src_r->w;
    outputBufferConfig.height         = src_r->h;

    PXP_SetOutputBufferConfig(PXP, &outputBufferConfig);
    /* Start PXP. */
    PXP_Start(PXP);
    /* Wait for process complete. */
    while (!(kPXP_CompleteFlag & PXP_GetStatusFlags(PXP)))
    {
    }
    PXP_ClearStatusFlags(PXP, kPXP_CompleteFlag);
    
    L1CACHE_InvalidateDCacheByRange((uint32_t)dst_p, src_r->h * dst->w * bpp);

    bitmap_unlock_buffer(src);
    bitmap_unlock_buffer(dst);

  return RET_OK;

}

/**
 * @method g2d_copy_image
 * 把图片指定的区域进行旋转并拷贝到framebuffer相应的区域，本函数主要用于辅助实现横屏和竖屏的切换，一般支持90度旋转即可。
 * @param {bitmap_t*} fb framebuffer对象。
 * @param {bitmap_t*} img 图片对象。
 * @param {rect_t*} src 要旋转并拷贝的区域。
 * @param {lcd_orientation_t} o 旋转角度(一般支持90度即可)。
 *
 * @return {ret_t} 返回RET_OK表示成功，否则表示失败，返回失败则上层用软件实现。
 */
ret_t g2d_rotate_image(bitmap_t* fb, bitmap_t* img, rect_t* src, lcd_orientation_t o)
{
    return RET_NOT_IMPL;
    
    uint8_t* fb_data = NULL;
    uint8_t* img_data = NULL;
    pxp_output_buffer_config_t outputBufferConfig = {0};
    pxp_ps_buffer_config_t     psBufferConfig = {0};
    
    img_data = (uint8_t *)img;
    fb_data = (uint8_t *)fb;

    /* 仅支持图形边长为8的整数倍,否则旋转后有锯齿 */
    if ((src->h % 8 != 0)||(src->w % 8 != 0)) {
       return RET_NOT_IMPL;  /* 返回，使用软件旋转 */
    }

    
    PXP_Reset(PXP);
    /* Disable CSC1, it is enabled by default. */
    PXP_EnableCsc1(PXP, false);
    
    /* PS configure. */
    PXP_SetProcessSurfaceBackGroundColor(PXP, 0);
    
    psBufferConfig.pixelFormat = kPXP_PsPixelFormatRGB888;    
    psBufferConfig.bufferAddr = (uint32_t )img_data;
    psBufferConfig.bufferAddrU = 0;
    psBufferConfig.bufferAddrV = 0;
    psBufferConfig.pitchBytes = 240*4;

    PXP_SetProcessSurfaceBufferConfig(PXP, &psBufferConfig);

    /* Disable AS. */
    PXP_SetAlphaSurfacePosition(PXP, 0xFFFFU, 0xFFFFU, 0U, 0U);
    
    /* Output config. */
    outputBufferConfig.pixelFormat    = kPXP_OutputPixelFormatRGB888;
    outputBufferConfig.interlacedMode = kPXP_OutputProgressive;
    outputBufferConfig.buffer0Addr    = (uint32_t)fb_data;
    outputBufferConfig.buffer1Addr    = 0;
    outputBufferConfig.pitchBytes     = 480*4;


    PXP_SetOutputBufferConfig(PXP, &outputBufferConfig);
    
    switch (o) {
    case LCD_ORIENTATION_0:
        PXP_SetRotateConfig(PXP, kPXP_RotateProcessSurface, kPXP_Rotate0, kPXP_FlipDisable);
        outputBufferConfig.width          = 480;
        outputBufferConfig.height         = 272;
        PXP_SetProcessSurfacePosition(PXP, 0, 0, 240-1, 136-1);
        break;
        
    case LCD_ORIENTATION_90:
        PXP_SetRotateConfig(PXP, kPXP_RotateProcessSurface, kPXP_Rotate90, kPXP_FlipDisable);
        outputBufferConfig.width    = 136;
        outputBufferConfig.height   = 240;
        PXP_SetProcessSurfacePosition(PXP, 0, 0, 136-1, 240-1);
        break;
        
    case LCD_ORIENTATION_180:
        PXP_SetRotateConfig(PXP, kPXP_RotateProcessSurface, kPXP_Rotate180, kPXP_FlipDisable);
        PXP_SetProcessSurfacePosition(PXP, 0, 0, 240-1, 136-1);
        break;
        
    case LCD_ORIENTATION_270:
        PXP_SetRotateConfig(PXP, kPXP_RotateProcessSurface, kPXP_Rotate270, kPXP_FlipDisable);
        PXP_SetProcessSurfacePosition(PXP, 0, 0, 136-1, 240-1);
        break;
        
    default:
        PXP_Reset(PXP);
        return RET_NOT_IMPL;
    }
    PXP_SetOutputBufferConfig(PXP, &outputBufferConfig);
    /* Start PXP. */
    PXP_Start(PXP);
    /* Wait for process complete. */
    while (!(kPXP_CompleteFlag & PXP_GetStatusFlags(PXP)))
    {
    }
    PXP_ClearStatusFlags(PXP, kPXP_CompleteFlag);
    
    return RET_OK;
}


/**
 * @method g2d_blend_image
 * 把图片指定的区域渲染到framebuffer指定的区域，src的大小和dst的大小不一致则进行缩放。
 * 1.硬件不支持缩放，则返回NOT_IMPL。
 * 2.硬件不支持全局alpha，global_alpha!=0xff时返回NOT_IMPL。
 * @param {bitmap_t*} fb framebuffer对象。
 * @param {bitmap_t*} img 图片对象。
 * @param {rect_t*} dst 目的区域。
 * @param {rect_t*} src 源区域。
 * @param {uint8_t} global_alpha 全局alpha。
 *
 * @return {ret_t} 返回RET_OK表示成功，否则表示失败，返回失败则上层用软件实现。
 */
ret_t g2d_blend_image(bitmap_t* fb, bitmap_t* img, rect_t* dst, rect_t* src, uint8_t global_alpha)
{
//    return RET_NOT_IMPL;
    uint8_t* fb_data = NULL;
    uint8_t* img_data = NULL;
    uint8_t* scale_data = NULL;
    pxp_output_pixel_format_t  output_pixel_format;
    pxp_ps_pixel_format_t      ps_pixel_format;
    pxp_as_pixel_format_t      as_pixel_format;
    uint32_t out_addr, ps_addr, as_addr;
    uint32_t img_flush_size, fb_flush_size;
    uint32_t out_pixsize, ps_pixsize, as_pixsize;
    uint32_t needScale = 0;
    
    pxp_output_buffer_config_t outputBufferConfig = {0};
    pxp_ps_buffer_config_t     psBufferConfig     = {0};
    pxp_as_buffer_config_t     asBufferConfig     = {0};
    pxp_as_blend_config_t      asBlendConfig      = {0};

    
    fb_data = bitmap_lock_buffer_for_write(fb);
    img_data = bitmap_lock_buffer_for_read(img);    
    
    /* Dont Support Scale, Maybe later */
    if ((src->w != dst->w) || (src->h != dst->h)) {
        return RET_NOT_IMPL;  //Dont Support Scale, Maybe later
        needScale = 1;  
    }
    
    if (needScale) {
        scale_data = (uint8_t*)SDK_Malloc(dst->w * dst->h * out_pixsize, 64);
        
        if (img->format == BITMAP_FMT_BGR565) {
            as_pixsize = 2;
            as_pixel_format = kPXP_AsPixelFormatRGB565;
            out_pixsize = 2;
            output_pixel_format = kPXP_OutputPixelFormatRGB565;
        } else {
            as_pixsize = 4;
            as_pixel_format = kPXP_AsPixelFormatARGB8888;
            out_pixsize = 4;
            output_pixel_format = kPXP_OutputPixelFormatARGB8888;
        }

        PXP_Reset(PXP);
        /* Disable CSC1, it is enabled by default. */
        PXP_EnableCsc1(PXP, false);
        
        /* PS configure. */
        psBufferConfig.bufferAddr = (uint32_t)img_data;    /* PS is Source FrameBuffer Rect */
        psBufferConfig.bufferAddrU = 0;
        psBufferConfig.bufferAddrV = 0;
        psBufferConfig.pitchBytes = fb->w * out_pixsize;
        psBufferConfig.pixelFormat = ps_pixel_format;
        PXP_SetProcessSurfaceBufferConfig(PXP, &psBufferConfig);
        PXP_SetProcessSurfacePosition(PXP, 0, 0, dst->w - 1, dst->h - 1);
        
        /* AS config. */
        asBufferConfig.pixelFormat = as_pixel_format;
        asBufferConfig.bufferAddr = (uint32_t)as_addr;
        asBufferConfig.pitchBytes = img->w * as_pixsize;
        PXP_SetAlphaSurfaceBufferConfig(PXP, &asBufferConfig);    
        
        asBlendConfig.invertAlpha = false;         /* Don't care. */
        asBlendConfig.alpha       = global_alpha;
        asBlendConfig.alphaMode   = kPXP_AlphaMultiply;
        asBlendConfig.ropMode     = 0;             /* Don't care. */
        PXP_SetAlphaSurfaceBlendConfig(PXP, &asBlendConfig);
        PXP_SetAlphaSurfacePosition(PXP, 0, 0, dst->w - 1, dst->h - 1);
    
    
    }
    
    if (img->format == BITMAP_FMT_BGR565) {
        as_pixsize = 2;
        as_pixel_format = kPXP_AsPixelFormatRGB565;

    } else {
        as_pixsize = 4;
        as_pixel_format = kPXP_AsPixelFormatARGB8888;
    }

    if (fb->format == BITMAP_FMT_BGR565) {
        ps_pixsize = 2;
        ps_pixel_format = kPXP_PsPixelFormatRGB565;
        out_pixsize = 2;
        output_pixel_format = kPXP_OutputPixelFormatRGB565;
    } else {
        ps_pixsize = 4;
        ps_pixel_format = kPXP_PsPixelFormatRGB888;
        out_pixsize = 4;
        output_pixel_format = kPXP_OutputPixelFormatRGB888;
    }
    
    /* calculate output buffer start addr */
    out_addr = (uint32_t)fb_data + (fb->w * dst->y + dst->x) * out_pixsize;
    fb_flush_size = ((dst->h - 1) * fb->w + dst->w) * out_pixsize;
    L1CACHE_CleanDCacheByRange(out_addr, fb_flush_size);
    /* calculate AS start addr */
    as_addr  = ((uint32_t)img_data + as_pixsize * (img->w * src->y + src->x));
    img_flush_size = ((src->h - 1) * img->w + src->w) * as_pixsize;
    L1CACHE_CleanDCacheByRange(as_addr, img_flush_size);
    
    PXP_Reset(PXP);
    /* Disable CSC1, it is enabled by default. */
    PXP_EnableCsc1(PXP, false);
    
    /* PS configure. */
    psBufferConfig.bufferAddr = out_addr;    /* PS is Source FrameBuffer Rect */
    psBufferConfig.bufferAddrU = 0;
    psBufferConfig.bufferAddrV = 0;
    psBufferConfig.pitchBytes = fb->w * out_pixsize;
    psBufferConfig.pixelFormat = ps_pixel_format;
    PXP_SetProcessSurfaceBufferConfig(PXP, &psBufferConfig);
    PXP_SetProcessSurfacePosition(PXP, 0, 0, dst->w - 1, dst->h - 1);
    
    /* AS config. */
    asBufferConfig.pixelFormat = as_pixel_format;
    asBufferConfig.bufferAddr = (uint32_t)as_addr;
    asBufferConfig.pitchBytes = img->w * as_pixsize;
    PXP_SetAlphaSurfaceBufferConfig(PXP, &asBufferConfig);    
    
    asBlendConfig.invertAlpha = false;         /* Don't care. */
    asBlendConfig.alpha       = global_alpha;
    asBlendConfig.alphaMode   = kPXP_AlphaMultiply;
    asBlendConfig.ropMode     = 0;             /* Don't care. */
    PXP_SetAlphaSurfaceBlendConfig(PXP, &asBlendConfig);
    PXP_SetAlphaSurfacePosition(PXP, 0, 0, dst->w - 1, dst->h - 1);
    
    /* Output config. */
    outputBufferConfig.pixelFormat    = output_pixel_format;
    outputBufferConfig.interlacedMode = kPXP_OutputProgressive;
    outputBufferConfig.buffer0Addr    = (uint32_t)out_addr;
    outputBufferConfig.buffer1Addr    = 0;
    outputBufferConfig.pitchBytes     = fb->w * out_pixsize;
    outputBufferConfig.width          = dst->w;
    outputBufferConfig.height         = dst->h;

    PXP_SetOutputBufferConfig(PXP, &outputBufferConfig);

    /* Start PXP. */
    PXP_Start(PXP);
    /* Wait for process complete. */
    while (!(kPXP_CompleteFlag & PXP_GetStatusFlags(PXP)))
    {
    }
    PXP_ClearStatusFlags(PXP, kPXP_CompleteFlag);
    L1CACHE_InvalidateDCacheByRange(out_addr, fb_flush_size);
    bitmap_unlock_buffer(fb);
    bitmap_unlock_buffer(img);
    
    return RET_OK;
}





/*
 * endfile
 */
