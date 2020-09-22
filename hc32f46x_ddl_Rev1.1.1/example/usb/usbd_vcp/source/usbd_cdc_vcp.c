/******************************************************************************
 * Copyright (C) 2016, Huada Semiconductor Co.,Ltd. All rights reserved.
 *
 * This software is owned and published by:
 * Huada Semiconductor Co.,Ltd ("HDSC").
 *
 * BY DOWNLOADING, INSTALLING OR USING THIS SOFTWARE, YOU AGREE TO BE BOUND
 * BY ALL THE TERMS AND CONDITIONS OF THIS AGREEMENT.
 *
 * This software contains source code for use with HDSC
 * components. This software is licensed by HDSC to be adapted only
 * for use in systems utilizing HDSC components. HDSC shall not be
 * responsible for misuse or illegal use of this software for devices not
 * supported herein. HDSC is providing this software "AS IS" and will
 * not be responsible for issues arising from incorrect user implementation
 * of the software.
 *
 * Disclaimer:
 * HDSC MAKES NO WARRANTY, EXPRESS OR IMPLIED, ARISING BY LAW OR OTHERWISE,
 * REGARDING THE SOFTWARE (INCLUDING ANY ACCOMPANYING WRITTEN MATERIALS),
 * ITS PERFORMANCE OR SUITABILITY FOR YOUR INTENDED USE, INCLUDING,
 * WITHOUT LIMITATION, THE IMPLIED WARRANTY OF MERCHANTABILITY, THE IMPLIED
 * WARRANTY OF FITNESS FOR A PARTICULAR PURPOSE OR USE, AND THE IMPLIED
 * WARRANTY OF NONINFRINGEMENT.
 * HDSC SHALL HAVE NO LIABILITY (WHETHER IN CONTRACT, WARRANTY, TORT,
 * NEGLIGENCE OR OTHERWISE) FOR ANY DAMAGES WHATSOEVER (INCLUDING, WITHOUT
 * LIMITATION, DAMAGES FOR LOSS OF BUSINESS PROFITS, BUSINESS INTERRUPTION,
 * LOSS OF BUSINESS INFORMATION, OR OTHER PECUNIARY LOSS) ARISING FROM USE OR
 * INABILITY TO USE THE SOFTWARE, INCLUDING, WITHOUT LIMITATION, ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL OR CONSEQUENTIAL DAMAGES OR LOSS OF DATA,
 * SAVINGS OR PROFITS,
 * EVEN IF Disclaimer HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 * YOU ASSUME ALL RESPONSIBILITIES FOR SELECTION OF THE SOFTWARE TO ACHIEVE YOUR
 * INTENDED RESULTS, AND FOR THE INSTALLATION OF, USE OF, AND RESULTS OBTAINED
 * FROM, THE SOFTWARE.
 *
 * This software may be replicated in part or whole for the licensed use,
 * with the restriction that this Disclaimer and Copyright notice must be
 * included with each copy of this software, whether used in part or whole,
 * at all times.
 */
/******************************************************************************/
/** \file usbd_cdc_vcp.c
 **
 ** \brief  Generic media access Layer.
 **
 **   - 2019-6-3  1.0  zhangxl First version for USB CDC VCP demo.
 **
 ******************************************************************************/
#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
#pragma     data_alignment = 4
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "usbd_cdc_vcp.h"
#include "usb_conf.h"

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/
LINE_CODING linecoding =
  {
    115200, /* baud rate*/
    0x00,   /* stop bits-1*/
    0x00,   /* parity - none*/
    0x08    /* nb. of bits 8*/
  };

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/
/* These are external variables imported from CDC core to be used for IN
   transfer management. */
extern uint8_t  APP_Rx_Buffer [APP_RX_DATA_SIZE]; /* Write CDC received data in this buffer.
                                     These data will be sent over USB IN endpoint
                                     in the CDC core functions. */
extern uint32_t APP_Rx_ptr_in;    /* Increment this pointer or roll it back to
                                     start address when writing received data
                                     in the buffer APP_Rx_Buffer. */
extern uint32_t APP_Rx_length;

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static uint16_t VCP_Init     (void);
static uint16_t VCP_DeInit   (void);
static uint16_t VCP_Ctrl     (uint32_t Cmd, uint8_t* Buf, uint32_t Len);
static uint16_t CDC_Receive_FS   (uint8_t* Buf, uint32_t Len);

CDC_IF_Prop_TypeDef VCP_fops =
{
    &VCP_Init,
    &VCP_DeInit,
    &VCP_Ctrl,
    NULL,//&CDC_Transmit_FS,
    &CDC_Receive_FS
};

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
  * @brief  VCP_Init
  *         Initializes the Media
  * @param  None
  * @retval Result of the opeartion (USBD_OK in all cases)
  */
static uint16_t VCP_Init(void)
{
    return USBD_OK;
}

/**
  * @brief  VCP_DeInit
  *         DeInitializes the Media
  * @param  None
  * @retval Result of the opeartion (USBD_OK in all cases)
  */
static uint16_t VCP_DeInit(void)
{
    return USBD_OK;
}


/**
  * @brief  VCP_Ctrl
  *         Manage the CDC class requests
  * @param  Cmd: Command code
  * @param  Buf: Buffer containing command data (request parameters)
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the opeartion (USBD_OK in all cases)
  */
static uint16_t VCP_Ctrl (uint32_t Cmd, uint8_t* Buf, uint32_t Len)
{
    switch (Cmd)
    {
        case SEND_ENCAPSULATED_COMMAND:
        /* Not  needed for this driver */
        break;

        case GET_ENCAPSULATED_RESPONSE:
        /* Not  needed for this driver */
        break;

        case SET_COMM_FEATURE:
        /* Not  needed for this driver */
        break;

        case GET_COMM_FEATURE:
        /* Not  needed for this driver */
        break;

        case CLEAR_COMM_FEATURE:
        /* Not  needed for this driver */
        break;

        case SET_LINE_CODING:
            linecoding.bitrate = ((uint32_t)Buf[0] | ((uint32_t)Buf[1] << 8u) | ((uint32_t)Buf[2] << 16u) | ((uint32_t)Buf[3] << 24u));
            linecoding.format = Buf[4];
            linecoding.paritytype = Buf[5];
            linecoding.datatype = Buf[6];
            /* Set the new configuration */
            //VCP_COMConfig();  /* MISRAC 2004*/
        break;
        case GET_LINE_CODING:
            Buf[0] = (uint8_t)(linecoding.bitrate);
            Buf[1] = (uint8_t)(linecoding.bitrate >> 8u);
            Buf[2] = (uint8_t)(linecoding.bitrate >> 16u);
            Buf[3] = (uint8_t)(linecoding.bitrate >> 24u);
            Buf[4] = linecoding.format;
            Buf[5] = linecoding.paritytype;
            Buf[6] = linecoding.datatype;
        break;
        case SET_CONTROL_LINE_STATE:
        /* Not  needed for this driver */
        break;
        case SEND_BREAK:
        /* Not  needed for this driver */
        break;
        default:
        break;
    }
    return USBD_OK;
}

/**
  * @brief  CDC_Transmit_FS
  *         CDC received data to be send over USB IN endpoint are managed in
  *         this function.
  * @param  pBuf: Buffer of data to be transmit
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the opeartion: USBD_OK if all operations are OK else VCP_FAIL
  */
uint16_t CDC_Transmit_FS (uint8_t* pBuf, uint32_t Len)
{
    uint32_t u32Ret = USBD_BUSY;

    /* If data transmit buffer enough */
    if((APP_RX_DATA_SIZE - APP_Rx_length) > Len)
    {
        for(uint32_t i=0ul; i<Len; i++)
        {
            APP_Rx_Buffer[APP_Rx_ptr_in] = *pBuf++;
            APP_Rx_ptr_in++;

            /* To avoid buffer overflow */
            if(APP_Rx_ptr_in == APP_RX_DATA_SIZE)
            {
                APP_Rx_ptr_in = 0u;
            }
        }

        u32Ret = USBD_OK;
    }
    return u32Ret;
}

/**
  * @brief  CDC_Receive_FS
  *         Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will block any OUT packet reception on USB endpoint
  *         untill exiting this function. If you exit this function before transfer
  *         is complete on CDC interface (ie. using DMA controller) it will result
  *         in receiving more data while previous ones are still not sent.
  *
  * @param  pBuf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the opeartion: USBD_OK if all operations are OK else VCP_FAIL
  */
static uint16_t CDC_Receive_FS (uint8_t* pBuf, uint32_t Len)
{
    /* Data analysis start */
    CDC_Transmit_FS (pBuf, Len);  //for test
    /* Data analysis end */
    return USBD_OK;
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
