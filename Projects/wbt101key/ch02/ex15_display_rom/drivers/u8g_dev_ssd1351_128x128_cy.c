/*

  u8g_dev_ssd1351_128x128.c

  Universal 8bit Graphics Library
  
  Copyright (c) 2013, jamjardavies@gmail.com
  Copyright (c) 2013, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification, 
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list 
    of conditions and the following disclaimer.
    
  * Redistributions in binary form must reproduce the above copyright notice, this 
    list of conditions and the following disclaimer in the documentation and/or other 
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  
  
  History:
  Initial version	20 May 2013 jamjardavies@gmail.com	
  indexed device	22 May 2013 olikraus@gmail.com
  
*/

#include "u8g.h"

#define WIDTH		128
#define HEIGHT		128
#define PAGE_HEIGHT	 8  // for 65k colors version
#define USE_GPIO_FOR_SLEEP // alternative is a controller command which is less efficient

const uint8_t u8g_dev_ssd1351_128x128_init_seq_65k[] PROGMEM = {
	U8G_ESC_CS(0),					/* enable chip */
//	U8G_ESC_DLY(50),
	U8G_ESC_ADR(0),					/* instruction mode */
	U8G_ESC_RST(1),					/* do reset low pulse with (1*16)+2 microseconds */
	U8G_ESC_CS(1),					/* enable chip */
//	U8G_ESC_DLY(50),

	0xfd,							/* Command Lock */
	U8G_ESC_ADR(1),
	0x12,						  /* UNLOCK_NORMAL_CMD */

	U8G_ESC_ADR(0),					/* instruction mode */
	0xfd,
	U8G_ESC_ADR(1),
	0xb1,							/* CFG_UNLOCK_SPECIAL_CMD */

	U8G_ESC_ADR(0),					/* instruction mode */
	0xae,							/* Set Display Off */

	U8G_ESC_ADR(0),					/* instruction mode */
	0xb3,                          /* SSD1351_CMD_FREQDIV */
	U8G_ESC_ADR(1),
	0xf1,							/* Front Clock Div 2 freq= 0xf */
	U8G_ESC_ADR(0),					/* instruction mode */
	0xca,                          /* CMD_MUXRATIO */
	U8G_ESC_ADR(1),
	0x7f,							/* Set Multiplex Ratio to 127*/

	U8G_ESC_ADR(0),					/* instruction mode */
	0xa0,                           /* CMD_MISCCFG */
	U8G_ESC_ADR(1),
	0x30, 							/* 0x30 -> 2 bytes color Set Colour Depth */

	U8G_ESC_ADR(0),					/* instruction mode */
	0xa1,                          /* CMD_DSTART */
	U8G_ESC_ADR(1),
	0x00,							/* Set Display Start Line */

	U8G_ESC_ADR(0),					/* instruction mode */
	0xa2,                          /* CMD_DOFFSET */
	U8G_ESC_ADR(1),
	0x00,							/* Set Display Offset */

	U8G_ESC_ADR(0),					/* instruction mode */
	0xab,                           /* CMD_FUNCSEL */
	U8G_ESC_ADR(1),
	0x01,							/* Set Function Selection */

	U8G_ESC_ADR(0),					/* instruction mode */
	0xb1,                           /* CMD_RESETCFG */
	U8G_ESC_ADR(1),
	0x32,							/* Set Phase Length not helping for mura */

 	U8G_ESC_ADR(0),					/* instruction mode */
	0xb4,                           /* CMD_LOWVOLTAGE */
	U8G_ESC_ADR(1),
	0xa0, 0xb5, 0x55,				/* Set Segment Low Voltage */

	U8G_ESC_ADR(0),					/* instruction mode */
	0xbb,                         
	U8G_ESC_ADR(1),
	0x17,							/* Set Precharge Voltage */

	U8G_ESC_ADR(0),					/* instruction mode */
	0xbe,
	U8G_ESC_ADR(1),
	0x05,							/* Set VComH Voltage */

	U8G_ESC_ADR(0),					/* instruction mode */
	0xc1,                          
	U8G_ESC_ADR(1),
	0xc8, 0x80, 0xc8,				/* Set Contrast not helping for mura*/

	U8G_ESC_ADR(0),					/* instruction mode */
	0xc7,
	U8G_ESC_ADR(1),
    0x0C,//0xC OK, 9 too dark; default was 0xf /* Set Master Contrast */

	U8G_ESC_ADR(0),					/* instruction mode */
	0xb6,
	U8G_ESC_ADR(1),
	0x01,							/* Set Second Precharge Period */

    U8G_ESC_ADR(0),					/* instruction mode */
	0xb5,
	U8G_ESC_ADR(1),
	0x03,							/* Set GPIO to High Level */

	U8G_ESC_ADR(0),					/* instruction mode */
	0xa6,							/* Set Display Mode Reset */


	U8G_ESC_ADR(0),					/* instruction mode */
	0xb8,							/* Set CMD Grayscale Lookup */
	U8G_ESC_ADR(1),
	0x05,
	0x06,
	0x07,
	0x08,
	0x09,
	0x0a,
	0x0b,
	0x0c,
	0x0D,
	0x0E,
	0x0F,
	0x10,
	0x11,
	0x12,
	0x13,
	0x14,
	0x15,
	0x16,
	0x18,
	0x1a,
	0x1b,
	0x1C,
	0x1D,
	0x1F,
	0x21,
	0x23,
	0x25,
	0x27,
	0x2A,
	0x2D,
	0x30,
	0x33,
	0x36,
	0x39,
	0x3C,
	0x3F,
	0x42,
	0x45,
	0x48,
	0x4C,
	0x50,
	0x54,
	0x58,
	0x5C,
	0x60,
	0x64,
	0x68,
	0x6C,
	0x70,
	0x74,
	0x78,
	0x7D,
	0x82,
	0x87,
	0x8C,
	0x91,
	0x96,
	0x9B,
	0xA0,
	0xA5,
	0xAA,
	0xAF,
	0xB4,

    U8G_ESC_ADR(0),
	0xaf,							/* Set Display On */
	0x5c,
//	U8G_ESC_DLY(50),
	U8G_ESC_CS(0),					/* disable chip */
	U8G_ESC_ADR(1),
	U8G_ESC_END						/* end of sequence */
};


uint8_t u8g_dev_ssd1351_128x128_column_seq[] PROGMEM = {
	U8G_ESC_CS(1),
	U8G_ESC_ADR(0), 0x15,
	U8G_ESC_ADR(1), 0x00, 0x7f,
	U8G_ESC_ADR(0), 0x75,
	U8G_ESC_ADR(1), 0x00, 0x7f,
	U8G_ESC_ADR(0), 0x5c,
	U8G_ESC_ADR(1),
	U8G_ESC_CS(0),
	U8G_ESC_END
};

static unsigned char *fifo_ptr= NULL;
uint8_t u8g_dev_ssd1351_128x128_hicolor_65k_fn(u8g_t *u8g, u8g_dev_t *dev, uint8_t msg, void *arg)
{
    static unsigned char page_change=0;
    u8g_pb_t *pb = (u8g_pb_t *)(dev->dev_mem);
  switch(msg)
  {
    case U8G_DEV_MSG_INIT:
#ifdef FPGA_BD_2045
           // FPGA with cables cannot handle 8MHz
       dev->com_fn(u8g, U8G_COM_MSG_INIT, U8G_SPI_CLK_CYCLE_1MHZ, &fifo_ptr);
#else
       dev->com_fn(u8g, U8G_COM_MSG_INIT, U8G_SPI_CLK_CYCLE_8MHZ, &fifo_ptr);
#endif
       u8g_WriteEscSeqP(u8g, dev, u8g_dev_ssd1351_128x128_init_seq_65k);
       break;
    case U8G_DEV_MSG_STOP:
      break;
    case U8G_DEV_MSG_CONTRAST:
      if(!(*(uint8_t*)arg)) // contrast 0 -> switch off display using GPIO for Freetronic
      {
          u8g_SetAddress(u8g, dev, 0);          /* instruction mode */
#ifdef USE_GPIO_FOR_SLEEP
          u8g_WriteByte(u8g, dev, 0xB5);        /* Set GPIO Level */
          u8g_SetAddress(u8g, dev, 1);          /* data mode */
          u8g_WriteByte(u8g, dev, 0x00);        /* Low Level */
#else
          u8g_WriteByte(u8g, dev, 0xAE);        /* Set Display OFF */
#endif
      }
      else
      {
          u8g_SetAddress(u8g, dev, 0);          /* instruction mode */
#ifdef USE_GPIO_FOR_SLEEP
          u8g_WriteByte(u8g, dev, 0xB5);        /* Set GPIO Level */
          u8g_SetAddress(u8g, dev, 1);          /* data mode */
          u8g_WriteByte(u8g, dev, 0x03);        /* High Level */
          u8g_SetAddress(u8g, dev, 0);          /* instruction mode */
#else
          u8g_WriteByte(u8g, dev, 0xAF);        /* Set Display ON */
#endif
          u8g_WriteByte(u8g, dev, 0xC7);        /* set main contrast */
          u8g_SetAddress(u8g, dev, 1);          /* data mode */
          /* arg: pointer to uint8_t, contrast value between 0 and 255 */
          u8g_WriteByte(u8g, dev, *(uint8_t*)arg);
      }
      break;
      case U8G_DEV_MSG_SLEEP_ON:
          u8g_SetAddress(u8g, dev, 0);          /* instruction mode */
#ifdef USE_GPIO_FOR_SLEEP
          // 3 lines below disable chip completely using GPIO
          // this feature is supported only on Freetronic board
          u8g_WriteByte(u8g, dev, 0xB5);        /* Set GPIO Level */
          u8g_SetAddress(u8g, dev, 1);          /* data mode */
          u8g_WriteByte(u8g, dev, 0x00);        /* Low Level */
#else
          // this will work for any board with 1351 controller
          u8g_WriteByte(u8g, dev, 0xAE);        /* Set Display OFF */
#endif
          break;
      case U8G_DEV_MSG_SLEEP_OFF:
          u8g_SetAddress(u8g, dev, 0);          /* instruction mode */
#ifdef USE_GPIO_FOR_SLEEP
          // 3 lines below enable chip if it was idsbaled using GPIO using GPIO
          u8g_WriteByte(u8g, dev, 0xB5);        /* Set GPIO Level */
          u8g_SetAddress(u8g, dev, 1);          /* data mode */
          u8g_WriteByte(u8g, dev, 0x03);        /* High Level */
#else
          u8g_WriteByte(u8g, dev, 0xAF);        /* Set Display ON */
#endif
          break;
    case U8G_DEV_MSG_PAGE_FIRST:
      u8g_dev_ssd1351_128x128_column_seq[14]= 0;
      page_change=1;
      break;
    case U8G_DEV_MSG_SET_PAGE:
    {
        register u8g_uint_t y1;
        u8g_page_t * p= &(((u8g_pb_t *)(u8g->dev->dev_mem))->p);

        if ( ((u8g_box_t *)arg)->y0 >= p->total_height )
          return 0;
        p->page_y0 = ((u8g_box_t *)arg)->y0;
        p->page++; // this should not metter
        y1 = p->page_y0 + (p->page_height-1);
        if ( y1 >= p->total_height )
        {
          y1 = p->total_height;
          y1--;
        }
        p->page_y1 = y1;

        // modify initiation sequence to set new page.
        u8g_dev_ssd1351_128x128_column_seq[14]= pb->p.page_y0;
        page_change=1;
        msg= U8G_DEV_MSG_GET_PAGE_BOX; // for underlying driver
        arg= &(u8g->current_page);
        break;
    }
    case U8G_DEV_MSG_PAGE_NEXT:
      {
#define BURST_TO_SCREEN
#ifndef BURST_TO_SCREEN
        uint8_t i, j;
#endif
        uint8_t page_height;
        uint8_t *ptr = pb->buf;

        u8g_SetChipSelect(u8g, dev, 1);

        page_height = pb->p.page_y1;
        page_height -= pb->p.page_y0;
        page_height++;
#ifdef BURST_TO_SCREEN

        {
//#define DISP_SWAP
#ifdef DISP_SWAP
            int i;
            mipi_dbic_wait_for_done();
            for (i=0; i< page_height*pb->width; i++)
            {
             //swapping in FIFO requires resetting before and after and overall takes more time
                *fifo_ptr= *(ptr+1);
                *fifo_ptr=*ptr;
                ptr+=2;
            }
            //cannot call library function as it has lenght parameter as byte
            dev->com_fn(u8g, U8G_COM_MSG_WRITE_SEQ, (int)page_height*pb->width*2, NULL);
#else
            if(page_change)
            {
                u8g_WriteEscSeqP(u8g, dev, u8g_dev_ssd1351_128x128_column_seq);
                page_change=0;
            }
            //cannot call library function as it has lenght parameter as byte
            dev->com_fn(u8g, U8G_COM_MSG_WRITE_SEQ, (int)page_height*pb->width*2,ptr);
#endif
        }
#else
        for( j = 0; j < page_height; j++ )
        {
    	    for (i = 0; i < pb->width; i+=RGB332_STREAM_BYTES)
    	    {
              // bytes have to be swapped
    	      u8g_WriteSequence(u8g, dev, RGB332_STREAM_BYTES*2, ptr);
    	      ptr += RGB332_STREAM_BYTES*2;
    	    }
        }
#endif	  

        u8g_SetChipSelect(u8g, dev, 0);
        // to remove screen clearing uncomment next line and comment break; below
        // our fill bgrnd procdure runs 22us (48MHZ CPU clock) instead of u8glib 300us (48MHz)
        // we still have to wait for transmission to finish
	   return   u8g_page_Next(&(pb->p)); 
        //break;    /* continue to base fn */
      }
    case U8G_DEV_MSG_GET_MODE:
     return U8G_MODE_HICOLOR;
  }
  return u8g_dev_pbxh16_base_fn(u8g, dev, msg, arg);
}

// next buffer has to be aligned !!!
uint8_t u8g_dev_ssd1351_128x128_byte_buf[WIDTH*PAGE_HEIGHT] U8G_NOCOMMON __attribute__((aligned(2)));

/* only half of the height, because two bytes are needed for one pixel */
u8g_pb_t u8g_dev_ssd1351_128x128_hicolor_byte_pb = { {PAGE_HEIGHT/2, HEIGHT, 0, 0, 0},  WIDTH, u8g_dev_ssd1351_128x128_byte_buf}; 
u8g_dev_t u8g_dev_ssd1351_128x128_hicolor_65k_hw_spi = { u8g_dev_ssd1351_128x128_hicolor_65k_fn, &u8g_dev_ssd1351_128x128_hicolor_byte_pb, U8G_COM_HW_SPI };
