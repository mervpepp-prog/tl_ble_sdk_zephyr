/********************************************************************************************************
 * @file    tl_fifo.c
 *
 * @brief   This is the source file for Telink RISC-V MCU
 *
 * @author  Driver Group
 * @date    2025
 *
 * @par     Copyright (c) 2025, Telink Semiconductor (Shanghai) Co., Ltd. ("TELINK")
 *
 *          Licensed under the Apache License, Version 2.0 (the "License");
 *          you may not use this file except in compliance with the License.
 *          You may obtain a copy of the License at
 *
 *              http://www.apache.org/licenses/LICENSE-2.0
 *
 *          Unless required by applicable law or agreed to in writing, software
 *          distributed under the License is distributed on an "AS IS" BASIS,
 *          WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *          See the License for the specific language governing permissions and
 *          limitations under the License.
 *
 *******************************************************************************************************/
#include "tl_string.h"
#include "tl_fifo.h"

_attribute_ram_code_sec_ void pp_fifo_reset (pl_fifo_t *f)
 {
     f->wptr = 0;
     f->rptr = 0;
 }
 _attribute_ram_code_sec_ int pp_fifo_push(pl_fifo_t *f, unsigned char cmd, unsigned char *buf, unsigned char len)
 {
     if (len > (f->size - 2))
     {
         return 1;
     }
     if (((f->wptr - f->rptr) & 255) < (f->num - 1))
     {

         unsigned char *pd = (unsigned char *)(f->p + (f->wptr & (f->num - 1)) * f->size);

         pd[0] = len;
         pd[1] = cmd;
         tmemcpy(&pd[2], &buf[0], len);
         f->wptr++;
         return 0;
     }
     return 2;
 }

 _attribute_ram_code_sec_ int pp_fifo_push_extra(pl_fifo_t *f, unsigned char cmd, unsigned char *buf, unsigned char len,
                                                 uint8_t retry_num, fifo_cb_t cb, void *user_arg)
 {
     if (len > (f->size - 12))
     {
         return 1;
     }
     if (((f->wptr - f->rptr) & 255) < (f->num - 1))
     {

         unsigned char *pd = (unsigned char *)(f->p + (f->wptr & (f->num - 1)) * f->size);

         pd[0] = len;
         pd[1] = cmd;
         tmemcpy(&pd[2], &buf[0], len);

         *(fifo_cb_t *)(&pd[len + 2]) = cb;
         *(void **)(&pd[len + 6]) = user_arg;
         pd[len + 10] = retry_num;

         f->wptr++;
         return 0;
     }
     return 2;
 }

_attribute_ram_code_sec_ unsigned char *pp_fifo_get_ptr (pl_fifo_t *f)
{
    if (f->rptr != f->wptr)
    {
        unsigned char *p = f->p + (f->rptr & (f->num-1)) * f->size;
        return p;
    }
    return 0;
}


_attribute_ram_code_sec_ unsigned short pp_fifo_get_num(pl_fifo_t *f)
 {
    unsigned short num =(f->wptr - f->rptr) & 255;
   return num;
 }

_attribute_ram_code_sec_ void pp_fifo_pop(pl_fifo_t *f)
 {
   f->rptr++;
 }

_attribute_ram_code_sec_ bool pp_fifo_is_empty(const pl_fifo_t *fifo)
{
    return (fifo->wptr == fifo->rptr);
}

_attribute_ram_code_sec_ bool pp_fifo_is_full(const pl_fifo_t *fifo)
{
    return (((fifo->wptr - fifo->rptr) & 255) >= (fifo->num - 1));
}

_attribute_ram_code_sec_ unsigned short pp_fifo_get_remaining_slots(pl_fifo_t *f)
{
    return ((f->num - 1) - ((f->wptr - f->rptr) & 255));
}

