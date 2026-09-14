/********************************************************************************************************
 * @file    d24g_stack_common.c
 *
 * @brief   Common 2.4G stack helper functions shared across dongle, mouse, and keyboard.
 *
 * @author  2.4G Group
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

#include "d24g_stack_common.h"

_attribute_ram_code_sec_ uint32_t fnv1a_32_hash(const uint8_t *data, size_t len)
{
    uint32_t hash = 0xD21C5AC5;

    for (size_t i = 0; i < len; i++) {
        hash ^= data[i];
        hash = hash + (hash << 24) + (hash << 8) + (hash << 5);
        hash ^= (hash >> 16);//Avalanche Effect
    }

    return hash;
}

#if TPSLL_RECON_RSSI_SCAN_EN
void rf_rssi_scan_param_set(void)
{
    rf_access_code_comm(0x1a2b3c4d);
    write_reg8(REG_RXCHN_EN, 0);
    write_reg8(0x17051a,40);//sync thr
    rf_clr_irq_mask(FLD_RF_IRQ_RX);
    rf_set_irq_mask(FLD_RF_IRQ_FIRST_TIMEOUT);
}

void tpsll_bubble_sort_rssi_index(uint16_t *chn_rssi, uint8_t num, uint8_t *sorted_idx) {
    for (uint8_t i = 0; i < num; i++) {
        sorted_idx[i] = i;
    }

    for (uint8_t i = 0; i < num - 1; i++) {
        for (uint8_t j = 0; j < num - 1 - i; j++) {
            if (chn_rssi[sorted_idx[j]] > chn_rssi[sorted_idx[j + 1]]) {
                uint8_t temp = sorted_idx[j];
                sorted_idx[j] = sorted_idx[j + 1];
                sorted_idx[j + 1] = temp;
            }
        }
    }
}
#endif
 

_attribute_ram_code_sec_ uint8_t wait_rf_fsm_idle_timeout(unsigned int timeout_us)
{
   unsigned int tick = stimer_get_tick();

    while(0 != (read_reg8(0x170224) & 0x7))
    {
        if(clock_time_exceed(tick, timeout_us)) {
            return 1;
        }
    }

    return 0;
}

unsigned char * pp_rf_tx_fifo_get_ptr (pl_fifo_t *f)
{
    if (f->rptr != f->wptr)
    {
        unsigned char *p = f->p + (f->rptr & (f->num-1)) * f->size;
        return p;
    }
    return 0;
}


void tpsll_gen_reconn_chn(uint8_t a, uint8_t out[RECONN_CHN_NUM])
{
    uint8_t pool[RECONN_MAX_CH];
    uint8_t rnd = a ^ 0xA5;

    for(uint8_t i = 0; i < RECONN_MAX_CH; i++)
        pool[i] = i;

    for(uint8_t i = 0; i < RECONN_CHN_NUM; i++)
    {
        uint8_t r = i + (prng8(&rnd) % (RECONN_MAX_CH - i));

        uint8_t tmp = pool[i];
        pool[i] = pool[r];
        pool[r] = tmp;

        out[i] = pool[i];
    }
}

#if TPSLL_N8K_CHN_EXPAND_EN
void tpsll_generate_deterministic_ble_channels(const uint8_t *existing_chs, uint8_t *new_chs)
{
    // 1. Generate a base seed using a simple hash of the 3 existing channels
    uint32_t seed = (existing_chs[0] * 131) ^ (existing_chs[1] * 137) ^ (existing_chs[2] * 139);

    uint64_t ch_mask = 0;
    uint8_t generated_count = 0;

    // 2. Mark the existing channels and their adjacent channels as occupied in the bitmask
    //    This ensures a minimum spacing of 2 (i.e., |a - b| >= 2)
    for (uint8_t i = 0; i < 3; i++) {
        uint8_t ch = existing_chs[i];
        ch_mask |= (1ULL << ch); // Mark the channel itself

        // Mark the adjacent channels to enforce the minimum spacing constraint
        if (ch > 0) {
            ch_mask |= (1ULL << (ch - 1));
        }
        if (ch < 39) {
            ch_mask |= (1ULL << (ch + 1));
        }
    }

    // 3. Loop to generate new channels until 3 unique and sufficiently spaced channels are found
    while (generated_count < 3) {
        // Update the seed using a Linear Congruential Generator (LCG) PRNG algorithm
        seed = (seed * 1103515245 + 12345) & 0x7FFFFFFF;

        // Extract the new channel number from the higher bits to avoid short-period cycles
        uint8_t rand_ch = (seed >> 16) % 40;

        // Check if the generated channel (and its adjacent slots) are free
        if (!(ch_mask & (1ULL << rand_ch))) {
            new_chs[generated_count] = rand_ch;

            // Mark the newly generated channel and its adjacent channels as occupied
            ch_mask |= (1ULL << rand_ch);
            if (rand_ch > 0) {
                ch_mask |= (1ULL << (rand_ch - 1));
            }
            if (rand_ch < 39) {
                ch_mask |= (1ULL << (rand_ch + 1));
            }

            generated_count++;
        }
    }
}
#endif


