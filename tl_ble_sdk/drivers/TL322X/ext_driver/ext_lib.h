/********************************************************************************************************
 * @file    ext_lib.h
 *
 * @brief   This is the header file for BLE SDK
 *
 * @author  BLE GROUP
 * @date    06,2022
 *
 * @par     Copyright (c) 2022, Telink Semiconductor (Shanghai) Co., Ltd.
 *          All rights reserved.
 *
 *          The information contained herein is confidential property of Telink
 *          Semiconductor (Shanghai) Co., Ltd. and is available under the terms
 *          of Commercial License Agreement between Telink Semiconductor (Shanghai)
 *          Co., Ltd. and the licensee or the terms described here-in. This heading
 *          MUST NOT be removed from this file.
 *
 *          Licensee shall not delete, modify or alter (or permit any third party to delete, modify, or
 *          alter) any information contained herein in whole or in part except as expressly authorized
 *          by Telink semiconductor (shanghai) Co., Ltd. Otherwise, licensee shall be solely responsible
 *          for any claim to the extent arising out of or relating to such deletion(s), modification(s)
 *          or alteration(s).
 *
 *          Licensees are granted free, non-transferable use of the information in this
 *          file under Mutual Non-Disclosure Agreement. NO WARRANTY of ANY KIND is provided.
 *
 *******************************************************************************************************/
#ifndef DRIVERS_TL322X_EXT_DRIVER_EXT_LIB_H_
#define DRIVERS_TL322X_EXT_DRIVER_EXT_LIB_H_


#include "types.h"
#include "../uart.h"
#include "../lib/include/pm/pm.h"
#include "lib/include/rf/rf_common.h"
#include "../lib/include/clic.h"
#include <stdbool.h>
#include "ext_pm.h"





/******************************* dbgport start ******************************************************************/
#define reg_bb_dbg_sel      REG_ADDR16(0x140d00)
#define reg_bb_dbg_sel_l    REG_ADDR8(0x140d00)
#define reg_bb_dbg_sel_h    REG_ADDR8(0x140d01)
#define bt_dbg_set_pin      dbg_bb_set_pin


void ble_dbg_port_init(int deg_sel0);

void dbg_bb_set_pin(gpio_pin_e pin);

void rf_enable_bb_debug(void);
/******************************* dbgport end ********************************************************************/


/******************************* dbgErrorCode start ******************************************************************/
#define PKE_OPERAND_MAX_WORD_LEN      (0x08)
#define PKE_OPERAND_MAX_BIT_LEN       (0x100)
#define ECC_MAX_WORD_LEN              PKE_OPERAND_MAX_WORD_LEN
/******************************* dbgErrorCode end ********************************************************************/


/******************************* ext_aes start ******************************************************************/
#define HW_AES_CCM_ALG_EN                                       0

extern unsigned int aes_data_buff[8];


void aes_encryption_le(u8* key, u8* plaintext, u8 *encrypted_data);
void aes_encryption_be(u8* key, u8* plaintext, u8 *encrypted_data);

bool aes_resolve_irk_rpa(u8 *key, u8 *addr);


#define big_integer_compare         uint32_BigNumCmp
#define pke_eccp_point_mul          eccp_pointMul
#define pke_eccp_point_verify       eccp_pointVerify

#if defined(MCU_CORE_TL322X_N22)

#define ext_crypto_hw_init_enable()

#else

#define ext_crypto_hw_init_enable()   do{ \
pke_reset(); \
pke_clk_en(); \
ske_reset();  \
ske_clk_en();  \
}while(0);
#endif

/******************************* ext_aes end ********************************************************************/




/******************************* ext_stimer start ******************************************************************/
#define SYSTICK_NUM_PER_US              24

#define SSLOT_TICK_NUM                  1875/4    //attention: not use "()" for purpose !!!    625uS*24/32=625*3/4=1875/4=468.75
#define SSLOT_TICK_REVERSE              4/1875    //attention: not use "()" for purpose !!!



/**
 * @brief define system clock tick per us/ms/s.
 */

#define SYSTEM_TIMER_TICK_125US  (SYSTICK_NUM_PER_US*125)


/**
 * @brief    This function serves to enable system timer interrupt.
 * @return  none
 */
#ifdef MCU_CORE_N22_ENABLE
#define systimer_irq_enable()     clic_interrupt_enable(IRQ_SYSTIMER)
#else
#define systimer_irq_enable()     plic_interrupt_enable(IRQ_SYSTIMER)
#endif

/**
 * @brief    This function serves to disable system timer interrupt.
 * @return  none
 */
__INLINE void systimer_irq_disable(void)
{
#ifdef MCU_CORE_N22_ENABLE
    clic_interrupt_disable(IRQ_SYSTIMER);
#else
    plic_interrupt_disable(IRQ_SYSTIMER);
#endif
}

__INLINE void systimer_set_irq_mask(void)
{
#ifdef MCU_CORE_N22_ENABLE
    stimer_set_irq_mask_n22(FLD_SYSTEM_IRQ_MASK_N22);
#else
    stimer_set_irq_mask_d25f(FLD_SYSTEM_IRQ_MASK_D25F);
#endif
}

__INLINE void systimer_clr_irq_mask(void)
{
#ifdef MCU_CORE_N22_ENABLE
    reg_system_irq_mask_n22 &= (~FLD_SYSTEM_IRQ_MASK_N22);
#else
    reg_system_irq_mask_d25f &= (~FLD_SYSTEM_IRQ_MASK_D25F);
#endif
}

__INLINE unsigned char systimer_get_irq_status(void)
{
#ifdef MCU_CORE_N22_ENABLE
    return stimer_get_irq_status_n22();
#else
    return stimer_get_irq_status_d25f(FLD_SYSTEM_IRQ_D25F);
#endif
}

__INLINE void systimer_clr_irq_status(void)
{

#ifdef MCU_CORE_N22_ENABLE
    stimer_clr_irq_status_n22();
#else
    stimer_clr_irq_status_d25f(FLD_SYSTEM_IRQ_D25F);
#endif
}

__INLINE void systimer_set_irq_capture(unsigned int tick)
{
#ifdef MCU_CORE_N22_ENABLE
    stimer_set_irq_capture_n22(tick);
#else
    stimer_set_irq_capture_d25f(tick);
#endif
}

__INLINE unsigned int systimer_get_irq_capture(void)
{
#ifdef MCU_CORE_N22_ENABLE
    return reg_system_irq_level_n22;
#else
    return reg_system_irq_level_d25f;
#endif
}

static _always_inline int tick1_exceed_tick2(u32 tick1, u32 tick2)
{
    return (u32)(tick1 - tick2) < BIT(30);
}


static _always_inline int tick1_closed_to_tick2(unsigned int tick1, unsigned int tick2, unsigned int tick_distance)
{
    return (unsigned int)(tick1 + tick_distance - tick2) < (tick_distance<<1);
}

static _always_inline int tick1_out_range_of_tick2(unsigned int tick1, unsigned int tick2, unsigned int tick_distance)
{
    return (unsigned int)(tick1 + tick_distance - tick2) > (tick_distance<<1);
}

/******************************* ext_stimer end ********************************************************************/



/******************************* ext_pm start ******************************************************************/

#define WAKEUP_STATUS_TIMER_PAD             ( WAKEUP_STATUS_TIMER | WAKEUP_STATUS_PAD)

 #ifndef PM_32k_RC_CALIBRATION_ALGORITHM_EN
 #define PM_32k_RC_CALIBRATION_ALGORITHM_EN          0
 #endif

extern  unsigned char           tl_24mrc_cal;
extern  unsigned int            g_pm_mspi_cfg;

extern  unsigned int            g_sleep_32k_rc_cnt;
extern  unsigned int            g_sleep_stimer_tick;
extern  unsigned int            g_sleep_n22_tick;

extern unsigned int ota_program_bootAddr;
extern unsigned int ota_firmware_max_size;
extern unsigned int ota_program_offset;

/**
 * @brief   pm 32k rc calibration algorithm.
 */
typedef struct __attribute__((packed))   pm_clock_drift
{
    unsigned int    ref_tick;
    unsigned int    ref_tick_32k;
    unsigned int    ref_tick_enter;
    unsigned int    ref_tick_32k_enter;
    int             offset;
    int             offset_dc;
//  int             offset_cur;
    unsigned int    offset_cal_tick;
    int             tc;
    int             rc32;
    int             rc32_wakeup;
    int             rc32_rt;
    int             s0;
    unsigned char   calib;
    unsigned char   ref_no;
} pm_clock_drift_t;


extern pm_clock_drift_t pmbcd;

static inline unsigned int pm_ble_get_latest_offset_cal_time(void){        
    return pmbcd.offset_cal_tick;
}

static inline unsigned int pm_ble_get_latest_ref_tick(void){
    return pmbcd.ref_tick;
}

static inline unsigned int pm_ble_get_latest_ref_tick_32k(void){
    return pmbcd.ref_tick_32k;
}

/**
 * @brief       Calculate the offset value based on the difference of 16M tick.
 * @param[in]   offset_tick - the 16M tick difference between the standard clock and the expected clock.
 * @param[in]   rc32_cnt    - 32k count
 * @return      none.
 */
_attribute_ram_code_sec_noinline_ void pm_ble_cal_32k_rc_offset (int offset_tick, int rc32_cnt);

/**
 * @brief       This function reset calibrates the value
 * @param[in]   none
 * @return      none.
 */
extern void ext_pm_ble_32k_rc_cal_reset(void);
#define pm_ble_32k_rc_cal_reset  ext_pm_ble_32k_rc_cal_reset
#define PM_MIN_SLEEP_US         (2000)

/**
 * @brief   internal oscillator or crystal calibration for environment change such as voltage, temperature
 *          to keep some critical PM or RF performance stable
 *          attention: this is a stack API, user can not call it
 * @param   none
 * @return  none
 */
void mcu_oscillator_crystal_calibration(void);

typedef int (*suspend_handler_t)(void);
typedef void (*pm_tim_recover_handler_t)(void);

extern  suspend_handler_t           func_before_suspend;
extern  pm_tim_recover_handler_t    ext_pm_tim_recover;

#define DEFAULT_DEEPSLEEP_MODE_RET_SRAM_SIZE  DEEPSLEEP_MODE_RET_SRAM_LOW64K


/**
 * @brief   pm 32k rc calibration algorithm.
 */
typedef struct
{
    unsigned int g_sleep_32k_rc_cnt_addr;
    unsigned int g_sleep_stimer_tick_addr;
    unsigned int g_sleep_n22_sleep_tick_addr;
    unsigned int g_sleep_pm_status_addr;
    unsigned int pmbcd_addr;
    unsigned int host_core_status;   /*tlk_pm_mcuStatus_t*/
    unsigned int controller_core_status;  /*tlk_pm_mcuStatus_t,Not in use for the time being, but might be used later.*/
} pm_share_sleep_info_t;

extern volatile pm_share_sleep_info_t g_pm_share_sleep_info;

/******************************* ext_pm end ********************************************************************/


/******************************* mcu_security start ******************************************************************/
#define SECBOOT_DESC_SECTOR_NUM     1
#define SECBOOT_DESC_SIZE           0x1000  //4K for secure boot descriptor size

#define DESCRIPTOR_PUBKEY_OFFSET    0x0004

#define DESCRIPTOR_WATCHDOG_OFFSET  0x008C


#define DESC_SECTOR_DATA_LEN      144


typedef struct __attribute__((packed))  {
    unsigned char   tlnk_mark[4];
    unsigned char   public_key[64];
    unsigned char   signature[64];
    unsigned int    run_code_adr;   //4 byte
    unsigned int    run_code_size;
    unsigned char   watdog_v[4];
} sb_desc_sector_t;


typedef struct __attribute__((packed))  {
    unsigned char   fw_enc_en;
    unsigned char   secboot_en;
    unsigned short  sb_desc_adr_k; //unit: 4KB
} mcu_secure_t;
extern mcu_secure_t  mcuSecur;

bool mcu_securuty_read_efuse(void);
bool mcu_securuty_read_idcode(void);
bool efuse_get_pubkey_hash(u8* pHash);
/******************************* mcu_security end ********************************************************************/

#endif /* DRIVERS_TL721X_EXT_DRIVER_EXT_LIB_H_ */
