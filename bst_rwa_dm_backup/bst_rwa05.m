
#include <bst_rwa05.h>

#include <bst.h>
#include <bst_crc.h>
#include <bst_debug.h>
#include <bst_time.h>
#include <bst_number.h>
#include <bst_mathx.h>

#include <bst_leos80_com.h>
#include <bst_leos80_com_state.h>

// 02_devices
#include <bst_unit_rwa.h>
#include <bst_mdl_rwa.h>

#include <bst_rwa05_cmd_hdl.h>
#include <bst_rwa05_defs.h>

#include <bst_tempsensors.h>

/* STM32 library files */
#include <stm32f10x_conf.h>
#include <stm32f10x_exti.h>
#include <stm32f10x_dbgmcu.h>
#include <stm32f10x_iwdg.h>
#include <misc.h>

/* BST STM32 library files */
#include <bst_stdio.h>
#include <bst_util.h>
#include <bst_stm32f10x_tools.h>

/* RWA05 specific files */
#include <bst_hal_wde.h>      // temperature ADC
#include <bst_wde.h>
#include <bst_ufors.h>
#include <bst_hal_bldc.h>     // Calculate the BST tick

// EXTIO Connected to Pin A.6

// ***************************************************************************************
// ***************************************************************************************
// ***************************************************************************************

extern _t_wde_data _wde_data;

static bst_sys_context_t     *_context;
       bst_rwa05_context_t    _rwa05;

static int  _scheduler(bst_sys_context_t *context);

static int  _init_context(bst_sys_context_t *context);

static void _sys_tick_init(unsigned int tickrate);

static bst_ret_t _update_trq_open(bst_rwa05_context_t * rwa);

// ***************************************************************************************
// ***************************************************************************************
// ***************************************************************************************

#if BST_DEBUG_LEVEL > BST_DEBUG_LEVEL_NO_DEBUG

#define BST_RWA_V_2_DEBUG_UART        USART2
#define BST_RWA_V_2_DEBUG_UART_BAUD   115200

int bst_usart_putchar(int ch)
{
  bst_usart_send_b(BST_RWA_V_2_DEBUG_UART, (UI8*)&ch, 1);
  return ch;
}

static void debug_init(void)
{
  // DEBUG INTERFACE
  bst_usart_init(BST_RWA_V_2_DEBUG_UART, BST_RWA_V_2_DEBUG_UART_BAUD, 0, NULL);
  bst_stdio_init(bst_usart_putchar);
}

#endif

// ***************************************************************************************
// ***************************************************************************************
// ***************************************************************************************

void bst_rwa05_init(void)
{
    // Add an delay of <1s before configuring to assure the voltage is stabilized to avoid brownouts
    bst_stm32f10x_tools_wait_usec(250000);

#if BST_DEBUG_LEVEL > BST_DEBUG_LEVEL_NO_DEBUG
    debug_init();
#endif

    BST_DEBUG_INFO("ENTER\r\n");

    bst_leos80_init(&_context,
    bst_rwa05_cmd_hdl,
    bst_rwa05_check_for_com_activities,
    bst_rwa05_defs_get_devlist_all(),
    BST_RWA05_DEFS_NUM_MASTERS,
    BST_RWA05_DEFS_NUM_SLAVES,
    &_rwa05);

    if(_init_context(_context)!=BST_RETURN_SUCCESS)
    {
        BST_DEBUG_FATAL("_init_context() failed\r\n");
    }

    //stop the timers when the core is stopped in debugger
    DBGMCU_Config( DBGMCU_TIM1_STOP | DBGMCU_TIM2_STOP | DBGMCU_TIM3_STOP | DBGMCU_TIM4_STOP, ENABLE);
    bst_wde_init();
    bst_hal_ufors_init(_wde_settings.gyro_fullscale,
        _wde_settings.gyro_dataBytes, _wde_settings.gyro_triggerRate,
        _wde_settings.gyro_baudRate);

    // RWA Context
    _rwa05.is_test_pending = 0;

    // ------------------------------------------------------------------------------
    // Init RW Parameter + Data

    if(bst_unit_rwa_init_rw_par(&_rwa05.par)!=BST_RETURN_SUCCESS)
    {
      BST_DEBUG_ERROR("bst_unit_rwa_init_rw_par() failed\n\r");
    }

    if(bst_mdl_rw_dat_init(&_rwa05.dat)!=BST_RETURN_SUCCESS)
    {
      BST_DEBUG_ERROR("bst_mdl_rw_dat_init() failed\n\r");
    }

    // ------------------------------------------------------------------------------
    // Angular Velocity

    if(bst_ctrl_init_pid(&_rwa05.pid_ang_v, BST_CTRL_ACT_CMD_RW_CURRENT,
                         -1.0, -0.01, 0.0,                                        // kp, ki, kd
                         NULL, NULL, NULL, NULL, 0,                               // adaptive
                         0, 0.7, _rwa05.par.cur_max,                              // anti-windup
                         0)!=BST_RETURN_SUCCESS)                                  // update interval (not used)
    {
      BST_DEBUG_ERROR("bst_ctrl_init_pid(ang_v) failed\n\r");
    }

    // ------------------------------------------------------------------------------
    // Angle

    if(bst_ctrl_init_pid(&_rwa05.pid_angle, BST_CTRL_ACT_CMD_RW_CURRENT,
                         -1.0, -0.01, -0.01,                                      // kp, ki, kd
                         NULL, NULL, NULL, NULL, 0,                               // adaptive
                         0, 0.7, _rwa05.par.cur_max,                              // anti-windup
                         0)!=BST_RETURN_SUCCESS)                                  // update interval (not used)
    {
      BST_DEBUG_ERROR("bst_ctrl_init_pid(angle) failed\n\r");
    }

    // ------------------------------------------------------------------------------
    // Torque

    if(bst_unit_rwa_init_pid_trq(&_rwa05.pid_trq)!=BST_RETURN_SUCCESS)
    {
      BST_DEBUG_ERROR("bst_unit_rwa_init_pid_trq() failed\n\r");
    }

    _rwa05.trq_open_init = 0;

    // ------------------------------------------------------------------------------
    // Speed

    if(bst_unit_rwa_init_pid_spd(&_rwa05.pid_spd)!=BST_RETURN_SUCCESS)
    {
      BST_DEBUG_ERROR("bst_unit_rwa_init_pid_spd() failed\n\r");
    }

    // ------------------------------------------------------------------------------

    // Initialize and start the main tick: tickrate=100 --> 10ms
    _sys_tick_init(100);

    BST_DEBUG_INFO("LEAVE\r\n");
}

// ***************************************************************************************
// ***************************************************************************************
// ***************************************************************************************

void bst_rwa05_mainloop(void)
{
  BST_DEBUG_INFO("ENTER\r\n");

  while(1)
  {
    IWDG_ReloadCounter();

    if(bst_leos80_com_state_step()!=BST_RETURN_SUCCESS)
    {
        BST_DEBUG_ERROR("bst_leos80_com_state_step() failed.\r\n");
    }

    if(_rwa05.is_test_pending == 1)
    {
      bst_wde_selftest_commutation_open();
      _rwa05.is_test_pending =0;
    }
    if(_rwa05.is_test_pending == 2)
    {
      bst_wde_selftest_commutations(10);
      _rwa05.is_test_pending =0;
    }
    if(_rwa05.is_test_pending == 3)
    {
      bst_wde_selftest_commutations(50);
      _rwa05.is_test_pending =0;
    }

    // Check activities excl. communication
    if(_scheduler(_context)!=BST_RETURN_SUCCESS)
    {
      BST_DEBUG_ERROR("_scheduler() failed.\r\n");
    }

  }
}

// ***************************************************************************************
// ***************************************************************************************
// ***************************************************************************************

static int _init_context(bst_sys_context_t *context)
{
  context->sys.cnt_master_cmds = 0;

  // Intervals
  context->sys.ival_temp_uC          = BST_LEOS80_RWA05_IVAL_TEMP_uC;

  return BST_RETURN_SUCCESS;
}

// ***************************************************************************************
// ***************************************************************************************
// ***************************************************************************************

static int _scheduler(bst_sys_context_t *context)
{
    static uint64_t t_temp_uC = 2000;   // next time to do temperature value 'reading'

    uint64_t msec = bst_time_cnt_msec_get();

    // ----------------------------------------------------------------------------------
    // always

    if(_rwa05.dat.wde_mode == BST_MDL_RWA_WDE_MODE_TRQ_OPEN)
    {
        _update_trq_open(&_rwa05);
    }

    // ----------------------------------------------------------------------------------
    // Temperatures

    if(msec > t_temp_uC)
    {
        _wde_data.temperatures[0] = bst_tempsensors_lm135_calc(adc_buf[0] * 3.3 / 4095.0);
        _wde_data.temperatures[1] = bst_tempsensors_stm32int_dig(adc_buf[1]);
        _wde_data.t_temp          = bst_time_cnt_msec_get();

        _context->sys.temp_uC  = _wde_data.temperatures[1];
        t_temp_uC             += context->sys.ival_temp_uC;
    }

    return BST_RETURN_SUCCESS;
}

// ***************************************************************************************
// ***************************************************************************************
// ***************************************************************************************

/**
   Currently set to 10 ms handles

 * @brief  This function handles SysTick Handler and distributes to other ticks
 * @param  None
 * @retval None
 */
void SysTick_Handler(void) __attribute__((interrupt));
void SysTick_Handler(void)
{
    static uint16_t SysTickCounter = 0;

    SysTickCounter++;

    // ----------------------------------------------------------------------------------
    // Run each 

    // TODO-MB --> not good, must be synced with "gyro_triggerRate" variable !!!!
    bst_ufors_tick();              // Triggers gyro reading


    // ------------------------------------------------------------------------------------
    // ------------------------------------------------------------------------------------
    // Support speed measurement -- Prevent TIM overflow: BST_HAL_BLDC_TIM_MAX_PERIOD

    if( (bst_time_cnt_msec_get_since_epoch() - _rwa05.bldc_dat.phase_t) >=  (BST_HAL_BLDC_TIM_MAX_PERIOD - 15))
    {
        bst_hal_bldc_rewind_tim(&_rwa05.bldc_dat);
    }

    // ----------------------------------------------------------------------------------
    // ----------------------------------------------------------------------------------

    if ((SysTickCounter % (BST_WDE_CTRL_INTVAL/10)) == 0)
    {
        bst_wde_tick();            // Control loops, ...
    }
}

// ***************************************************************************************
// ***************************************************************************************
// ***************************************************************************************

/**
 * @brief  This function initializes SysTick Handler.
 * @param  Ticks per Second.
 * @retval None
 */
void _sys_tick_init(unsigned int tickrate)
{
    if(SysTick_Config((SystemCoreClock) / tickrate))           // Setup SysTick Timer, e.g. tickrate=100 results in 10 msec interrupts
    {
        BST_DEBUG_ERROR(".fault.:Systick_Config\n\r");
    }
    NVIC_SetPriority(SysTick_IRQn, 0x8);                       // Low priority, Priority group 3, Subpriority group 0
    return;
}

// ***************************************************************************************
// ***************************************************************************************
// ***************************************************************************************

bst_ret_t _update_trq_open_calc_cur_fw(FXX * cur_fw, const FXX trq_int, 
                                                     const int cnt_p1, const int cnt_p2, const int cnt_ps, 
                                                     const bst_rwa05_context_t * rwa)
{
    // assuming explicit current driven is smaller than actual int. current
    // (so only int. current flows through motor ...)

    // tgt    = ( num1*trq_int + num2*trq_fw ) / nums                           (average torque)
    // trq_fw = ( nums*tgt - num1*trq_int    ) / num2
    //          with trq_int = bst_mdl_rw_get_friction(NEG);
    //               trq_fw  = km * cur_fw + bst_mdl_rw_get_friction(POS);
    //
    // cur_fw = ( ( nums*tgt*ki - num1*trq_int*ki ) / num2 ) - bst_mdl_rw_get_friction(POS)*ki

    *cur_fw = ((FXX)cnt_ps) * rwa->dat.tgt*rwa->par.ki - ((FXX)cnt_p1) * trq_int * rwa->par.ki;
    *cur_fw = (*cur_fw) / ((FXX)cnt_p2);
    *cur_fw = (*cur_fw) - bst_mdl_rw_get_friction(&rwa->par, rwa->dat.spd, BST_MDL_RWA_FRCTN_TYPE_POS)*rwa->par.ki;

    return BST_RETURN_SUCCESS;
}

bst_ret_t _update_trq_open(bst_rwa05_context_t * rwa)
{
    static int64_t p_cnt_old = BST_NUMBER_MAX_I32;                      // make sure we only trigger after cmd is sent
    static int     part1_ready;
    static FXX     cur_fw;                                              // forward current (same sign as speed)

    // Phase count init values
    // part1 = 2 phase changes and part2 = 2 phase changes ensures that the different parts (px) are always happening
    // at different HAL sensor states (Hx) as total number of phases per revolution is 6
    // (needed as motor is not symmetric, so time per phase is not equal ... so averaging it out)
    // H1 H2 H3 H4 H5 H6 H1 H2 H3 H4 H5 H6 ...
    // p1 p1 p2 p2 p1 p1 p2 p2 p1 p1 p2 p2 ...
    //
    // ... but we use speed dependend values with criteria: 2*min should not be multiple of 3

    static int     cnt_p1;
    static int     cnt_p2;
    static int     cnt_ps;
    static int     cnt_min;

    // make minimum cnt dependend on actual speed
    // - target is to have about 10-20ms per part
    // - 5000 rpm -->  2 ms per phase
    //   2500 rpm -->  4 ms
    //   1250 rpm -->  8 ms
    //   1000 rpm --> 10 ms
    //    500 rpm --> 20 ms
    if(     fabsx(rwa->dat.spd) > (4000*RPM_TO_RADPSEC))  { cnt_min = 10; }  // 10, so update > 40 ms
    else if(fabsx(rwa->dat.spd) > (3000*RPM_TO_RADPSEC))  { cnt_min =  7; }  //  7
    else if(fabsx(rwa->dat.spd) > (2000*RPM_TO_RADPSEC))  { cnt_min =  4; }  //  4
    else if(fabsx(rwa->dat.spd) > (1000*RPM_TO_RADPSEC))  { cnt_min =  2; }
    else                                                  { cnt_min =  2; }

    if(rwa->trq_open_init)
    {
        cnt_p1 = cnt_min;
        cnt_p2 = cnt_min;
        cnt_ps = 2*cnt_min;

        p_cnt_old = rwa->bldc_dat.phase_cnt_tot - ((int64_t)cnt_ps); // make sure part 1 is started
        rwa->trq_open_init = 0;
    }

    if((p_cnt_old + cnt_ps) <= rwa->bldc_dat.phase_cnt_tot)             // part 2 done -- restart procedure, so start part 1
    {
        if((rwa->dat.spd * rwa->dat.tgt) > 0.0)                         // no internal current, just drive the necessary current
        {
            rwa->dat.cur_tgt = rwa->par.ki * (rwa->dat.tgt - bst_mdl_rw_get_friction(&rwa->par, rwa->dat.spd, BST_MDL_RWA_FRCTN_TYPE_POS));
            part1_ready = 1;                                            // just do it for both parts

            cnt_p1 = cnt_min;                                           // set back to default in case of prior part length shifting
            cnt_p2 = cnt_min;
            cnt_ps = 2*cnt_min;
        }
        // spd and trq/cur will be in opposite direction
        // --> internal current driven by the Back-EMF through the diodes
        //
        // - in the first part we drive the 'too big' internal current
        // - in the 2nd part we drive the appropriate 'pos.' current to have desired average trq over both parts
        else
        {
            FXX trq_int = bst_mdl_rw_get_friction(&rwa->par, rwa->dat.spd, BST_MDL_RWA_FRCTN_TYPE_NEG);

            // 'friction' trq already bigger than target
            if(fabsf(trq_int) > fabsf(rwa->dat.tgt))
            {
                // drive the minimum reverse current in part 1 (triggers internal current)
                if(rwa->dat.spd > 0.0)  { rwa->dat.cur_tgt = -rwa->par.cur_min; }
                else                    { rwa->dat.cur_tgt = +rwa->par.cur_min; }

                // 'internal friction' trq driven in part 1 is smaller than target
                //     We now need to increase the length of part 1 vs. part 2 until the trq of part one is again bigger than target 
                //     to drive a forward current in the second part
                //     (otherwise in the second part a small reverse current would be needed that is not possible because of the internal currents)
                while( ((((FXX)cnt_p1)/((FXX)cnt_ps))*fabsf(trq_int)) < fabsf(rwa->dat.tgt))
                {
                    if(cnt_p2 > cnt_min) { cnt_p2--; cnt_ps=cnt_p1+cnt_p2; } // First try to lower part2 length, but at least one phase transition
                    else                 { cnt_p1++; cnt_ps=cnt_p1+cnt_p2; } // otherwise increase part 1 length ...
                }

                _update_trq_open_calc_cur_fw(&cur_fw, trq_int, cnt_p1, cnt_p2, cnt_ps, rwa);

                // 'internal friction' trq driven in part 1 is too high (necessary forward current in part 2 higher than max. current)
                //     We now need to decrease the length of part 1 vs. part 2 until the trq of part one is again small enough
                //     to be compensated by driving a forward current in the second part
                while(fabsx(cur_fw) > rwa->par.cur_max)
                {
                    if(cnt_p1 > cnt_min) { cnt_p1--; cnt_ps=cnt_p1+cnt_p2; } // First try to lower part1 length, but at least one phase transition
                    else                 { cnt_p2++; cnt_ps=cnt_p1+cnt_p2; } // otherwise increase part 2 length ...

                    _update_trq_open_calc_cur_fw(&cur_fw, trq_int, cnt_p1, cnt_p2, cnt_ps, rwa);
                } 

                part1_ready = 0;                                        // allow different 2nd part
            }
            // friction lower than target:
            // - the necessary target current driven now will prevent the internal current from flowing
            //   (diode will not carry any current, so is dead because voltage drop over motor resistance is already bigger than V_BEMF-V_D then)
            // - just compensate for normal friction (... like commutating forward)
            //
            else
            {
                rwa->dat.cur_tgt = rwa->par.ki * (rwa->dat.tgt - bst_mdl_rw_get_friction(&rwa->par, rwa->dat.spd, BST_MDL_RWA_FRCTN_TYPE_POS));
                part1_ready = 1;                                        // just do it for both parts

                cnt_p1 = cnt_min;                                       // set back to default in case of prior part length shifting
                cnt_p2 = cnt_min;
                cnt_ps = 2*cnt_min;
            }
        }
        bst_wde_setCurrent(rwa->dat.cur_tgt);

        p_cnt_old += cnt_ps;
    }
    else if((p_cnt_old + cnt_p1) <= rwa->bldc_dat.phase_cnt_tot)        // first part ready
    {
        if(part1_ready==0)                                              // only call at start of second part or never (in case no internal current compensatin is necessary)
        {
            rwa->dat.cur_tgt = cur_fw;
            bst_wde_setCurrent(rwa->dat.cur_tgt);
            part1_ready = 1;
        }
    }

    rwa->trq_open_cnt_p1 = cnt_p1;
    rwa->trq_open_cnt_p2 = cnt_p2;

    return BST_RETURN_SUCCESS;
}
