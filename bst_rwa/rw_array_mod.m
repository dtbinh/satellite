/*! \file      bst_rw_array_mod.c
    \brief     Implements reaction wheel
    \author    Nicolas Vinikoff, Matthias Buhl
    \copyright Copyright 2016, Berlin space Technologies GmbH
*/

#include <bst_rw_array_mod.h>

#include <string.h>
#include <stdlib.h>

#include <bst_mathx.h>
#include <bst_matrixx.h>
#include <bst_debug.h>
#include <bst_control.h>

// **************************************************************************************
// **************************************************************************************
// **************************************************************************************

/**
  * Dynamics Equation Propagation based on Euler Propagation
  *
  * @return   \ref bst_ret_t
  **/
static bst_ret_t _array_euler(bst_rw_array_dev_param_t *dev,
                              bst_rw_array_dev_dat_t   *dat,
                              bst_rw_array_mod_param_t *mod_dev,
                              bst_rw_array_mod_dat_t   *mod_dat,
                              double dt);

static int _ang_mom_saturation(bst_rw_array_dev_dat_t *rwdata_new, bst_rw_array_dev_param_t *rw_dev);
static int _torque_saturation( bst_rw_array_dev_dat_t *rwdata_new, bst_rw_array_dev_param_t *rw_dev);


// **************************************************************************************
// **************************************************************************************
// **************************************************************************************

static int _ang_mom_saturation(bst_rw_array_dev_dat_t *rwdata_new, bst_rw_array_dev_param_t *rwa_dev)
{
  for(int i=0; i<rwa_dev->RW_Number; i++)
  {
    // Checking if the reaction wheel has not reached its saturation
    if( fabsx(rwdata_new->rw[i].h) >= (rwa_dev->rw[i].n_max*rwa_dev->rw[i].MoI) ) // if h >= max_speed*Moi
    {
      if (rwdata_new->rw[i].h > 0)                                              // if h > 0
      {
        rwdata_new->rw[i].sp = rwa_dev->rw[i].n_max;                                  // speed = max_speed
        rwdata_new->rw[i].h  = rwdata_new->rw[i].sp * rwa_dev->rw[i].MoI;             // h = speed * Moi
      }
      else                                                                      // else if h < 0
      {
        rwdata_new->rw[i].sp = -1.0*rwa_dev->rw[i].n_max;                             // speed = -max_speed
        rwdata_new->rw[i].h  = rwdata_new->rw[i].sp * rwa_dev->rw[i].MoI;             // h = speed*Moi
      }
      rwdata_new->rw[i].trq = 0;                                                // set trq = 0
    }
  }
  return 0;
}

// **************************************************************************************

static int _torque_saturation(bst_rw_array_dev_dat_t *rwdata_new, bst_rw_array_dev_param_t *rwa_dev)
{
  for(int i=0; i<rwa_dev->RW_Number; i++)
  {
    // Checking if the reaction wheel has not reached its saturation
    if (fabsx(rwdata_new->rw[i].trq) >= rwa_dev->rw[i].trq_max)          // if torque >= torque_max
    {
      if (rwdata_new->rw[i].trq > 0)                                     // if torque > 0
      {
        rwdata_new->rw[i].trq = rwa_dev->rw[i].trq_max;                     // torque = torque_max
      }
      else
      {
        rwdata_new->rw[i].trq = -1.0*rwa_dev->rw[i].trq_max;            // else torque = -torque_max
      }
    }
  }
  return 0;
}

// **************************************************************************************
// **************************************************************************************
// **************************************************************************************

bst_ret_t bst_rwa_friction_model(FXX *trq_frc,                        // torque friction
                                 FXX *w,                              // omega
                                 bst_rw_array_dev_param_t *dev_par,   // dev_parameters
                                 bst_rw_array_mod_param_t *mod_par)   // mod_parameters
{
  for(int i=0; i<dev_par->RW_Number; i++)
  {
    trq_frc[i] = bst_rwa_friction_model_idx(w[i], &mod_par->rw[i]); // calculates the torque friction for each wheel 
  }
  return BST_RETURN_SUCCESS;
}

FXX bst_rwa_friction_model_idx(const FXX w, const bst_rw_mod_param_t *mod_par)
{
  
  return (- bst_mathx_sign(w) * (  fabsx(w)*fabs(w)*mod_par->C_c  +
                                   fabsx(w)        *mod_par->C_b  +
                                                    mod_par->C_a  ) );  //      - sign(w) * [  C*abs(w^2) + B*abs(w) + A ]
  return BST_RETURN_SUCCESS;
}

// **************************************************************************************
// **************************************************************************************
// **************************************************************************************

bst_ret_t bst_rwa_mod_set_idx(int idx, 
                              bst_rw_array_dev_dat_t*   dat __attribute__((unused)),
                              bst_rw_array_dev_param_t* dev)
{
  // modeling/simulating: reset internal PID
  ((bst_rw_array_mod_param_t*)(dev->gp_ptr))->rw[idx].pid.init = 1;

  return BST_RETURN_SUCCESS;
}

// **************************************************************************************
// **************************************************************************************
// **************************************************************************************

bst_ret_t bst_rwa_sim_step(bst_rw_array_dev_param_t *dev,
                           bst_rw_array_dev_dat_t   *dat,
                           bst_rw_array_mod_param_t *mod_dev,
                           bst_rw_array_mod_dat_t   *mod_dat,
                           double dt, int64_t t)
{
  (void)t;
  if(dev->ctrl_mode == BST_RW_CTRL_MODE_TORQUE)
  {
    if(bst_rwa_sim_step_trq_mode(dev, dat, mod_dev, mod_dat, t)!=0)
    {
      BST_DEBUG_ERROR("bst_rwa_sim_step_trq_mode() failed\r\n");
      return BST_RETURN_FAILURE;
    }
  }
  else if(dev->ctrl_mode == BST_RW_CTRL_MODE_SPEED)
  {
    if(bst_rwa_sim_step_spd_mode(dev, dat, mod_dev, mod_dat, t)!=0)
    {
      BST_DEBUG_ERROR("bst_rwa_sim_step_spd_mode() failed\r\n");
      return BST_RETURN_FAILURE;
    }
  }
  else if(dev->ctrl_mode == BST_RW_CTRL_MODE_CURRENT)
  {
    if(bst_rwa_sim_step_cur_mode(dev, dat, mod_dev, mod_dat)!=0)
    {
      BST_DEBUG_ERROR("bst_rwa_sim_step_cur_mode() failed\r\n");
      return BST_RETURN_FAILURE;
    }
  }
  else
  {
    BST_DEBUG_ERROR("RW CTRL mode not implemented: dev->ctrl_mode = %d\r\n", dev->ctrl_mode);
    return BST_RETURN_FAILURE;
  }






  // Determining the RW response based on the driven current: euler dynamics propagation
  if(_array_euler(dev, dat, mod_dev, mod_dat, dt)!=0)
  {
    BST_DEBUG_ERROR("_array_euler() failed\r\n");
    return BST_RETURN_FAILURE;
  }

  return BST_RETURN_SUCCESS;
}

// **************************************************************************************
// **************************************************************************************
// **************************************************************************************

bst_ret_t bst_rwa_sim_step_trq_mode(bst_rw_array_dev_param_t *dev,
                                    bst_rw_array_dev_dat_t   *dat,
                                    bst_rw_array_mod_param_t *mod_dev,
                                    bst_rw_array_mod_dat_t   *mod_dat,
                                    int64_t t)
{
  int num = dev->RW_Number;

  BST_DEBUG("trq_tgt:   %10.3f %10.3f %10.3f [mNm]\n\r", dat->rw[0].tgt*1e3,
                                                         dat->rw[1].tgt*1e3,
                                                         dat->rw[2].tgt*1e3);

  if(mod_dev->model_type == BST_RW_MODEL_TYPE_GENERAL_BLDC)
  {
    for(int i=0; i<num; i++)
    {
      if(mod_dev->rw[i].pid.init)
      {
        // integrator is initialized with actual output value then ...
        //                         [A/Nm]                       * [Nm]
        mod_dat->rw[i].cur_cmd = (1.0/mod_dev->rw[i].bldc_kM) * (dat->rw[i].tgt - bst_rwa_friction_model_idx(dat->rw[i].sp*RPS2RPM, &mod_dev->rw[i])); // init value

        mod_dat->rw[i].t_next  = t;
      }

      if(t >= mod_dat->rw[i].t_next)
      {
        mod_dat->rw[i].t_next += mod_dev->rw[i].pid.update_intvl;               

        bst_ctrl_pid_update(&mod_dat->rw[i].cur_cmd, dat->rw[i].tgt - dat->rw[i].trq,
                            &mod_dev->rw[i].pid, mod_dev->intvl_mod);
      }
      BST_DEBUG("cmd/ctrl current[%d]: %10.3f A | t=%lu t_next=%lu\n\r", i, mod_dat->rw[i].cur_cmd, (long unsigned)t, (long unsigned)mod_dat->rw[i].t_next);
    }
  }
  else if(mod_dev->model_type == BST_RW_MODEL_TYPE_TORQUE_FIRST_ORDER)
  {
    for(int i=0; i<num; i++)
    {
      mod_dat->rw[i].trq_cmd = dat->rw[i].tgt;
    }
  }
  else if(mod_dev->model_type == BST_RW_MODEL_TYPE_HT_RW200_15)
  {
    for(int i=0; i<num; i++)
    {
      mod_dat->rw[i].trq_cmd = dat->rw[i].tgt;
    }
  }
  else
  {
    BST_DEBUG_ERROR("Unknown model: %u [mod_dev*: 0x%p]\r\n",mod_dev->model_type,mod_dev);
    return BST_RETURN_FAILURE;
  }

  return 0;
}

// **************************************************************************************
// **************************************************************************************
// **************************************************************************************

bst_ret_t bst_rwa_sim_step_spd_mode(bst_rw_array_dev_param_t *dev,
                                    bst_rw_array_dev_dat_t   *dat,
                                    bst_rw_array_mod_param_t *mod_dev,
                                    bst_rw_array_mod_dat_t   *mod_dat,
                                    int64_t t)
{
  if(mod_dev->model_type == BST_RW_MODEL_TYPE_GENERAL_BLDC)
  {
    for(int i=0; i<dev->RW_Number; i++)
    {
      if(mod_dev->rw[i].pid.init)
      {
        mod_dat->rw[i].cur_cmd = 0.0; // init value
        mod_dat->rw[i].t_next    = t;
      }

      if(t >= mod_dat->rw[i].t_next)
      {
        mod_dat->rw[i].t_next += mod_dev->rw[i].pid.update_intvl;
        
        bst_ctrl_pid_update(&mod_dat->rw[i].cur_cmd, dat->rw[i].tgt - dat->rw[i].sp,
                            &mod_dev->rw[i].pid, mod_dev->intvl_mod);
      }
      BST_DEBUG("cmd/ctrl current[%d]: %10.3f A | t=%lu t_next=%lu\n\r", i, mod_dat->rw[i].cur_cmd, (long unsigned)t, (long unsigned)mod_dat->rw[i].t_next);
    }
  }
  else if(mod_dev->model_type == BST_RW_MODEL_TYPE_TORQUE_FIRST_ORDER)
  {
    for(UI8 i=0;i<dev->RW_Number;i++)
    {
      if(bst_ctrl_pid_update(&mod_dat->rw[i].trq_cmd, dat->rw[i].tgt - dat->rw[i].sp, &mod_dev->rw[i].pid, mod_dev->intvl_mod)!=0)
        return 1;
    }
  }
  else if(mod_dev->model_type == BST_RW_MODEL_TYPE_HT_RW200_15)
  {
    for(int i=0; i<dev->RW_Number; i++)
    {
      mod_dat->rw[i].spd_cmd = dat->rw[i].tgt;
    }
  }
  else
  {
    BST_DEBUG_ERROR("Unknown model\r\n");
    return BST_RETURN_FAILURE;
  }

  return 0;
}

// **************************************************************************************
// **************************************************************************************
// **************************************************************************************

bst_ret_t bst_rwa_sim_step_cur_mode(bst_rw_array_dev_param_t *dev,
                                    bst_rw_array_dev_dat_t   *dat,
                                    bst_rw_array_mod_param_t *mod_dev,
                                    bst_rw_array_mod_dat_t   *mod_dat)
{
  FXX current_error[BST_RW_ARRAY_N_MAX];
  int num = dev->RW_Number;


  if(mod_dev->model_type == BST_RW_MODEL_TYPE_GENERAL_BLDC)
  {
    for(int i=0; i<num; i++)
    {
      mod_dat->rw[i].cur_cmd = dat->rw[i].tgt;
    }
  }
  else if(mod_dev->model_type == BST_RW_MODEL_TYPE_TORQUE_FIRST_ORDER)
  {
    FXX t_cmd[BST_RW_ARRAY_N_MAX];

    for(int i=0; i<num; i++)
    {
      current_error[i] = dat->rw[i].tgt - dat->rw[i].cur;

      if(bst_ctrl_pid_update(&t_cmd[i], current_error[i], &mod_dev->rw[i].pid, mod_dev->intvl_mod)!=0) return 1;
      mod_dat->rw[i].trq_cmd = t_cmd[i];
    }
  }
  else if(mod_dev->model_type == BST_RW_MODEL_TYPE_HT_RW200_15)
  {
    FXX s_cmd[BST_RW_ARRAY_N_MAX];

    for(int i=0; i<num; i++)
    {
      current_error[i] = dat->rw[i].tgt - dat->rw[i].cur;
      if(bst_ctrl_pid_update(&s_cmd[i], current_error[i], &mod_dev->rw[i].pid, mod_dev->intvl_mod)!=0) return 1;
      mod_dat->rw[i].spd_cmd = s_cmd[i];
    }
  }
  else
  {
    BST_DEBUG_ERROR("Unknown model\r\n");
    return BST_RETURN_FAILURE;
  }

  return 0;
}

// **************************************************************************************
// **************************************************************************************
// **************************************************************************************

/* Forward integration using explicit euler integration (forward) */
static bst_ret_t _array_euler(bst_rw_array_dev_param_t *dev,
                              bst_rw_array_dev_dat_t   *dat,
                              bst_rw_array_mod_param_t *mod_dev,
                              bst_rw_array_mod_dat_t   *mod_dat,
                              double dt)
{
  static double cnt = 0;

  FXX h[BST_RW_ARRAY_N_MAX];
  FXX trq[BST_RW_ARRAY_N_MAX];
  int num = dev->RW_Number;

  // Forward integration of euler
  if(mod_dev->model_type == BST_RW_MODEL_TYPE_GENERAL_BLDC)//BST_RW_MODEL_TYPE_GENERAL_BLDC///////////////////////////////////////////////////////////////////////////////////////
  {
    for(int i=0; i<num; i++)     // save old ang. mom.
    {
      h[i] = dat->rw[i].h;
    }

    if(bst_rwa_dynamics_rwa05(trq, dev, dat, mod_dev, mod_dat)!=0)
      return 1;

    // RWA Data Update
    for(int i=0; i<num; i++)
    {
      dat->rw[i].h  = h[i] + trq[i] * dt;                  // ang. momentum

      dat->rw[i].trq = trq[i];                             // measured trq val   TODO-MB: update of trq only with commutation state change
      dat->rw[i].sp  = (1.0/dev->rw[i].MoI)*dat->rw[i].h;  // [rps] measured spd val   TODO-MB: update of spd only with commutation state change
      dat->rw[i].cur = mod_dat->rw[i].cur_motor;           // measured cur val
      dat->rw[i].valid = 1;
    }
  }
  else if(mod_dev->model_type == BST_RW_MODEL_TYPE_TORQUE_FIRST_ORDER) //BST_RW_MODEL_TYPE_TORQUE_FIRST_ORDER/////////////////////////////////////////////////////////////////
  {
    FXX torque_cmd[BST_RW_ARRAY_N_MAX];
    FXX dhdot[BST_RW_ARRAY_N_MAX];
    FXX dh[BST_RW_ARRAY_N_MAX];

    for(int i=0;i<num;i++)
    {
      trq[i] = dat->rw[i].trq;                // save old trq
    }

    for(int i=0;i<num;i++)
    {
      torque_cmd[i] = mod_dat->rw[i].trq_cmd; // save old trq_cmd
    }

    // Runge-Kutta 1st step
    if(bst_rwa_dynamics_first_order_trq(dhdot, trq, torque_cmd, dev,mod_dev)!=0) return 1;
    if(bst_matrixx_mul_by_scalar(dh,dhdot,num,1,dt)!=0) return 1;

    // Reaction Wheels Developed Torques Update
    for(int i=0;i<num;i++)
    {
      dat->rw[i].trq  = trq[i] + dh[i];
    }
    if(_torque_saturation(dat, dev)!=0) return 1;

    // Angular Momentum Update
    for(int i=0;i<num;i++)
    {
      dat->rw[i].h  = dat->rw[i].h + dat->rw[i].trq*dt;
      dat->rw[i].sp = 1.0/dev->rw[i].MoI * dat->rw[i].h;
      dat->rw[i].valid = 1;
    }
    if(_ang_mom_saturation(dat,dev)!=0) return 1;

  }// eof if(mod_dev->model_type)

  else if(mod_dev->model_type == BST_RW_MODEL_TYPE_HT_RW200_15)   /////////////////////// iACDS Model//////////////////////////////////////////////////
  {
    if(dev->ctrl_mode == BST_RW_CTRL_MODE_TORQUE)////////////////////////// BST_RW_CTRL_MODE_TORQUE////////////////
    {
      FXX torque_cmd[BST_RW_ARRAY_N_MAX];
      FXX w_dot_in[BST_RW_ARRAY_N_MAX];
      FXX h_dot[BST_RW_ARRAY_N_MAX];

      for(int i=0;i<num;i++)
      {
        torque_cmd[i] = mod_dat->rw[i].trq_cmd;
        w_dot_in[i] = torque_cmd[i]/dev->rw[i].MoI;
      }

      if(cnt >= 0.7)
      {
        if(bst_rwa_dynamics_iacds_trq(h_dot, w_dot_in, dev)!=0)
        {
          BST_DEBUG_ERROR("bst_rwa_dynamics_iacds_trq() failed!\n\r");
          return 1;
        }
      }
      else
      {
        for (int i = 0; i < num; ++i)
        {
          h_dot[i] = 0;
        }
      }

      // Reaction Wheels Developed Torques Update
      for(int i=0;i<num;i++)
      {
        dat->rw[i].trq  = h_dot[i];
      }

      cnt += dt;
    } 
    else if(dev->ctrl_mode == BST_RW_CTRL_MODE_SPEED)    ///////////////// BST_RW_CTRL_MODE_SPEED/////////////////////////////////////
    {
      FXX dhdot[BST_RW_ARRAY_N_MAX], dh[BST_RW_ARRAY_N_MAX];

      for(int i=0;i<num;i++)
      {
        trq[i] = dat->rw[i].trq;  // save old trq
      }
      //            t_dot = d(dh_dot)/dt = I_RW*[-(2*d/t)*h_dot/I_RW - (1/t^2)*speed + (k/t^2)*speed_cmd] */
      //            t_dot                = I_RW * [     a*h_dot      +       b*speed +       c*speed_cmd]                                     */
      // where:     a = -(2*d/(t*I_RW))
      //            b = -(1/t^2)
      //            c =  (k/t^2)     

      if(bst_rwa_dynamics_iacds_spd(dhdot, dev, dat, mod_dev, mod_dat)!=0) return 1;
      if(bst_matrixx_mul_by_scalar(dh, dhdot, num, 1, dt)!=0) return 1;

      // Reaction Wheels Developed Torques Update
      for(int i=0; i<num; i++)
      {
        dat->rw[i].trq = trq[i] + dh[i];
      }
    }

    if(_torque_saturation(dat, dev)!=0) return 1;

    // Angular Momenta Update
    for(int i=0;i<num;i++)
    {
      dat->rw[i].h  = dat->rw[i].h + dat->rw[i].trq*dt;
      dat->rw[i].sp = (1.0/dev->rw[i].MoI)*dat->rw[i].h;
      dat->rw[i].valid = 1;
    }

    if(_ang_mom_saturation(dat,dev)!=0) return 1;
  }// eof if(rwa_mod_dev->model_type)

  return 0;
}

// **************************************************************************************
// **************************************************************************************
// **************************************************************************************

bst_ret_t bst_rwa_dynamics_rwa05(FXX *h_dot,
                                 bst_rw_array_dev_param_t *dev,
                                 bst_rw_array_dev_dat_t   *dat,
                                 bst_rw_array_mod_param_t *mod_dev,
                                 bst_rw_array_mod_dat_t   *mod_dat)
{
  FXX trq[BST_RW_ARRAY_N_MAX];
  FXX trq_friction[BST_RW_ARRAY_N_MAX];
  FXX spd_rpm[BST_RW_ARRAY_N_MAX];     // [rpm]
  FXX cur[BST_RW_ARRAY_N_MAX];

  // ------------------------------------------------------------------------------------
  // motor current to motor torque

  for(int i=0; i<dev->RW_Number; i++)
  {
    spd_rpm[i] = dat->rw[i].sp*RPS2RPM;

    // ----------------------------------------------------------------------------------
    // ang. momentum saturation; max. speed check
    if( (dat->rw[i].sp >=  dev->rw[i].n_max) ||
        (dat->rw[i].sp <= -dev->rw[i].n_max)  )
    {
      cur[i] = 0.0;
    }
    else
    {
      // --------------------------------------------------------------------------------
      // Limit current to user limit value
      cur[i] = bst_mathx_saturate_lim(mod_dat->rw[i].cur_cmd, mod_dev->rw[i].I_max);

      // --------------------------------------------------------------------------------
      // Adapt current due to Back-EMF: I_max_bemf = (U_sup - U_bemf)/R
      {
        FXX cur_max_bemf = (mod_dev->rw[i].U_sup - (fabsx(spd_rpm[i])*mod_dev->rw[i].bldc_kE)) / mod_dev->rw[i].bldc_R;

        // Back-EMF limits the maximum current
        cur[i] = bst_mathx_saturate_lim(cur[i], cur_max_bemf);

#if 0
        // spd and cur in opposite direction
        // --> internal current driven by the Back-EMF through the diodes (voltage drop of 0.7V is ignored)
        if(dat->rw[i].sp*mod_dat->rw[i].cur_motor < 0)
        {
          // add the additional current
          // either cur < 0 and spd > 0
          // or     cur > 0 and spd < 0
          cur[i] = cur[i] - spd_rpm[i]*mod_dev->rw[i].bldc_kE / mod_dev->rw[i].bldc_R;
        }
#endif
      }
    }

    // save resulting motor current
    mod_dat->rw[i].cur_motor = cur[i];

    // Conversion losses (Motor Efficiency bldc_n) and kM (Motor Torque Constant)
    trq[i]                   = cur[i] * mod_dev->rw[i].bldc_n * mod_dev->rw[i].bldc_kM;
  }

  // ------------------------------------------------------------------------------------
  // motor torque to net-torque

  // Torque is induced torque minus/plus friction torque.
  // Friction torque is already calculated to be of oppposite sign to spd
  if(bst_rwa_friction_model(trq_friction, spd_rpm, dev, mod_dev)  !=BST_RETURN_SUCCESS)  return BST_RETURN_FAILURE;
  if(bst_matrixx_add(h_dot, trq, trq_friction, dev->RW_Number, 1) !=BST_RETURN_SUCCESS)  return BST_RETURN_FAILURE;

  BST_DEBUG("Net-Torque:  %10.6f %10.6f %10.6f [Nm]\r\n", h_dot[0], h_dot[1], h_dot[2]);
  return BST_RETURN_SUCCESS;
}

// **************************************************************************************
// **************************************************************************************
// **************************************************************************************

bst_ret_t bst_rwa_dynamics_first_order_trq(FXX *dh_dot,
                                           FXX *h_dot,
                                           FXX *t_demanded,
                                           bst_rw_array_dev_param_t *dev,
                                           bst_rw_array_mod_param_t *mod_dev)
{
  FXX bandwidth_matrix[BST_RW_ARRAY_N_MAX*BST_RW_ARRAY_N_MAX];
  FXX bandwidth_vec[BST_RW_ARRAY_N_MAX];
  FXX sub[BST_RW_ARRAY_N_MAX];
  int num = dev->RW_Number;

  /* Generation of a Diagonal Matrix whose Diagonal Comprises the Inverse of the RWs Banwidths          */
  /*                            bandwidth_matrix = diag(BW1,BW2,...,BWN)                          */
  for(int i=0;i<num;i++)
  {
    bandwidth_vec[i] = mod_dev->rw[i].bandwidth;
  }
  if(bst_matrixx_create_diag_mat(bandwidth_matrix,bandwidth_vec,num,num,num)!=0)
  {
    BST_DEBUG_ERROR("-E- [bst_rw_array.c::bst_rw_array_torque_dynamics()] bst_matrixx_create_diag_mat() failed.");
    return 1;
  }

  /* Computation of the Difference between the Demanded Torque and the Current Torque:                  */
  /*                                        t_demanded - h_dot                                          */
  if(bst_matrixx_sub(sub,t_demanded,h_dot,num,1)!=0)
  {
    BST_DEBUG_ERROR("-E- [bst_rw_array.c::bst_rw_array_torque_dynamics()] bst_matrixx_sub() failed.");
    return 1;
  }

  /* Computation of the Torque Derivative Vector: bandwidth_matrix*(t_demanded - h_dot)                 */
  if(bst_matrixx_mul(dh_dot,bandwidth_matrix,sub,num,num,1)!=0)
  {
    BST_DEBUG_ERROR("-E- [bst_rw_array.c::bst_rw_array_torque_dynamics()] bst_matrixx_mul() failed.");
    return 1;
  }

  return 0;
}

// **************************************************************************************
// **************************************************************************************
// **************************************************************************************

/*            t_dot = d(dh_dot)/dt = I_RW*[-(2*d/t)*h_dot/I_RW - (1/t^2)*speed + (k/t^2)*speed_cmd] */
/*            t_dot                = I_RW * [     a*h_dot      +       b*speed +       c*speed_cmd]                                     */
/* where:     a = -(2*d/(t*I_RW))
              b = -(1/t^2)
              c =  (k/t^2)                                                                          */
bst_ret_t bst_rwa_dynamics_iacds_spd(FXX *dh_dot,
                                     bst_rw_array_dev_param_t *dev,
                                     bst_rw_array_dev_dat_t   *dat,
                                     bst_rw_array_mod_param_t *mod_dev,
                                     bst_rw_array_mod_dat_t   *mod_dat)
{
  FXX I_RW[BST_RW_ARRAY_N_MAX];
  FXX a[BST_RW_ARRAY_N_MAX], b[BST_RW_ARRAY_N_MAX], c[BST_RW_ARRAY_N_MAX];
  FXX prod_mat1[BST_RW_ARRAY_N_MAX], prod_mat2[BST_RW_ARRAY_N_MAX], prod_mat3[BST_RW_ARRAY_N_MAX];
  FXX add_mat1[ BST_RW_ARRAY_N_MAX], add_mat2[ BST_RW_ARRAY_N_MAX];
  FXX diag[  BST_RW_ARRAY_N_MAX*BST_RW_ARRAY_N_MAX];
  FXX diag_a[BST_RW_ARRAY_N_MAX*BST_RW_ARRAY_N_MAX];
  FXX diag_b[BST_RW_ARRAY_N_MAX*BST_RW_ARRAY_N_MAX];
  FXX diag_c[BST_RW_ARRAY_N_MAX*BST_RW_ARRAY_N_MAX];

  FXX speed_cmd[BST_RW_ARRAY_N_MAX];
  FXX speed[BST_RW_ARRAY_N_MAX];
  FXX h_dot[BST_RW_ARRAY_N_MAX];

  int num = dev->RW_Number;

  // Generation of diag(I_RW_1,...,I_RW_N)
  for(int i=0;i<num;i++)
  {
    I_RW[i] = dev->rw[i].MoI;
    a[i]    = -2.0*mod_dev->rw[i].pt2_d/(mod_dev->rw[i].pt2_t*I_RW[i]);          // a = -(2*d/(t*I_RW))
    b[i]    = -1.0/(mod_dev->rw[i].pt2_t*mod_dev->rw[i].pt2_t);                  // b = -(1/t^2)
    c[i]    = mod_dev->rw[i].pt2_k/(mod_dev->rw[i].pt2_t*mod_dev->rw[i].pt2_t);  // c =  (k/t^2) 

    speed_cmd[i] = mod_dat->rw[i].spd_cmd;
    speed[i]     = dat->rw[i].sp;
    h_dot[i]     = dat->rw[i].trq;
#if 0
    BST_DEBUG_INFO("speed_cmd[%d]                = %10.6f [rpm]\r\n",i,speed_cmd[i]*60/(2*M_PIx));
    BST_DEBUG_INFO("h_dot[%d]                    = %10.6f [Nm]\r\n",i,h_dot[i]);
    BST_DEBUG_INFO("speed[%d]                    = %10.6f [rpm]\r\n",i,speed[i]*60/(2*M_PIx));
    BST_DEBUG_INFO("b*speed[%d] + c*speed_cmd[%d]= %10.6f [Nm/s]\r\n",i,i,b[i]*speed[i] + c[i]*speed_cmd[i]);
    BST_DEBUG_INFO("a*h_dot[%d]                  = %10.6f [Nm/s]\r\n",i,a[i]*h_dot[i]);
#endif
  }

  if(bst_matrixx_create_diag_mat(diag,I_RW,num,num,num)!=0) return 1; // Generation of diag(I_RW)
  if(bst_matrixx_create_diag_mat(diag_a,a,num,num,num)!=0)  return 1; // Generation of diag(a)
  if(bst_matrixx_create_diag_mat(diag_b,b,num,num,num)!=0)  return 1; // Generation of diag(b)
  if(bst_matrixx_create_diag_mat(diag_c,c,num,num,num)!=0)  return 1; // Generation of diag(c)

  // Computation of a*h_dot
  if(bst_matrixx_mul(prod_mat1,diag_a,h_dot,num,num,1)!=0)  return 1; // diag_a*h_dot

  // Computation of b*speed
  if(bst_matrixx_mul(prod_mat2,diag_b,speed,num,num,1)!=0)  return 1; // diag_b*speed

  // Computation of c*speed_cmd
  if(bst_matrixx_mul(prod_mat3,diag_c,speed_cmd,num,num,1)!=0) return 1; // diag_c*speed_cmd

  // Computation of dh_dot = diag(I_RW_1,...,I_RW_N)*[a*h_dot + b*speed + c*speed_cmd]
  if(bst_matrixx_add(add_mat1,prod_mat1,prod_mat2,num,1)!=0) return 1;
  if(bst_matrixx_add(add_mat2,add_mat1,prod_mat3,num,1)!=0)  return 1;
  if(bst_matrixx_mul(dh_dot,diag,add_mat2,num,num,1)!=0)     return 1;

#if 0
  BST_DEBUG_INFO("t_dot[0]                       = %10.6f [Nm/s]\r\n",dh_dot[0]);
#endif

  return 0;
}

// **************************************************************************************
// **************************************************************************************
// **************************************************************************************

/* dh/dt = I_RW*w_dot_in */

bst_ret_t bst_rwa_dynamics_iacds_trq(FXX *h_dot,
                                     FXX *w_dot_in,
                                     bst_rw_array_dev_param_t *dev)
{
  FXX I_RW[BST_RW_ARRAY_N_MAX];
  FXX diag[BST_RW_ARRAY_N_MAX*BST_RW_ARRAY_N_MAX];
  int num = dev->RW_Number;

  for(int i=0;i<num;i++)
  {
    I_RW[i] = dev->rw[i].MoI;
  }
  if(bst_matrixx_create_diag_mat(diag, I_RW, num, num, num)!=0) return 1;
  if(bst_matrixx_mul(h_dot, diag, w_dot_in, num, num, 1)!=0)    return 1;

  return 0;
}
