/**
  ******************************************************************************
  * @file    ll_sys_startup.h
  * @author  MCD Application Team
  * @brief   Header for Link Layer startup interfaces
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifndef LL_SYS_STARTUP_H
#define LL_SYS_STARTUP_H

/* Link Layer system interface startup module functions  ************************************************/
/*vvv PATCH STMC-485 vvvvvvvvvv*/
// #if BLE
#ifdef BLE
/*^^^ END OF PATCH STMC-485 ^^^*/
void ll_sys_ble_cntrl_init(hst_cbk hostCallback);
#endif
void ll_sys_mac_cntrl_init(void);
void ll_sys_thread_init(void);
/*vvv PATCH STMC-485 vvvvvvvvvv*/
void ll_sys_dependencies_init(void);
void ll_sys_dependencies_deinit(void);
/*^^^ END OF PATCH STMC-485 ^^^*/

#endif /* LL_SYS_STARTUP_H */