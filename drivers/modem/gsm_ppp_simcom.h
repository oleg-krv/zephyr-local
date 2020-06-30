//
// Created by oleg on 25.06.20.
//

#ifndef STM32_BASE_GSM_PPP_SIMCOM_H
#define STM32_BASE_GSM_PPP_SIMCOM_H
#ifdef __cplusplus
extern "C" {
#endif

#include <device.h>


__syscall void modem_power_on(void);
__syscall void modem_power_off(void);
__syscall void modem_power_reset(void);
__syscall struct k_work_q* modem_get_worker(void);


#ifdef __cplusplus
}
#endif

#include <syscalls/gsm_ppp_simcom.h>
#endif //STM32_BASE_GSM_PPP_SIMCOM_H
