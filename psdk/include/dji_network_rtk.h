/**
 ********************************************************************
 * @file    dji_network_rtk.h
 * @version V1.0.0
 * @date    2025/10/09
 * @brief   This is the header file for "dij_core.c", defining the structure and
 * (exported) function prototypes.
 *
 * @copyright (c) 2021 DJI. All rights reserved.
 *
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DJI_NETWORK_RTK_H
#define DJI_NETWORK_RTK_H

/* Includes ------------------------------------------------------------------*/
#include "dji_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/
/**
 * @brief The network rtk state.
 */
typedef enum
{
    DJI_LOG_IN = 0,  /*! Logging in (login in progress) */
    DJI_CONNECTING = 1,/*! Connecting to remote/server */
    DJI_TRANSFER = 2, /*! Data transfer in progress */
    DJI_RECONNECT = 3, /*! Reconnecting (attempting to restore connection) */
    DJI_BROKEN = 4, /*! Connection broken / disconnected from remote server */
    DJI_STOPPING = 5, /*! Stopping (shutdown in progress) */
    DJI_REBOOTING = 6,  /*! Rebooting: waiting for restart after switching source) */
} E_DjiNetworkRtkOnboardState;

/* Exported types ------------------------------------------------------------*/
/**
 * @brief Position of target point and other details returned by interface of requesting position.
 */
typedef struct {
    uint8_t account[16]; /*!< RTK account (username). Null-terminated string, up to 15 chars + '\0'. */
    uint8_t password[64]; /*!< RTK password. Null-terminated string, up to 63 chars + '\0'. */
    uint8_t host[64]; /*!< IP address of the NTRIP caster/host.MUST be a valid IPv4 or IPv6 address. Domain names (e.g., "example.com") are NOT supported.*/ 

    /*!< NTRIP port information. Often used to indicate coordinate system info for some providers.
     * Field MUST be a string. The string SHOULD represent an integer value in
     * the range 1 to 65535 (inclusive). Example valid values: "80", "443", "2101".*/
    uint8_t port[6];

     /*!< NTRIP mountpoint information. Typically configures RTCM system.
      * -Allowed characters: a-z, A-Z, 0-9, '-' (hyphen), '_' (underscore), '.' (dot).
      * -Disallowed characters include (but are not limited to): space, '@', '#', '$'.
      * -Recommended validation regex: /^[A-Za-z0-9._-]+$/ */
    uint8_t mountPoint[32];
} T_DjiNetworkRtkServiceConfig;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief Register a callback function to receive network RTK service state updates.
 * @note This callback is used to asynchronously obtain the state of the network RTK service.
 *       It is recommended to register the callback before starting the network RTK service.
 * @param callback [in] Pointer to the user-implemented callback function for handling received network RTK state data.
 * @return Execution result.
 */
typedef void (*DjiReceiveNetworkRtkStateCallback)(E_DjiNetworkRtkOnboardState state);
T_DjiReturnCode DjiNetworkRtk_RegReceiveNetworkRtkStateCallback(DjiReceiveNetworkRtkStateCallback callback);
/**
 * @brief Start the network RTK service.
 * @note
 * Behavior and usage notes:
 * - RTK service state updates are delivered via
 *   DjiPositioning_RegReceiveNetworkRtkStateCallback. Monitor that callback
 *   to observe state transitions (e.g., DJI_REBOOTING, DJI_TRANSFER).
 * - Note: If the current RTK source is NOT a custom network source, the first call
 *   to this API will require a device reboot to switch to the network RTK
 *   source. After the device reboots, you must call this API again to start
 *   the Network RTK service so that it takes effect.
 * - DJI_REBOOTING indicates that a reboot is required/underway to apply the
 *   change of RTK source. Note: the device will NOT reboot automatically;
 *   a manual reboot is required to complete the change.
 * - DJI_TRANSFER indicates that the RTK service is connected and data
 *   transfer between RTK endpoints has been established. DJI_TRANSFER does
 *   NOT guarantee that the RTK solution has converged to a usable fix.
 *   Convergence must be verified by subscribing to RTK positioning data and
 *   checking the quality/accuracy indicators provided in those data messages.
 *
 * Recommended call sequence:
 * 1. Call DjiPositioning_StartNetworkRtkService().
 * 2. If state callback reports DJI_REBOOTING, perform a MANUAL reboot of the device.
 * 3. After the manual reboot, call DjiPositioning_StartNetworkRtkService() again.
 * 4. Monitor state callback for DJI_TRANSFER and subscribe to RTK position
 *    data to verify convergence.
 * @param config [in] Pointer to a T_DjiNetworkRtkServiceConfig structure containing the configuration information for the network RTK service.
 * @return Execution result.
 */
T_DjiReturnCode DjiNetworkRtk_StartService(const T_DjiNetworkRtkServiceConfig* config);

/**
 * @brief Stop the network RTK service.
 * @note After calling this function, the device will disconnect from the network RTK service and stop receiving high-precision positioning data.
 * @param None
 * @return Execution result.
 */
T_DjiReturnCode DjiNetworkRtk_StopService(void);

#ifdef __cplusplus
}
#endif

#endif // DIJ_CORE_H
/************************ (C) COPYRIGHT DJI Innovations *******END OF FILE******/
