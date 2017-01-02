//
// Created by han on 16-12-27.
//

#ifndef ROS_COMMON_H
#define ROS_COMMON_H

#include <ecrt.h>

// used 6 * 6 = 36 in fact
#define ECAT_REG_TABLE_SIZE 64
#define PI 3.14159265359

#define TASK_FREQUENCY       125  /* Hz */ 
#define TIMEOUT_CLEAR_ERROR  (2*TASK_FREQUENCY) /* clearing error timeout */

#define AKD_STS_MASK       0xAFF  /* mask to remove manufacturer special bits and target_reached */
#define AKD_STS_SWION_DIS  0x250  /* switched on disabled */
#define AKD_STS_RDY_SWION  0x231  /* ready to switch on   */
#define AKD_STS_SWION_ENA  0x233  /* switched on enabled  */
#define AKD_STS_ERROR      0x218  /* error                */

#define AKD_CMD_ENA_QSTOP  0x00   /* enable quick stop   */
#define AKD_CMD_DIS_QSTOP  0x06   /* disable quick stop  */
#define AKD_CMD_ENA_SWION  0x07   /* enable switched  on */
#define AKD_CMD_ENA_OP     0x0F   /* enable operation    */
#define AKD_CMD_CLR_ERROR  0x80   /* clear error         */

#endif //ROS_COMMON_H
