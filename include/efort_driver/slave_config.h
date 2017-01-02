//
// Created by han on 16-12-27.
//

#ifndef ROS_SLAVE_CONFIG_H
#define ROS_SLAVE_CONFIG_H

#include <ecrt.h>

static ec_pdo_entry_info_t cdhd_pdo_entries[] = {
        { 0x6040, 0x00, 16 },
        { 0x607A, 0x00, 32 },
        { 0x6081, 0x00, 32 },
        { 0x6041, 0x00, 16 },
        { 0x6064, 0x00, 32 },
        { 0x606C, 0x00, 32 },
    };


static ec_pdo_info_t cdhd_pdos[] = {
        { 0x1600, 1, cdhd_pdo_entries + 0 },
        { 0x1601, 2, cdhd_pdo_entries + 1 },
        { 0x1a00, 1, cdhd_pdo_entries + 3 },
        { 0x1a01, 2, cdhd_pdo_entries + 4 },
    };

static ec_sync_info_t cdhd_syncs[] = {
        { 0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE },
        { 1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE },
        { 2, EC_DIR_OUTPUT, 2, cdhd_pdos + 0, EC_WD_DISABLE },
        { 3, EC_DIR_INPUT, 2, cdhd_pdos + 2, EC_WD_DISABLE },
        { 0xFF }
    };

#endif //ROS_SLAVE_CONFIG_H
