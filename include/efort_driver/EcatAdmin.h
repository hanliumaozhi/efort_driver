//
// Created by han on 16-12-27.
//

#ifndef ROS_ECATADMIN_H
#define ROS_ECATADMIN_H

#include <cstdint>
#include <memory>
#include <unordered_map>

#include <ros/console.h>

#include "Common.h"
#include "motor.h"

#include "slave_config.h"

class EcatAdmin {
public:
    using slaves_map = std::unordered_map<int, std::shared_ptr<Motor>>;


    static slaves_map slaves_dict;

    static ec_master_t *master;
    static ec_master_state_t master_state;

    static ec_domain_t *domain1;
    static ec_domain_state_t domain1_state;

    static uint8_t *domain1_pd;

    static int start_for_ros_control();

    static void add_motor(int joint_no, std::string joint_name, uint16_t alias, uint16_t position, uint32_t vendor_id, uint32_t product_code);

    static void shutdown();

    static void check_domain_state(void);

    static void check_master_state(void);

private:
    static std::vector<ec_pdo_entry_reg_t> get_pdo_entry_regs_with_terminated();

};


#endif //ROS_ECATADMIN_H
