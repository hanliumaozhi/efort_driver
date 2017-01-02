//
// Created by han on 16-12-26.
//

#ifndef ROS_MOTOR_H
#define ROS_MOTOR_H

#include <cstdint>
#include <string>
#include <vector>

#include "Common.h"


//
//          radian * param_1 * param_2
// pulse = -----------------------------  + offset
//                 2*PI*param_3
//


class Motor {
public:
    Motor(int joint_no, std::string joint_name, uint16_t alias, uint16_t position, uint32_t vendor_id, uint32_t product_code);
    ~Motor() = default;
    Motor(const Motor&) = delete;

    uint16_t get_alias() const noexcept;
    uint16_t get_position() const noexcept;
    uint32_t get_vendor_id() const noexcept;
    uint32_t get_product_code() const noexcept;

    ec_slave_config_t *sc_;
    ec_slave_config_state_t slave_config_state_;
    ec_slave_config_state_t old_slave_config_state_;

    static unsigned int regs[ECAT_REG_TABLE_SIZE];
    static int current_index_;

    std::vector<int> index_to_offset_list_;
    std::vector<uint16_t> pdo_index_;
    std::vector<uint8_t> pdo_subindex_;

    int get_control_word() const noexcept;
    int get_r_position() const noexcept;
    int get_r_velocity() const noexcept;

    int get_status_word() const noexcept;
    int get_t_position() const noexcept;
    int get_t_velocity() const noexcept;

    void setting_trans(double offset, double param_1, double param_2, double param_3);

    int radian_to_pulse(double radian);

    double pulse_to_radian(int pulse);

private:

    int joint_no_;
    std::string joint_name_;
    uint16_t alias_;
    uint16_t position_;
    uint32_t vendor_id_;
    uint32_t product_code_;

    double offset_;
    double param_1_;
    double param_2_;
    double param_3_;

};


#endif //ROS_MOTOR_H
