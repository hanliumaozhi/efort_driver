//
// Created by han on 16-12-27.
//

#include "efort_driver/EcatAdmin.h"

ec_master_t* EcatAdmin::master = nullptr;
ec_master_state_t EcatAdmin::master_state = {};

ec_domain_t* EcatAdmin::domain1 = nullptr;
ec_domain_state_t EcatAdmin::domain1_state = {};

uint8_t* EcatAdmin::domain1_pd = nullptr;

EcatAdmin::slaves_map EcatAdmin::slaves_dict;

int EcatAdmin::start_for_ros_control()
{
    master = ecrt_request_master(0);

    if (!master) {
        ROS_INFO("master init error\n");
        goto out_return;
    }

    domain1 = ecrt_master_create_domain(master);
    if (!domain1) {
        ROS_INFO("domain1 init error\n");
        goto out_release_master;
    }

    for(auto & item : slaves_dict){
        auto &drive = item.second;
        if (!(drive->sc_ = ecrt_master_slave_config(master, drive->get_alias(), drive->get_position(), drive->get_vendor_id(), drive->get_product_code()))) {
            ROS_INFO("Could not get slave configuration for Drive at bus position = %u\n", (unsigned int)drive->get_position());
            goto out_release_master;
        }
        /* Clear RxPdo */
        ecrt_slave_config_sdo8( drive->sc_, 0x1C12, 0, 0 ); /* clear sm pdo 0x1c12 */
        ecrt_slave_config_sdo8( drive->sc_, 0x1600, 0, 0 ); /* clear RxPdo 0x1600 */
        ecrt_slave_config_sdo8( drive->sc_, 0x1601, 0, 0 ); /* clear RxPdo 0x1601 */
        ecrt_slave_config_sdo8( drive->sc_, 0x1602, 0, 0 ); /* clear RxPdo 0x1602 */
        ecrt_slave_config_sdo8( drive->sc_, 0x1603, 0, 0 ); /* clear RxPdo 0x1603 */

        /* Define RxPdo */
        ecrt_slave_config_sdo8( drive->sc_, 0x1600, 0, 1 ); /* set number of PDO entries for 0x1600 */
        ecrt_slave_config_sdo32( drive->sc_, 0x1600, 1, 0x60400010 ); /*  control word */
        ecrt_slave_config_sdo8( drive->sc_, 0x1601, 0, 2 ); /* set number of PDO entries for 0x1601 */
        ecrt_slave_config_sdo32( drive->sc_, 0x1601, 1, 0x607A0020 ); /*  target position*/
        ecrt_slave_config_sdo32( drive->sc_, 0x1601, 2, 0x60810020 ); /*  profile_velocity */

        /* clear TxPdo */
        ecrt_slave_config_sdo8( drive->sc_, 0x1C13, 0, 0 ); /* clear sm pdo 0x1c13 */
        ecrt_slave_config_sdo8( drive->sc_, 0x1A00, 0, 0 ); /* clear TxPdo 0x1A00 */
        ecrt_slave_config_sdo8( drive->sc_, 0x1A01, 0, 0 ); /* clear TxPdo 0x1A01 */
        ecrt_slave_config_sdo8( drive->sc_, 0x1A02, 0, 0 ); /* clear TxPdo 0x1A02 */
        ecrt_slave_config_sdo8( drive->sc_, 0x1A03, 0, 0 ); /* clear TxPdo 0x1A03 */
        /* Define TxPdo */
        ecrt_slave_config_sdo32( drive->sc_, 0x1A00, 1, 0x60410010 ); /* 0x6041:0/16bits, status word */
        ecrt_slave_config_sdo8( drive->sc_, 0x1A00, 0, 1 ); /* set number of PDO entries for 0x1A00 */
        ecrt_slave_config_sdo32( drive->sc_, 0x1A01, 1, 0x606C0020 );  /* 0x606c:0/32bits, act velocity */
        ecrt_slave_config_sdo32( drive->sc_, 0x1A01, 2, 0x60640020 );  /* 0x6063:0/32bits, act position */
        ecrt_slave_config_sdo8( drive->sc_, 0x1A01, 0, 2 ); /* set number of PDO entries for 0x1A01 */

        ecrt_slave_config_sdo8( drive->sc_, 0x6060, 0, 8 );

        if (ecrt_slave_config_pdos(drive->sc_, EC_END, cdhd_syncs)) {
            ROS_INFO("Could not configure PDOs for Drive at bus position = %u\n", (unsigned int)drive->get_position());
            goto out_release_master;
        }
    }

    if (ecrt_domain_reg_pdo_entry_list(domain1, &(get_pdo_entry_regs_with_terminated()[0]))) {
        ROS_INFO("Could not register PDO entries. \n");
        goto out_release_master;
    }


    ROS_INFO("Activating the master. \n");

    if (ecrt_master_activate(master)) {
        ROS_INFO("Could not activate the ethercat master. \n");
        goto out_release_master;
    }

    domain1_pd = ecrt_domain_data(domain1);


    return 0;

    out_release_master:
    ROS_INFO("releasing the master\n");
    ecrt_release_master(master);
    out_return:
    ROS_INFO("error occur!!!\n");
    return 1;
}

std::vector<ec_pdo_entry_reg_t> EcatAdmin::get_pdo_entry_regs_with_terminated()
{
    std::vector<ec_pdo_entry_reg_t> regs;

    for (auto & item : slaves_dict) {
        auto &drive = item.second;

        for (int i = 0; i < 6; ++i) {
            ec_pdo_entry_reg_t entry;
            entry.alias = drive->get_alias();
            entry.position = drive->get_position();
            entry.vendor_id = drive->get_vendor_id();
            entry.product_code = drive->get_product_code();

            entry.index = drive->pdo_index_[i];
            entry.subindex = drive->pdo_subindex_[i];
            entry.offset = &Motor::regs[drive->index_to_offset_list_[i]];
            entry.bit_position = 0;
            regs.push_back(entry);
        }
    }

    ec_pdo_entry_reg_t terminator;
    regs.push_back(terminator);

    return (regs);
}

void EcatAdmin::add_motor(int joint_no, std::string joint_name, uint16_t alias, uint16_t position, uint32_t vendor_id, uint32_t product_code)
{
    std::shared_ptr<Motor> motor_ptr = std::make_shared<Motor>(joint_no, joint_name, alias, position, vendor_id, product_code);

    slaves_dict[joint_no] = motor_ptr;
}

void EcatAdmin::shutdown()
{
    ecrt_release_master(master);
}

void EcatAdmin::check_domain_state(void)
{
    ec_domain_state_t ds = {};

    ecrt_domain_state(domain1, &ds);

    if (ds.working_counter != domain1_state.working_counter) {
        ROS_INFO("Domain1: WC %u.\n", ds.working_counter);
    }

    if (ds.wc_state != domain1_state.wc_state) {
        ROS_INFO("Domain1: State %u.\n", ds.wc_state);
    }

    domain1_state = ds;
}

void EcatAdmin::check_master_state(void)
{
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding) {
        ROS_INFO("%u slave(s).\n", ms.slaves_responding);
    }

    if (ms.al_states != master_state.al_states) {
        ROS_INFO("AL states: 0x%02X.\n", ms.al_states);
    }

    if (ms.link_up != master_state.link_up) {
        ROS_INFO("Link is %s.\n", ms.link_up ? "up" : "down");
    }

    master_state = ms;
}