/*****************************************************************************
 *
 *  Copyright (C) 2007-2009  Florian Pose, Ingenieurgemeinschaft IgH
 *
 *  This file is part of the IgH EtherCAT Master.
 *
 *  The IgH EtherCAT Master is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License version 2, as
 *  published by the Free Software Foundation.
 *
 *  The IgH EtherCAT Master is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 *  Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with the IgH EtherCAT Master; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 ****************************************************************************/

#include <iostream>
#include <bitset>
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h> /* clock_gettime() */
#include <sys/mman.h> /* mlockall() */
#include <sched.h> /* sched_setscheduler() */

/****************************************************************************/

#include "ecrt.h"


/****************************************************************************/

/* Constants */
#define NSEC_PER_SEC (1000000000L)
#define FREQUENCY 1000

/****************************************************************************/

/** Task period in ns. */
#define PERIOD_NS (NSEC_PER_SEC / FREQUENCY)

#define MAX_SAFE_STACK (8 * 1024) /* The maximum stack size which is
                                     guranteed safe to access without
                                     faulting */

/****************************************************************************/

volatile sig_atomic_t stop;

void inthand(int signum) {
    stop = 1;
}

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

/****************************************************************************/

// process data
static uint8_t *domain1_pd = NULL;

#define BusCouplerPos  0, 0
#define DigInSlavePos  0, 1
#define DigOutSlavePos 0, 2
#define MotorSlavePos 0, 3

#define Beckhoff_EK1100 0x00000002, 0x044c2c52
#define Beckhoff_EL1008 0x00000002, 0x03f03052
#define Beckhoff_EL2008 0x00000002, 0x07d83052
#define Lichuan_TLC57E 0x00000a79, 0x00003000

// offsets for PDO entries
static unsigned int off_dig_in;
static unsigned int off_dig_out;
static unsigned int off_motor_operation_mode;
static unsigned int off_motor_operation_status;
static unsigned int off_motor_control_word;
static unsigned int off_motor_control_status;
static unsigned int off_motor_target_velocity;

const static ec_pdo_entry_reg_t domain1_regs[] = {
    {DigInSlavePos,  Beckhoff_EL1008, 0x6000, 1, &off_dig_in},
    {DigOutSlavePos, Beckhoff_EL2008, 0x7000, 1, &off_dig_out},
    {MotorSlavePos, Lichuan_TLC57E, 0x6040, 0, &off_motor_control_word},
    {MotorSlavePos, Lichuan_TLC57E, 0x6041, 0, &off_motor_control_status},
    {MotorSlavePos, Lichuan_TLC57E, 0x6060, 0, &off_motor_operation_mode},
    {MotorSlavePos, Lichuan_TLC57E, 0x6061, 0, &off_motor_operation_status},
    {MotorSlavePos, Lichuan_TLC57E, 0x60ff, 0, &off_motor_target_velocity},
    // {MotorSlavePos, Lichuan_TLC57E, 0x603f, 0, &off_motor_error_code},
    // {MotorSlavePos, Lichuan_TLC57E, 0x6083, 0, &off_motor_acceleration},
    // {MotorSlavePos, Lichuan_TLC57E, 0x6084, 0, &off_motor_deacceleration},
};

/* Master 0, Slave 1, "EL1008"
 * Vendor ID:       0x00000002
 * Product code:    0x03f03052
 * Revision number: 0x00120000
 */

ec_pdo_entry_info_t slave_1_pdo_entries[] = {
    {0x6000, 0x01, 1}, /* Input */
    {0x6010, 0x01, 1}, /* Input */
    {0x6020, 0x01, 1}, /* Input */
    {0x6030, 0x01, 1}, /* Input */
    {0x6040, 0x01, 1}, /* Input */
    {0x6050, 0x01, 1}, /* Input */
    {0x6060, 0x01, 1}, /* Input */
    {0x6070, 0x01, 1}, /* Input */
};

ec_pdo_info_t slave_1_pdos[] = {
    {0x1a00, 1, slave_1_pdo_entries + 0}, /* Channel 1 */
    {0x1a01, 1, slave_1_pdo_entries + 1}, /* Channel 2 */
    {0x1a02, 1, slave_1_pdo_entries + 2}, /* Channel 3 */
    {0x1a03, 1, slave_1_pdo_entries + 3}, /* Channel 4 */
    {0x1a04, 1, slave_1_pdo_entries + 4}, /* Channel 5 */
    {0x1a05, 1, slave_1_pdo_entries + 5}, /* Channel 6 */
    {0x1a06, 1, slave_1_pdo_entries + 6}, /* Channel 7 */
    {0x1a07, 1, slave_1_pdo_entries + 7}, /* Channel 8 */
};

ec_sync_info_t slave_1_syncs[] = {
    {0, EC_DIR_INPUT, 8, slave_1_pdos + 0, EC_WD_DISABLE},
    {0xff}
};

/* Master 0, Slave 2, "EL2008"
 * Vendor ID:       0x00000002
 * Product code:    0x07d83052
 * Revision number: 0x00120000
 */

ec_pdo_entry_info_t slave_2_pdo_entries[] = {
    {0x7000, 0x01, 1}, /* Output */
    {0x7010, 0x01, 1}, /* Output */
    {0x7020, 0x01, 1}, /* Output */
    {0x7030, 0x01, 1}, /* Output */
    {0x7040, 0x01, 1}, /* Output */
    {0x7050, 0x01, 1}, /* Output */
    {0x7060, 0x01, 1}, /* Output */
    {0x7070, 0x01, 1}, /* Output */
};

ec_pdo_info_t slave_2_pdos[] = {
    {0x1600, 1, slave_2_pdo_entries + 0}, /* Channel 1 */
    {0x1601, 1, slave_2_pdo_entries + 1}, /* Channel 2 */
    {0x1602, 1, slave_2_pdo_entries + 2}, /* Channel 3 */
    {0x1603, 1, slave_2_pdo_entries + 3}, /* Channel 4 */
    {0x1604, 1, slave_2_pdo_entries + 4}, /* Channel 5 */
    {0x1605, 1, slave_2_pdo_entries + 5}, /* Channel 6 */
    {0x1606, 1, slave_2_pdo_entries + 6}, /* Channel 7 */
    {0x1607, 1, slave_2_pdo_entries + 7}, /* Channel 8 */
};

ec_sync_info_t slave_2_syncs[] = {
    {0, EC_DIR_OUTPUT, 8, slave_2_pdos + 0, EC_WD_ENABLE},
    {0xff}
};

/* Master 0, Slave 3
 * Vendor ID:       0x00000a79
 * Product code:    0x00003000
 * Revision number: 0x00000001
 */

ec_pdo_entry_info_t slave_3_pdo_entries[] = {
    {0x6060, 0x00, 8}, /* Modes of operation */
    {0x6040, 0x00, 16},
    {0x607a, 0x00, 32},
    {0x60ff, 0x00, 32},
    {0x6060, 0x00, 8},
    {0x0000, 0x00, 8}, /* Gap */
    {0x6041, 0x00, 16},
    {0x6064, 0x00, 32},
    {0x606c, 0x00, 32},
    {0x6061, 0x00, 8},
    {0x0000, 0x00, 8}, /* Gap */
};

ec_pdo_info_t slave_3_pdos[] = {
    {0x1a01, 0, NULL}, /* TxPDO 2 */
    {0x1601, 1, slave_3_pdo_entries + 0}, /* RxPDO 2 */
    {0x1600, 5, slave_3_pdo_entries + 1},
    {0x1a00, 5, slave_3_pdo_entries + 6},
};

ec_sync_info_t slave_3_syncs[] = {
    {0, EC_DIR_OUTPUT, 2, slave_3_pdos + 0, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_3_pdos + 2, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, slave_3_pdos + 3, EC_WD_DISABLE},
    {0xff}
};

/****************************************************************************/



/* Application Specific Variables */
static unsigned int shared_counter = 0;
static unsigned int blink = 0;
static unsigned int counter_val = 0;

static unsigned int state = 0;
static unsigned int prev_state = -1;

static unsigned int motor_control_status = 255;
static unsigned short int motor_operation_status = 255;
static unsigned int motor_speed = 0;
/****************************************************************************/

void check_domain1_state(void)
{
    ec_domain_state_t ds;

    ecrt_domain_state(domain1, &ds);

    if (ds.working_counter != domain1_state.working_counter) {
        printf("Domain1: WC %u.\n", ds.working_counter);
    }
    if (ds.wc_state != domain1_state.wc_state) {
        printf("Domain1: State %u.\n", ds.wc_state);
    }

    domain1_state = ds;
}

/****************************************************************************/

void check_master_state(void)
{
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding) {
        std::cout << ms.slaves_responding << " slave(s) responding." << std::endl;
    }
    if (ms.al_states != master_state.al_states) {
        std::cout << "AL states: " << std::bitset<8>(ms.al_states) << std::endl;
    }
    if (ms.link_up != master_state.link_up) {
        std::cout << "Link is " << (ms.link_up ? "up" : "down") << "." << std::endl;
    }

    master_state = ms;
}

/****************************************************************************/

void cyclic_task()
{
    // receive process data
    ecrt_master_receive(master);
    ecrt_domain_process(domain1);

    // check process data state
    check_domain1_state();
    motor_operation_status = 255;
    motor_control_status = 255;
    motor_operation_status = EC_READ_U8(domain1_pd + off_motor_operation_status);
    motor_control_status = EC_READ_U16(domain1_pd + off_motor_control_status);
    
    bool execute_shared_counter = false;
    if (shared_counter) {
        shared_counter--;
    } else { // do this at 1 Hz
        shared_counter = 1000;
        execute_shared_counter = true;
        check_master_state();
        motor_speed += 10000;
        if (motor_speed > 150000) {
            motor_speed = 0;
        }
    }

    switch (state)
    {
    case 0:
        if(state != prev_state) {
            std::cout << "State 0: Reset the drive." << std::endl;
            prev_state = state;
        }
        EC_WRITE_U16(domain1_pd + off_motor_control_word, 0x0); 
        std::cout << "[State " << state << "]"<< " Motor Status: " 
                  << std::bitset<16>(motor_control_status) 
                  << std::endl;
        if (execute_shared_counter) {
            std::cout << "[State " << state << "]"<< " Motor Operation Status: " << motor_operation_status << std::endl;
            // state = 10;
        }
        if (motor_control_status != 0){
            state = 10;
        }
        break;
    case 10:
        if(state != prev_state) {
            std::cout << "State " << state << ": Set Motor Operation Mode to Velocity." << std::endl;
            prev_state = state;
        }
        
        if(motor_operation_status == 0x03){
            state = 11;
        }
        if (execute_shared_counter) {
            std::cout << "[State " << state << "] "<< "#################################" << std::endl;
            std::cout << "[State " << state << "]"<< " Sent Motor Operation Command (0x6060): " << std::bitset<8>(EC_READ_U8(domain1_pd + off_motor_operation_mode)) << std::endl;
            std::cout << "[State " << state << "]"<< " Motor Operation Status (0x6061): " << std::bitset<8>(motor_operation_status) << std::endl;
            std::cout << "[State " << state << "]"<< " Motor Control Status (0x6041): " 
            << std::bitset<16>(motor_control_status) 
            << std::endl;
        }
        
        EC_WRITE_U8(domain1_pd + off_motor_operation_mode, 0x3); // Set motor to velocity mode
        break;
    case 11:
        if(state != prev_state) {
            std::cout << "State " << state << ": Set 0x6 to 0x6040" << std::endl;
            prev_state = state;
        }
        
        if (execute_shared_counter) {
            std::cout << "[State " << state << "]"<< " Motor Operation Status: " << motor_operation_status << std::endl;
            std::cout << "[State " << state << "]"<< " Motor Status: " 
            << std::bitset<16>(motor_control_status) 
            << std::endl;
            if (motor_operation_status != 0x03) {
                state = 12;
            }
        }
        EC_WRITE_U16(domain1_pd + off_motor_control_word, 0x6); // Set motor to velocity mode
        break;
    case 12:
        if(state != prev_state) {
            std::cout << "State " << state << ": Set 0x7 to 0x6040" << std::endl;
            prev_state = state;
        }
        if (execute_shared_counter) {
            std::cout << "[State " << state << "]"<< " Motor Operation Status: " << motor_operation_status << std::endl;
            std::cout << "[State " << state << "]"<< " Motor Status: " 
            << std::bitset<16>(motor_control_status) 
            << std::endl;
            if (motor_operation_status != 0x03) {
                state = 13;
            }
        }
        EC_WRITE_U16(domain1_pd + off_motor_control_word, 0x7); // Set motor to velocity mode
        break;
    case 13:
        if(state != prev_state) {
            std::cout << "State " << state << ": Set 0xF to 0x6040" << std::endl;
            prev_state = state;
        }
       
        if(motor_operation_status == 0x03){
            state = 1000;
        }
        if (execute_shared_counter) {
            std::cout << "[State " << state << "]"<< " Motor Operation Status: " << motor_operation_status << std::endl;
            std::cout << "[State " << state << "]"<< " Motor Status: " 
            << std::bitset<16>(motor_control_status) 
            << std::endl;
            if (motor_operation_status != 0x03) {
                state = 10;
            }
        }
        EC_WRITE_U16(domain1_pd + off_motor_control_word, 0xF); // Set motor to velocity mode
        break;
    case 1000:
        if(state != prev_state) {
            std::cout << "State " << state << ": Motor Running" << std::endl;
            prev_state = state;
        }
        
        if (execute_shared_counter) {
            std::cout << "[State " << state << "]"<< " Motor Operation Status: " << motor_operation_status << std::endl;
            std::cout << "[State " << state << "]"<< " Motor Status: " 
            << std::bitset<16>(motor_control_status) 
            << std::endl;
        }
        EC_WRITE_U32(domain1_pd + off_motor_target_velocity, 2000);
        break;
    
    }

    ecrt_domain_queue(domain1);
    ecrt_master_send(master);
}

/****************************************************************************/

void stack_prefault(void)
{
    unsigned char dummy[MAX_SAFE_STACK];

    memset(dummy, 0, MAX_SAFE_STACK);
}

/****************************************************************************/

int main(int argc, char **argv)
{
    ec_slave_config_t *sc;
    struct timespec wakeup_time;
    int ret = 0;
    signal(SIGINT, inthand);
    master = ecrt_request_master(0);
    if (!master) {
        return -1;
    }

    domain1 = ecrt_master_create_domain(master);
    if (!domain1) {
        return -1;
    }

    if (!(sc = ecrt_master_slave_config(master, DigInSlavePos, Beckhoff_EL1008))) {
        std::cerr << "Failed to get slave configuration." << std::endl;
        return -1;
    }
    if (ecrt_slave_config_pdos(sc, EC_END, slave_1_syncs)) {
        std::cerr << "Failed to configure PDOs." << std::endl;
        return -1;
    }
    if (!(sc = ecrt_master_slave_config(master, DigOutSlavePos, Beckhoff_EL2008))) {
        std::cerr << "Failed to get slave configuration." << std::endl;
        return -1;
    }
    if (ecrt_slave_config_pdos(sc, EC_END, slave_2_syncs)) {
        std::cerr << "Failed to configure PDOs." << std::endl;
        return -1;
    }
    if (!(sc = ecrt_master_slave_config(master, MotorSlavePos, Lichuan_TLC57E))) {
        std::cerr << "Failed to get slave configuration." << std::endl;
        return -1;
    }
    if (ecrt_slave_config_pdos(sc, EC_END, slave_3_syncs)) {
        std::cerr << "Failed to configure PDOs." << std::endl;
        return -1;
    }

    // Create configuration for bus coupler
    sc = ecrt_master_slave_config(master, BusCouplerPos, Beckhoff_EK1100);
    if (!sc) {
        return -1;
    }

    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) {
        std::cerr << "PDO entry registration failed!" << std::endl;
        return -1;
    }

    std::cout << "Activating master..." << std::endl;
    if (ecrt_master_activate(master)) {
        return -1;
    }

    if (!(domain1_pd = ecrt_domain_data(domain1))) {
        return -1;
    }

    /* Set priority */

    struct sched_param param = {};
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);

    std::cout << "Using priority " << param.sched_priority << "." << std::endl;
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        std::cerr << "Warning: Failed to set scheduler: " << strerror(errno) << std::endl;
        return -1;
    }

    /* Lock memory */

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        std::cerr << "Warning: Failed to lock memory: " << strerror(errno) << std::endl;
    }

    stack_prefault();

    std::cout << "Starting RT task with dt=" << PERIOD_NS << " ns." << std::endl;

    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
    wakeup_time.tv_sec += 1; /* start in future */
    wakeup_time.tv_nsec = 0;

    while (!stop) {
        ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                &wakeup_time, NULL);
        if (ret) {
            std::cerr << "clock_nanosleep(): " << strerror(ret) << std::endl;
            break;
        }

        cyclic_task();

        wakeup_time.tv_nsec += PERIOD_NS;
        while (wakeup_time.tv_nsec >= NSEC_PER_SEC) {
            wakeup_time.tv_nsec -= NSEC_PER_SEC;
            wakeup_time.tv_sec++;
        }
    }

    std::cout << "Exiting..." << std::endl;
    EC_WRITE_U32(domain1_pd + off_motor_target_velocity, 0);
    ecrt_domain_queue(domain1);
    ecrt_master_send(master);
    ecrt_release_master(master);
    return 0;
}

/****************************************************************************/
