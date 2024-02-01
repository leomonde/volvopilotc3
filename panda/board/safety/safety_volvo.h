/*
Volvo Electronic Control Units abbreviations and network topology
Platforms C1

Look in selfdrive/car/volvo/values.py for more information.
*/

// Globals
// diagnostic msgs
#define MSG_DIAG_CEM 0x726
#define MSG_DIAG_PSCM 0x730
#define MSG_DIAG_FSM 0x764
#define MSG_DIAG_CVM 0x793
#define MSG_DIAG_BROADCAST 0x7df

// msg ids
#define MSG_BTNS_VOLVO_C1 0x10      // Steering wheel buttons
#define MSG_FSM0_VOLVO_C1 0x30      // ACC status message
#define MSG_FSM1_VOLVO_C1 0xd0      // LKA steering message
#define MSG_PSCM1_VOLVO_C1 0x125    // Steering angle from servo
#define MSG_ACC_PEDAL_VOLVO_C1 0x55 // Gas pedal
#define MSG_SPEED_VOLVO_C1 0x130    // Speed signal

// platform eucd
// msg ids
#define MSG_FSM0_VOLVO_V60 0x51  // ACC status message
#define MSG_FSM1_VOLVO_V60 0x260
#define MSG_FSM2_VOLVO_V60 0x262 // LKA steering message
#define MSG_FSM3_VOLVO_V60 0x270
#define MSG_FSM4_VOLVO_V60 0x31a
#define MSG_FSM5_VOLVO_V60 0x3fd
#define MSG_PSCM1_VOLVO_V60 0x246
#define MSG_ACC_PEDAL_VOLVO_V60 0x20 // Gas pedal
#define MSG_BTNS_VOLVO_V60 0x127     // Steering wheel buttons


// safety params
const SteeringLimits VOLVO_STEERING_LIMITS = {
  .enforce_angle_error = true,
  .inactive_angle_is_zero = true,
  .angle_deg_to_can = 1/.04395,  // 22.753... inverse of dbc scaling 
  .angle_rate_up_lookup = {
    {7., 17., 36.},
    {2, .25, .1}
  },
  .angle_rate_down_lookup = {
    {7., 17., 36.},
    {2, .25, .1}
  },
};

// TX checks
// platform eucd
const CanMsg VOLVO_TX_MSGS[] = { {MSG_FSM0_VOLVO_V60, 0, 8}, {MSG_FSM1_VOLVO_V60, 0, 8},
                                       {MSG_FSM2_VOLVO_V60, 0, 8}, {MSG_FSM3_VOLVO_V60, 0, 8},
                                       {MSG_FSM4_VOLVO_V60, 0, 8}, {MSG_FSM5_VOLVO_V60, 0, 8},
                                       {MSG_PSCM1_VOLVO_V60, 2, 8},
                                       {MSG_BTNS_VOLVO_V60, 0, 8},
                                       {MSG_DIAG_FSM, 2, 8}, {MSG_DIAG_PSCM, 0, 8},
                                       {MSG_DIAG_CEM, 0, 8}, {MSG_DIAG_CVM, 0, 8},
                                       {MSG_DIAG_BROADCAST, 0, 8}, {MSG_DIAG_BROADCAST, 2, 8},
                                    };
const int VOLVO_TX_MSGS_LEN = sizeof(VOLVO_TX_MSGS) / sizeof(VOLVO_TX_MSGS[0]);

// expected_timestep in microseconds between messages.
// WD timeout in external black panda if monitoring all messages.
// Works fine in C3.
AddrCheckStruct volvo_checks[] = {
  {.msg = {{MSG_PSCM1_VOLVO_V60,     0, 8, .check_checksum = false, .expected_timestep = 20000U}}},
  {.msg = {{MSG_FSM0_VOLVO_V60,      2, 8, .check_checksum = false, .expected_timestep = 10000U}}},
  {.msg = {{MSG_ACC_PEDAL_VOLVO_V60, 0, 8, .check_checksum = false, .expected_timestep = 10000U}}},
};

#define VOLVO_RX_CHECKS_LEN sizeof(volvo_checks) / sizeof(volvo_checks[0])
addr_checks volvo_rx_checks = {volvo_checks, VOLVO_RX_CHECKS_LEN};

static const addr_checks* volvo_init(uint16_t param) {
  UNUSED(param);
  return &volvo_rx_checks;
}

static int volvo_rx_hook(CANPacket_t *to_push) {

  bool valid = addr_safety_check(to_push, &volvo_rx_checks,
                                 NULL, NULL, NULL);

  if( valid ) {
    int bus = GET_BUS(to_push);
    int addr = GET_ADDR(to_push);

    // check acc status
    if( (addr == MSG_FSM0_VOLVO_V60) && (bus == 2) ) {
      giraffe_forward_camera_volvo = 1;
      int acc_status = (GET_BYTE(to_push, 2) & 0x07);
      bool acc_active = (acc_status >= 6) ? true : false;

      // only allow lateral control when acc active
      if( acc_active && !acc_active_prev_volvo ) {
        controls_allowed = 1;
      }
      if( !acc_active ) {
        controls_allowed = 0;
      }
      acc_active_prev_volvo = acc_active;
    }

    // Disengage when accelerator pedal pressed
    if( (addr == MSG_ACC_PEDAL_VOLVO_V60) && (bus == 0) ) {
      int acc_ped_val = ((GET_BYTE(to_push, 2) & 0x03) << 8) | GET_BYTE(to_push, 3);
      if( (acc_ped_val > 100) && (acc_ped_val_prev <= 100) ) {
        controls_allowed = 0;
      }
      acc_ped_val_prev = acc_ped_val;
    }

    // dont forward if message is on bus 0
    if( (addr == MSG_FSM0_VOLVO_V60) && (bus == 0) ) {
      giraffe_forward_camera_volvo = 0;
    }

    // If LKA msg is on bus 0, then relay is unexpectedly closed
    if( (safety_mode_cnt > RELAY_TRNS_TIMEOUT) && (addr == MSG_FSM2_VOLVO_V60) && (bus == 0) ) {
      relay_malfunction_set();
    }
  }
  return valid;
}


static int volvo_tx_hook(CANPacket_t *to_send) {

  //int bus = GET_BUS(to_send);
  //int addr = GET_ADDR(to_send);

  int tx = 1;

  if ( !msg_allowed(to_send, VOLVO_TX_MSGS, VOLVO_TX_MSGS_LEN) || relay_malfunction ) {
    tx = 0;
  }

  return tx;
}

static int volvo_fwd_hook(int bus_num, int addr) {
  int bus_fwd = -1; // fallback to do not forward
  int addr = GET_ADDR(to_fwd);

  if( !relay_malfunction && giraffe_forward_camera_volvo ) {
    if( bus_num == 0 ){
      bool block_msg = (addr == MSG_PSCM1_VOLVO_V60);
      //bool allw_msg = val_in_arr(addr, ALLOWED_MSG_EUCD, ALLOWED_MSG_EUCD_LEN); // block not relevant msgs
      bus_fwd = block_msg ? -1 : 2;  // forward bus 0 -> 2
    }

    if( bus_num == 2 ) {
      bool block_msg = (addr == MSG_FSM2_VOLVO_V60);  // block if lkas msg
      if( !block_msg ) {
        bus_fwd = 0; // forward bus 2 -> 0
      }
    }
  }
  return bus_fwd;
}

const safety_hooks volvo_hooks = {
  .init = volvo_init,
  .rx = volvo_rx_hook,
  .tx = volvo_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = volvo_fwd_hook,
};
