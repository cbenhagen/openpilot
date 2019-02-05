int vw_ignition_started = 0;

void vw_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {
  int bus_number = (to_push->RDTR >> 4) & 0xFF;
  uint32_t addr;
  if (to_push->RIR & 4)
  {
    // Extended
    // Not looked at, but have to be separated
    // to avoid address collision
    addr = to_push->RIR >> 3;
  }
  else
  {
    // Normal
    addr = to_push->RIR >> 21;
  }

  if (addr == 0x3c0 && bus_number == 0) {
    uint32_t ign = (to_push->RDLR) & 0x200;
    vw_ignition_started = ign > 0;
  }
}

int vw_ign_hook() {
  return vw_ignition_started;
}

// FIXME
// *** all output safety mode ***

static void vw_init(int16_t param) {
  controls_allowed = 1;
}

static int vw_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {
  return true;
}

static int vw_tx_lin_hook(int lin_num, uint8_t *data, int len) {
  return true;
}

static int vw_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {
  
  
  // shifts bits from 29 to 11
  int32_t addr = to_fwd->RIR >> 21;
  
  // forward messages from car to extended
  if (bus_num == 0) {
    
    return 1; //extended 
    
  }
  // forward messages from extended to car
  else if (bus_num == 1) {
    
    //filter 0x126 from being forwarded
    if (addr == 0x126) {
      return -1;
    }
    
    return 0; //car 
  }

  // fallback to do not forward
  return -1;
}

const safety_hooks vw_hooks = {
  .init = vw_init,
  .rx = vw_rx_hook,
  .tx = vw_tx_hook,
  .tx_lin = vw_tx_lin_hook,
  .ignition = vw_ign_hook,
  .fwd = vw_fwd_hook,
};