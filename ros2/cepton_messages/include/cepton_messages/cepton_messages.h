#pragma once

/**
 * Use for indexing into the voltage field of CeptonPanic::voltage_out_of_range
 */
enum {
  VoltageFault_Komodo_0_8v = 0,
  VoltageFault_Komodo_1_0v,
  VoltageFault_Komodo_1_8v,
  VoltageFault_Komodo_3_3v,
  VoltageFault_Motor_5_3v,
  VoltageFault_OM_5_3v,
  VoltageFault_Laser_32v
};
