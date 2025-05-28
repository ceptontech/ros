Header header

# Timestamp of the packet
uint64 timestamp

# The sensor handle that triggered this update
uint64 handle

uint8[] voltage_out_range
uint8 temperature_out_range_komodo
uint8 temperature_out_range_iox
uint8 temperature_out_range_iguana
uint8 temperature_out_range_gecko
uint8 iguana_iox_comm_crc_error
uint8 iox_gecko_comm_crc_error
uint8 gecko_komodo_comm_crc_error
uint8 iguana_boot_failure
uint8 phy_sync_failure
uint8 eth_link_down
uint8 ptp_expiration
uint8 thermal_shut_down
uint8 lasers_partial_block
uint8 lasers_full_block
uint8 mmt_interlock
uint8 iguana_failure
uint8 fpga_runtime_fail
uint8 mmt_timeout
uint8 gecko_boot_failure
uint8[] rsv