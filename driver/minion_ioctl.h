/**
 * minion_ioctl.h
 *
 * Copyright (C) 2019 Oxford Nanopore Technologies Ltd.
 *
 * Author: <info@nanoporetech.com>
 *
 * These are the IOCTL definitions for interacting with the Oxford Nanopore
 * Technologies MinION-1C firmware and hardware.
 *
 */
#ifndef MINION_IOCTL_H
#define MINION_IOCTL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <linux/ioctl.h>
#include <linux/types.h>

/**
 * @brief register read/write ioctl
 * offset   to-driver   offset of the register from the start of the bar
 * size     to-driver   size of transfer in bytes (1,2,4 or 8)
 * write    to-driver   will write to the register if non-zero
 * bar      to-driver   selects the BAR containing the register
 * value    to/from-driver data to write to the register or the data read from
 *                      the register
 */
struct minion_register_s {
    __u32 offset;
    __u16 size;
    __u8 write;
    __u8 bar;
    __u64 value;
} __attribute__(( packed ));
#define MINION_REGISTER_SIZE 16

#define MINION_IOCTL_REG_ACCESS _IOWR('b', 64, struct minion_register_s)


#define MINION_IOCTL_SHIFT_REG_BUFFER_SIZE 283
/**
 * @brief read/write data to the ASICs shift-register for sending commands and
 * reading the current ASIC configuration and OTP bits, etc.
 *
 * to_device to-driver  Pointer to a 283-byte buffer containing data to write
 *                      to the ASIC shift-register. If null existing hardware
 *                      buffer contents will be re-sent.
 * from_device to-driver Pointer to a 283-byte buffer for data read from the
 *                      ASIC shift-register. This can be the same buffer as the
 *                      one pointed to by to_device. If null any data read from
 *                      the hardware will be discarded.
 * clock_hz to-driver   Clock speed in Hz for the transfer. Due to hardware
 *                      limitations the actual clock speed may differ from that
 *                      requested
 * start    to-driver   Sets start-bit if non-zero
 * enable   to-driver   Sets enable-bit it non-zero
 * command_id to/from-driver
 *                      Appears in the DMA meta-data, helps associate commands
 *                      with their effects. This will be written to hardware
 *                      if to_device is set and will be updated with the value
 *                      from the register if from_device is set.
 * waveform_frame_count to/from-driver
 *                      How many frames each sample in the waveform will be
 *                      applied for. A value of 0 dictates that the waveform
 *                      will not be enabled
 * waveform_table_length to/from-driver
 *                      How many frames are in the waveform table.
 *                      Note: this does't specify how big the storage for the
 *                      waveform is. If specified, it should always be
 *                      MINION_WAVEFORM_SIZE 16-bit entries.
 * waveform_table       to-driver
 *                      A pointer to the waveform storage, if null or if
 *                      to_device is null then waveforms will not be applied.
 *                      This will be updated with the current waveform by the
 *                      time the call returns if not null and from_device is
 *                      not null.
 */
struct minion_shift_reg_s {
    __u64   to_device;
    __u64   from_device;
    __u32   clock_hz;
    __u8    start;
    __u8    enable;
    __u8    command_id;
    __u8    waveform_frame_count;
    __u16   waveform_table_length;
    __u16   padding2; // 2 so as not confused with padding that's been removed
    __u64   waveform_table;
} __attribute__(( packed ));
#define MINION_SHIFT_REG_SIZE 36

#define MINION_WAVEFORM_SIZE 512

#define MINION_IOCTL_SHIFT_REG _IOWR('b', 65, struct minion_shift_reg_s)


#define MINION_IOCTL_HS_RECEIVER_REG_SIZE 13
/**
 * @brief Optionally write data, then read the contents of the HS Receiver core
 * registers to/from-driver If write is set then the writable registers will be
 *                      written, then all registers will be read and their
 *                      contents copied into these registers.
 * write    to-driver   If non-zero then contents of the registers array will
 *                      be written to writable registers. Which registers are
 *                      writable is programmed into the driver.
 */
struct  minion_hs_receiver_s {
   __u16    registers[MINION_IOCTL_HS_RECEIVER_REG_SIZE];
   __u8     write;
   __u8     padding[1]; // pad to multiple of 64 bytes (MUST BE ZERO)
} __attribute__(( packed ));
#define MINION_HS_RECEIVER_SIZE 28

#define MINION_IOCTL_HS_RECIEVER _IOWR('b', 66, struct minion_hs_receiver_s)

/**
 * @brief Transfer data to/from the EEPROM
 * data     to-driver   Pointer to a buffer containing the data to be written to
 *                      the EEPROM or to a buffer to be filled with data read
 *                      from the EEPROM
 * start    to_driver   Where in the EEPROM to start the read/write
 * length   to_driver   Number of bytes to read/write
 */
struct minion_eeprom_transfer_s {
    __u64           data;
    __u32           start;
    __u32           length;
} __attribute__(( packed ));
#define MINION_EEPROM_TRANSFER_SIZE 16

/**
 * For these defines the return values may indicate
 *  EBUSY   The EEPROM shares electrical connections with the ASIC acquisition
 *          data. If the flow-cell is acquiring data these will return EBUSY
 *  ENODEV  No flow-cell present (the EEPROM is on the flow-cell)
 *  EIO     Probably timed out
 *  ERANGE  Start and/or length are too big for the EEPROM
 */
#define MINION_IOCTL_EEPROM_READ  _IOWR('b', 67, struct minion_eeprom_transfer_s)
#define MINION_IOCTL_EEPROM_WRITE _IOWR('b', 68, struct minion_eeprom_transfer_s)

/**
 * @brief DMA data into memory, optionally with a signal when complete
 *
 * Use to submit transfers to the driver. When these are done, the driver will
 * send the process that submitted the transfer the signal [signal_number]. The
 * user-space process should then send a [MINION_IOCTL_WHATS_COMPLETED] ioctl
 * to get a list of transfers that are done.
 *
 * Signals will cause some system-calls to return early so decide if you want
 * use signals or just poll. with the MINION_IOCTL_WHATS_COMPLETED ioctl.
 *
 * buffer        to-driver Pointer to a buffer that will contain the data read
 * buffer_size   to_driver The size of the abave buffer
 * transfer_id   to_driver Used to identify the transfer when completed
 * signal_number to_driver This signal number will be sent to signal that
 *                         transfers have completed.
 * pid           to_driver A signal will be sent to this process to indicate
 *                         that the transfer has completed. Signals may be
 *                         coalesced by the OS. If this value is 0 no signal
 *                         will be sent
 */
struct minion_data_transfer_s {
    __u64           buffer;
    __u32           buffer_size;
    __u32           transfer_id;
    __u32           signal_number;
    __u32           pid;
} __attribute__(( packed ));
#define MINION_DATA_TRANSFER_SIZE 24

#define MINION_IOCTL_SUBMIT_TRANSFER  _IOWR('b', 69, struct minion_data_transfer_s)

/**
 * @brief information about a completed transfer
 *
 * This is used to for the driver to pass information about completed transfers
 * back to the client/user-space
 *
 * transfer_id   from_driver matches up with transfer_id in minion_data_transfer_s
 * status        from_driver Non-zero indicates an error, zero success
 */
struct minion_transfer_status_s {
    __u32           transfer_id;
    __u32           status;// 0 = OK
} __attribute__(( packed ));
#define MINION_TRANSFER_STATUS_SIZE 8

/**
 * @todo: Add a pid field to we don't steal another processes completed transfers
 * @brief For fetching information about completed transfers.
 *
 * completed_transfers      to_driver
 *           Pointer to a buffer for accepting status information about
 *           completed transfers.
 * completed_transfers_size to_driver
 *           The size in number of elements of the completed_transfers buffer.
 * no_completed_transfers   from_driver
 *           How many entries have been written to the completed_transfers
 *           buffer. If this number is the same as completed_transfers_size
 *           then the driver may have more completed transfers and another
 *           call to this IOCTL should be made to fetch them.
 */
struct minion_completed_transfers_s {
    __u64           completed_transfers;
    __u32           completed_transfers_size;
    __u32           no_completed_transfers;
} __attribute__(( packed ));
#define MINION_COMPLETED_TRANSFERS_SIZE 16

/**
 * After receiving a signal from the driver, the user-space code uses this
 * ioctl to enquire which transfers are done. When this IOCTL returns, the
 * transfers for which status is provided are completely disowned by the driver.
 */
#define MINION_IOCTL_WHATS_COMPLETED  _IOWR('b', 70, struct minion_completed_transfers_s)

/** Cancel all transfers */
#define MINION_IOCTL_CANCEL_TRANSFERS  _IO('b', 71)

/**
 * @brief Firmware version info
 *
 * major, minor and patch are the firmware version
 * timestamp is in seconds since Jan 1 1970 for firmware production date
 */
struct minion_firmware_info_s {
    __u16 major;
    __u8  minor;
    __u8  patch;
    __u32 timestamp; // seconds since epoch
} __attribute__(( packed ));
#define MINION_FIRMWARE_INFO_SIZE 8

#define MINON_IOCTL_FIRMWARE_INFO _IOR('b', 72, struct minion_firmware_info_s)


#ifdef __cplusplus
}
#endif

#endif        //  #ifndef MINION_IOCTL_H
