/*
 * zlink_command.h
 *
 *  Created on: 2019年5月7日
 *      Author: damon
 */

#ifndef _ZLINK_HOST_COMMAND_H_
#define _ZLINK_HOST_COMMAND_H_

#include "zlink_common.h"

#include "zlink_command.h"


typedef struct _zlink_command_arg {
    uint32_t arg_length;
    uint32_t m_arguments[zLinkMaxCommandArguments];
}zlink_command_arg_t;

typedef struct _get_property_arg {
    uint32_t property_tag;
    uint32_t memory_id;
} get_property_arg_t;

typedef struct _set_property_arg {
    uint32_t property_tag;
    uint32_t property_value;
} set_property_arg_t;

typedef struct _flash_erase_all_arg {
    uint32_t memory_id;
} flash_erase_all_arg_t;

typedef struct _flash_erase_region_arg {
    uint32_t start_address;
    uint32_t byte_count;
    uint32_t memory_id;
} flash_erase_region_arg_t;

typedef struct _read_memory_arg {
    uint32_t start_address;
    uint32_t byte_count;
    uint32_t memory_id;
} read_memory_arg_t;

typedef struct _write_memory_arg {
    uint32_t start_address;
    uint32_t byte_count;
    uint32_t memory_id;
    uint8_t *pdata;
} write_memory_arg_t;

typedef struct _fill_memory_arg {
    uint32_t start_address;
    uint32_t byte_count;
    uint32_t fill_pattern;
} fill_memory_arg_t;

typedef struct _execute_arg {
    uint32_t jump_address;
    uint32_t argument_word;
    uint32_t sp_pointer;
} execute_arg_t;

typedef struct _call_arg {
    uint32_t call_address;
    uint32_t argument_word;
} call_arg_t;

typedef struct _ota_program_once_arg {
    uint32_t index_ota;
    uint32_t byte_count;  // 4 or 8
    uint32_t data0;
    uint32_t data1;   // if (byte_count == 4) data1 = NULL;
} ota_program_once_arg_t;

typedef struct _ota_read_once_arg {
    uint32_t index_ota;
    uint32_t byte_count;  //must be 4 for eFuseReadOnce
} ota_read_once_arg_t;

typedef struct _configure_memory_arg {
    uint32_t memory_id;       //Memory ID
    uint32_t config_address;  //Configuration block address
} configure_memory_arg_t;

typedef struct _receive_sb_file_arg {
    uint32_t byte_count;      //Byte count
} receive_sb_file_arg_t;



extern zlinkh_command_t ping_cmd;
/*
    call_arg_t call_arg = {
      .call_address = 0x0215,
      .argument_word = 0x0,
    };

    call_cmd.pfun_init(&call_arg);
    call_cmd.pfun_sendTo(&g_serial_packetizer);
 */
extern zlinkh_command_t call_cmd;
extern zlinkh_command_t execute_cmd;
extern zlinkh_command_t reset_cmd;
extern zlinkh_command_t get_property_cmd;
extern zlinkh_command_t read_memory_cmd;
extern zlinkh_command_t flash_erase_region_cmd;
extern zlinkh_command_t write_memory_cmd;
extern zlinkh_command_t get_lockstat_cmd;

#endif /* _ZLINK_HOST_COMMAND_H_ */
