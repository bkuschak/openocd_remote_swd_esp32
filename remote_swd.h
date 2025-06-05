#ifndef REMOTE_SWD_H
#define REMOTE_SWD_H

#ifdef __cplusplus
extern "C" {
#endif

#define REMOTE_SWD_SW_VERSION         0x0100      // 8 bit major, 8 bit minor
#define REMOTE_SWD_PROTOCOL_VERSION   0x01        // Client side must match this.

#define REMOTE_SWD_TCP_PORT           5253

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a)   (sizeof(a) / sizeof((a)[0]))
#endif

// First 4 bits hold the command code.
#define FLAGS_OP_PROTOCOL       (1<<0)    // Get remote protocol version.
#define FLAGS_OP_VERSION        (2<<0)    // Get remote HW/SW version.
#define FLAGS_OP_SERIAL_NUM     (3<<0)    // Get remote serial number.
#define FLAGS_OP_SPEED          (4<<0)    // Set speed in 'data' field.
#define FLAGS_OP_RESET          (5<<0)    // Set NRST state in 'data' field.
#define FLAGS_OP_SWITCH_SEQ     (6<<0)    // Send sequence in 'data' field.
#define FLAGS_OP_READ_REG       (7<<0)    // SWD read register.
#define FLAGS_OP_WRITE_REG      (8<<0)    // SWD write register.
#define FLAGS_OP(flags)         (flags & 0x0F)

// Remaining bits are reserved for flags.
#define FLAGS_EXPECT_ACK        (1<<4)
#define FLAGS_EXPECT_DATA       (1<<5)
#define FLAGS_SRST_OPEN_DRAIN   (1<<4)    // Used for OP_RESET only.

struct queued_command {
    uint8_t flags;              // Opcode and flags.
    uint8_t ap_delay_clks;      // inAdditional clock cycles sent after data.
    uint8_t cmd;                // SWD command field.
    uint8_t ack;                // SWD ACK field (3 bits).
    uint32_t data;              // SWD data, sent or returned.
};

//#define DEBUG_PRINTING

#ifdef DEBUG_PRINTING
#define LOG_DEBUG(...) \
{ \
    fprintf(stderr, "DEBUG: "); \
    fprintf(stderr, ##__VA_ARGS__); \
    fprintf(stderr, "\n"); \
}

#define LOG_DEBUG_IO(...) \
{ \
    fprintf(stderr, "DEBUG_IO: "); \
    fprintf(stderr, ##__VA_ARGS__); \
    fprintf(stderr, "\n"); \
}

#define LOG_ERROR(...) \
{ \
    fprintf(stderr, "ERROR: "); \
    fprintf(stderr, ##__VA_ARGS__); \
    fprintf(stderr, "\n"); \
}

#define LOG_INFO(...) \
{ \
    fprintf(stderr, "INFO: "); \
    fprintf(stderr, ##__VA_ARGS__); \
    fprintf(stderr, "\n"); \
}
#else
#define LOG_DEBUG(...) { }
#define LOG_DEBUG_IO(...) { }
#define LOG_ERROR(...) { }
#define LOG_INFO(...) { }
#endif

/* general failures
 * error codes < 100
 */
#define ERROR_OK                        (0)
#define ERROR_NO_CONFIG_FILE            (-2)
#define ERROR_BUF_TOO_SMALL             (-3)
/* see "Error:" log entry for meaningful message to the user. The caller should
 * make no assumptions about what went wrong and try to handle the problem.
 */
#define ERROR_FAIL                      (-4)
#define ERROR_WAIT                      (-5)
#define ERROR_TIMEOUT_REACHED           (-6)
#define ERROR_NOT_IMPLEMENTED           (-7)

// TODO - include arm_adi_v5.h instead.
// This must match the definition used by OpenOCD.
enum swd_special_seq {
    LINE_RESET,
    JTAG_TO_SWD,
    JTAG_TO_DORMANT,
    SWD_TO_JTAG,
    SWD_TO_DORMANT,
    DORMANT_TO_SWD,
    DORMANT_TO_JTAG,
};

// Initialize the library and provide callback functions.
// swd_* functions manipulate the SWD pins.  They should return 0 on success or
// <0 on error, except for swdio_read() which should return the pin state.
int remote_swd_server_init(uint32_t serial_num, uint16_t hw_version, uint16_t port_number,
        // Called once to initialize the pins.
        int (*swdio_swclk_init)(void),
        // Switch SWDIO to input mode.
        int (*swdio_input)(void),
        // Switch SWDIO to output mode.
        int (*swdio_output)(void),
        // Set SWDIO pin to 'bit'.
        int (*swdio_write)(bool bit),
        // Read state of SWDIO.
        bool (*swdio_read)(void),
        // Generate an active-high pulse on SWCLK.
        int (*swclk_send_pulse)(void),
        // Set the state of the SRST (NRST) pin.
        int (*set_srst)(bool val, bool open_drain));

// Process any pending packets, execute the commands, and generate responses.
// Returns zero on success or nothing to do, and <0 on error.
int remote_swd_server_process(void);

// ----------------------------------------------------------------------------
// These functions are not typically used, but are available to directly
// perform SWD transfers.

int remote_swd_switch_seq(enum swd_special_seq seq);
void remote_swd_write_sequence(const uint8_t* data, int len);
void remote_swd_write_reg(uint8_t cmd, uint32_t data, uint8_t* ack,
        int ap_delay_clks);
int remote_swd_read_reg(uint8_t cmd, uint32_t* data, uint8_t* ack,
        int ap_delay_clks);

// Perform SWD reads/writes to DP or AP.
int remote_swd_read_dp_reg(uint8_t addr, uint32_t* data, uint8_t* ack,
        int ap_delay_clks, char* name);
void remote_swd_write_dp_reg(uint8_t addr, uint32_t data, uint8_t* ack,
        int ap_delay_clks, char* name);
int remote_swd_read_ap_reg(uint8_t addr, uint32_t* data, uint8_t* ack,
        int ap_delay_clks, char* name);
void remote_swd_write_ap_reg(uint8_t addr, uint32_t data, uint8_t* ack,
        int ap_delay_clks, char* name);
int check_error(void);

#ifdef __cplusplus
}
#endif

#endif  // REMOTE_SWD_H
