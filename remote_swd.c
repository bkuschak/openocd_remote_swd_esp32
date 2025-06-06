// Implement the remote (server) side of the OpenOCD remote_swd driver.
// Intended as a much faster alternative to the remote_bitbang driver.
// This file implements the protocol processing that is platform-independent.
//
// Brian Kuschak <bkuschak@gmail.com>
//
// For reference, this page has a good description of the physical SWD protocol:
// https://qcentlabs.com/posts/swd_banger/

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include "remote_swd.h"

// ----------------------------------------------------------------------------
// Some code copied/adapted from OpenOCD src/jtag/swd.h and helpers.h:

/* Bits in SWD command packets, written from host to target
 * first bit on the wire is START
 */
#define SWD_CMD_START   (1 << 0)    /* always set */
#define SWD_CMD_APNDP   (1 << 1)    /* set only for AP access */
#define SWD_CMD_RNW     (1 << 2)    /* set only for read access */
#define SWD_CMD_A32     (3 << 3)    /* bits A[3:2] of register addr */
#define SWD_CMD_PARITY  (1 << 5)    /* parity of APnDP|RnW|A32 */
#define SWD_CMD_STOP    (1 << 6)    /* always clear for synch SWD */
#define SWD_CMD_PARK    (1 << 7)    /* driven high by host */
/* followed by TRN, 3-bits of ACK, TRN */

/**
 * SWD Line reset.
 *
 * SWD Line reset is at least 50 SWCLK cycles with SWDIO driven high,
 * followed by at least two idle (low) cycle.
 * Bits are stored (and transmitted) LSB-first.
 */
static const uint8_t swd_seq_line_reset[] = {
    /* At least 50 SWCLK cycles with SWDIO high */
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    /* At least 2 idle (low) cycles */
    0x00,
};
static const unsigned int swd_seq_line_reset_len = 64;

/**
 * Dormant-to-SWD sequence.
 *
 * This is at least 8 TCK/SWCLK cycles with TMS/SWDIO high to abort any ongoing
 * selection alert sequence, followed by a specific 128-bit selection alert
 * sequence, followed by 4 TCK/SWCLK cycles with TMS/SWDIO low, followed by
 * a specific protocol-dependent activation code. For SWD the activation code
 * is an 8-bit sequence. The sequence ends with a line reset.
 * Bits are stored (and transmitted) LSB-first.
 */
static const uint8_t swd_seq_dormant_to_swd[] = {
    /* At least 8 SWCLK cycles with SWDIO high */
    0xff,
    /* Selection alert sequence */
    0x92, 0xf3, 0x09, 0x62, 0x95, 0x2d, 0x85, 0x86,
    0xe9, 0xaf, 0xdd, 0xe3, 0xa2, 0x0e, 0xbc, 0x19,
    /*
     * 4 SWCLK cycles with SWDIO low ...
     * + SWD activation code 0x1a ...
     * + at least 8 SWCLK cycles with SWDIO high
     */
    0xa0, /* ((0x00)      & GENMASK(3, 0)) | ((0x1a << 4) & GENMASK(7, 4)) */
    0xf1, /* ((0x1a >> 4) & GENMASK(3, 0)) | ((0xff << 4) & GENMASK(7, 4)) */
    0xff,
    /* At least 50 SWCLK cycles with SWDIO high */
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    /* At least 2 idle (low) cycles */
    0x00,
};
static const unsigned int swd_seq_dormant_to_swd_len = 224;

/**
 * SWD-to-dormant sequence.
 *
 * This is at least 50 SWCLK cycles with SWDIO high to put the interface
 * in reset state, followed by a specific 16-bit sequence.
 * Bits are stored (and transmitted) LSB-first.
 */
static const uint8_t swd_seq_swd_to_dormant[] = {
    /* At least 50 SWCLK cycles with SWDIO high */
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
     /* Switching sequence from SWD to dormant */
    0xbc, 0xe3,
};
static const unsigned int swd_seq_swd_to_dormant_len = 72;

/**
 * JTAG-to-SWD sequence.
 *
 * The JTAG-to-SWD sequence is at least 50 TCK/SWCLK cycles with TMS/SWDIO
 * high, putting either interface logic into reset state, followed by a
 * specific 16-bit sequence and finally a line reset in case the SWJ-DP was
 * already in SWD mode.
 * Bits are stored (and transmitted) LSB-first.
 */
static const uint8_t swd_seq_jtag_to_swd[] = {
	/* At least 50 TCK/SWCLK cycles with TMS/SWDIO high */
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	/* Switching sequence from JTAG to SWD */
	0x9e, 0xe7,
	/* At least 50 TCK/SWCLK cycles with TMS/SWDIO high */
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	/* At least 2 idle (low) cycles */
	0x00,
};
static const unsigned int swd_seq_jtag_to_swd_len = 136;

/**
 * SWD-to-JTAG sequence.
 *
 * The SWD-to-JTAG sequence is at least 50 TCK/SWCLK cycles with TMS/SWDIO
 * high, putting either interface logic into reset state, followed by a
 * specific 16-bit sequence and finally at least 5 TCK/SWCLK cycles with
 * TMS/SWDIO high to put the JTAG TAP in Test-Logic-Reset state.
 * Bits are stored (and transmitted) LSB-first.
 */
static const uint8_t swd_seq_swd_to_jtag[] = {
	/* At least 50 TCK/SWCLK cycles with TMS/SWDIO high */
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	/* Switching sequence from SWD to JTAG */
	0x3c, 0xe7,
	/* At least 5 TCK/SWCLK cycles with TMS/SWDIO high */
	0xff,
};
static const unsigned int swd_seq_swd_to_jtag_len = 80;

// ----------------------------------------------------------------------------

// SWD pin I/O function pointers that are set by the user.
static int (*swdio_swclk_init_)(void);
static int (*swdio_input_)(void);
static int (*swdio_output_)(void);
static int (*swdio_write_)(bool bit);
static bool (*swdio_read_)(void);
static int (*swclk_pulse_)(void);
static int (*set_srst_)(bool val, bool open_drain);

// Socket I/O function pointers that are set by the user.
static int (*socket_available_)(void);
static int (*socket_read_)(void* buf, int len);
static int (*socket_write_)(void* buf, int len);

static uint32_t serial_number_;
static uint16_t hw_version_;
static const uint16_t sw_version_ = REMOTE_SWD_SW_VERSION;

// Queue up the responses, so we can send them all at once in a single packet.
static struct queued_command responses[128];
static int response_idx = 0;

// ----------------------------------------------------------------------------
// Socket code (should work on Linux, Mac, ESP32)

static int server_sockfd;
static int client_sockfd;

static int socket_available(void)
{
    if(client_sockfd == 0)
        return 0;

    int nbytes;
    int ret = ioctl(client_sockfd, FIONREAD, &nbytes);
    if(ret < 0)
        return 0;
    if(ret > 0)
        fprintf(stderr, "Available %d\n", ret);
    return nbytes;
}

// Returns 1 if disconnected, or 0 if still connected, or <0 on error.
// https://stackoverflow.com/questions/5640144/c-how-to-use-select-to-see-if-a-socket-has-closed/5640173
static int socket_disconnected(void)
{
    char x;
    int r;

    // Non blocking peek to see if any data is available.
    // If client has disconnected, then recv() will return 0.
    while(true) {
        r = recv(client_sockfd, &x, 1, MSG_DONTWAIT|MSG_PEEK);
        if (r < 0) {
            switch (errno) {
                case EINTR:     continue;
                case EAGAIN:    break; /* empty rx queue */
                case ETIMEDOUT: break; /* recv timeout */
                case ENOTCONN:  break; /* not connected yet */
                default:        return -errno;
            }
        }
        break;
    }
    return r == 0;
}

// Start the server.
static int start_server(int port)
{
    int ret;
    int connfd;
    struct sockaddr_in server_addr;

    server_sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if(server_sockfd < 0) {
        fprintf(stderr, "Failed to open server socket.\n");
        return -1;
    }

    int optval = 1;
    setsockopt(server_sockfd, SOL_SOCKET, SO_REUSEPORT, &optval,
            sizeof(optval));

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(port);

    ret = bind(server_sockfd, (void*)&server_addr, sizeof(server_addr));
    if(ret != 0) {
        fprintf(stderr, "Failed to bind server socket.\n");
        return -1;
    }

    ret = fcntl(server_sockfd, F_SETFL, O_NONBLOCK);
    if(ret < 0) {
        fprintf(stderr, "Failed to set nonblocking server socket.\n");
        return -1;
    }

    ret = listen(server_sockfd, 1);
    if(ret < 0) {
        fprintf(stderr, "Failed to listen to server socket.\n");
        return -1;
    }

    fprintf(stderr, "remote_swd server listening on port %d.\n", port);
    return 0;
}

// Handle incoming connections on server socket. Non-blocking.
static int handle_server(void)
{
    // Handle a new client connecting.
    struct sockaddr_in client_addr;
    unsigned len = sizeof(client_addr);
    int ret = accept(server_sockfd, (void*)&client_addr, &len);
    if(ret < 0) {
        if(errno == EWOULDBLOCK || errno == EAGAIN)
            return 0; // No clients connecting.
        else
            return ret;
    }

    // We only support a single connected client. If a client is already
    // connected, drop new connections.
    if(client_sockfd != 0) {
        fprintf(stderr, "Dropping new connection.\n");
        close(ret);
    }
    else {
        fprintf(stderr, "Client connected.\n");
        client_sockfd = ret;

        // Use TCP keepalives to detect dead clients.
        int val = 1;
        setsockopt(client_sockfd, SOL_SOCKET, SO_KEEPALIVE, &val, sizeof(val));
#if defined(__linux__) || defined(ESP32) || defined(ESP8266) || defined(ARDUINO_ARCH_ESP32)
        // Seconds between probes (Linux and ESP32)
        val = 1;
        setsockopt(client_sockfd, IPPROTO_TCP, TCP_KEEPIDLE, &val,
                    sizeof(val));
        setsockopt(client_sockfd, IPPROTO_TCP, TCP_KEEPINTVL, &val,
                    sizeof(val));
        // Number of probes to send before closing the connection.
        val = 5;
        setsockopt(client_sockfd, IPPROTO_TCP, TCP_KEEPCNT, &val,
                    sizeof(val));
#elif defined(__APPLE__)
        // Seconds between probes (MacOS). TCP_KEEPALIVE is like TCP_KEEPIDLE.
        val = 5;
        setsockopt(client_sockfd, IPPROTO_TCP, TCP_KEEPALIVE, &val,
                    sizeof(val));
#else
#error "Platform not recognized! Cannot setup TCP keepalive."
#endif
        ret = fcntl(client_sockfd, F_SETFL, O_NONBLOCK);
        if(ret < 0) {
            fprintf(stderr, "Failed to set nonblocking server socket.\n");
            return -1;
        }
    }
    return 0;
}

// Just check if the client has disconnected.
static int handle_client(void)
{
    if(client_sockfd == 0)
        return 0;

    if(socket_disconnected()) {
        fprintf(stderr, "Client disconnected.\n");
        close(client_sockfd);
        client_sockfd = 0;
    }
    return 0;
}

// Read from socket. Return number of bytes, or <0 on error.
static int socket_read(void* data, int len)
{
    if(client_sockfd == 0)
        return -1;      // No client.

    int ret = read(client_sockfd, data, len);
    return ret;
}

// Read from socket. Return number of bytes, or <0 on error.
static int socket_write(void* data, int len)
{
    if(client_sockfd == 0)
        return -1;      // No client.

    return write(client_sockfd, data, len);
}

// ----------------------------------------------------------------------------

int remote_swd_init(uint32_t serial_number, uint16_t hw_version,
        int (*swdio_swclk_init)(void),
        int (*swdio_input)(void),
        int (*swdio_output)(void),
        int (*swdio_write)(bool bit),
        bool (*swdio_read)(void),
        int (*swclk_send_pulse)(void),
        int (*set_srst)(bool val, bool open_drain),
        int (*socket_available)(void),
        int (*socket_read)(void* buf, int len),
        int (*socket_write)(void* buf, int len))
{
    serial_number_ = serial_number;
    hw_version_ = hw_version;
    swdio_swclk_init_ = swdio_swclk_init;
    swdio_input_ = swdio_input;
    swdio_output_ = swdio_output;
    swdio_write_ = swdio_write;
    swdio_read_ = swdio_read;
    swclk_pulse_ = swclk_send_pulse;
    set_srst_ = set_srst;
    socket_available_ = socket_available;
    socket_read_ = socket_read;
    socket_write_ = socket_write;

    memset(responses, 0, sizeof(responses));
    response_idx = 0;

    int ret = swdio_swclk_init_();
    if(ret < 0)
        fprintf(stderr, "Failed initializing SWD pins!\n");
    return ret;
}

void remote_swd_write_sequence(const uint8_t* data, int len)
{
    // Set SWD as output.
    swdio_output_();

    // Send byte.
    for(int i=0; i<len; i++) {
        uint8_t byte = data[i/8];
        bool bit = byte & (1<<(i%8));

        // Drive SWD before rising edge of SWCLK.
        swdio_write_(bit);
        swclk_pulse_();
    }
}

/**
 * Copied from OpenOCD.
 * Calculate the (even) parity of a 32-bit datum.
 * @param x The datum.
 * @return 1 if the number of set bits in x is odd, 0 if it is even.
 */
static inline int parity_u32(uint32_t x)
{
#ifdef __GNUC__
    return __builtin_parityl(x);
#else
    x ^= x >> 16;
    x ^= x >> 8;
    x ^= x >> 4;
    x ^= x >> 2;
    x ^= x >> 1;
    return x & 1;
#endif
}

void remote_swd_write_reg(uint8_t cmd, uint32_t data, uint8_t* ack, int ap_delay_clks)
{
    bool bit;

    // Set SWD as output.
    swdio_output_();

    // Send command.
    for(int i=0; i<8; i++) {
        bool bit = cmd & (1<<i);

        // Drive SWD before rising edge of SWCLK.
        swdio_write_(bit);
        swclk_pulse_();
    }

     if(ack) {
        // Turnaround cycle.
        swdio_input_();
        swclk_pulse_();

        // Receive ACK bits.
        *ack = 0;
        for(int i=0; i<3; i++) {
            bit = swdio_read_();
            swclk_pulse_();
            if(bit)
                *ack |= (1<<i);
        }

        // Turnaround cycle.
        swclk_pulse_();
        swdio_output_();
    }

    // Send data.
    for(int i=0; i<32; i++) {
        bool bit = data & (1<<i);
        swdio_write_(bit);
        swclk_pulse_();
    }

    // Send parity bit.
    swdio_write_(parity_u32(data));
    swclk_pulse_();

    // Send final clocks, if any, with SWD=0.
    for(int i=0; i<ap_delay_clks; i++) {
        swdio_write_(0);
        swclk_pulse_();
    }
}

// Return 0 on success, or <0 on parity error.
int remote_swd_read_reg(uint8_t cmd, uint32_t* data, uint8_t* ack, int ap_delay_clks)
{
    bool bit;

    // Set SWD as output.
    swdio_output_();

    // Send command.
    for(int i=0; i<8; i++) {
        bool bit = cmd & (1<<i);
        swdio_write_(bit);
        swclk_pulse_();
    }

    // Turnaround cycle.
    swdio_input_();
    swclk_pulse_();

    if(ack) {
        *ack = 0;
        // Receive ACK bits.
        for(int i=0; i<3; i++) {
            bit = swdio_read_();
            swclk_pulse_();
            if(bit)
                *ack |= (1<<i);
        }
    }

    // Receive data.
    *data = 0;
    for(int i=0; i<32; i++) {
        bit = swdio_read_();
        swclk_pulse_();
        if(bit)
            *data |= (1<<i);
    }

    // Receive parity bit. We should have odd parity overall.
    bool parity = parity_u32(*data);
    bit = swdio_read_();
    swclk_pulse_();
    parity ^= bit;

    // Turnaround cycle.
    swclk_pulse_();
    swdio_output_();
    swdio_write_(0);

    // Send final clocks, if any, with SWD=0.
    for(int i=0; i<ap_delay_clks; i++)
        swclk_pulse_();

    // We expect odd parity, so the 'parity' result should be 0.
    if(parity == 0)
        return 0;
    else
        return -1;
}

/**
 * Copied from OpenOCD.
 * Construct a "cmd" byte, in LSB bit order, which swd_driver.read_reg()
 * and swd_driver.write_reg() methods will use directly.
 */
static inline uint8_t swd_cmd(bool is_read, bool is_ap, uint8_t regnum)
{
    uint8_t cmd = (is_ap ? SWD_CMD_APNDP : 0)
        | (is_read ? SWD_CMD_RNW : 0)
        | ((regnum & 0xc) << 1);

    /* 8 cmd bits 4:1 may be set */
    if (parity_u32(cmd))
        cmd |= SWD_CMD_PARITY;

    cmd |= SWD_CMD_START;
    cmd |= SWD_CMD_PARK;
    return cmd;
}

// Convenience functions - usage optional.
// Read a DP register.
// Addr must be <= 0xC.
int remote_swd_read_dp_reg(uint8_t addr, uint32_t* data, uint8_t* ack, int ap_delay_clks,
        char* name)
{
    uint8_t cmd = swd_cmd(true, false, addr);
    int ret = remote_swd_read_reg(cmd, data, ack, ap_delay_clks);
    if(ret != 0)
        fprintf(stderr, "RD DP %s (0x%X) failed: ack=%hx, data=%08x\n",
                name, addr, *ack, *data);
    else
        fprintf(stderr, "RD DP %s (0x%X) success: ack=%hx, data=%08x.\n",
                name, addr, *ack, *data);
    return ret;
}

// Write a DP register.
// Addr must be <= 0xC.
void remote_swd_write_dp_reg(uint8_t addr, uint32_t data, uint8_t* ack, int ap_delay_clks,
        char* name)
{
    uint8_t cmd = swd_cmd(false, false, addr);
    remote_swd_write_reg(cmd, data, ack, ap_delay_clks);
    fprintf(stderr, "WR DP %s (0x%X): ack=%hx, data=%08x.\n",
            name, addr, *ack, data);
}

// TODO AP Bank select in DP SELECT register

// Read an AP register.
// Addr must be <= 0xC.
int remote_swd_read_ap_reg(uint8_t addr, uint32_t* data, uint8_t* ack, int ap_delay_clks,
        char* name)
{
    uint8_t cmd = swd_cmd(true, true, addr);
    int ret = remote_swd_read_reg(cmd, data, ack, ap_delay_clks);
    if(ret != 0)
        fprintf(stderr, "RD AP %s (0x%X) failed: ack=%hx, data=%08x\n",
                name, addr, *ack, *data);
    else
        fprintf(stderr, "RD AP %s (0x%X) success: ack=%hx, data=%08x.\n",
                name, addr, *ack, *data);
    return ret;
}

// Write an AP register.
// Addr must be <= 0xC.
void remote_swd_write_ap_reg(uint8_t addr, uint32_t data, uint8_t* ack, int ap_delay_clks,
        char* name)
{
    uint8_t cmd = swd_cmd(false, true, addr);
    remote_swd_write_reg(cmd, data, ack, ap_delay_clks);
    fprintf(stderr, "WR AP %s (0x%X): ack=%hx, data=%08x.\n",
            name, addr, *ack, data);
}

// Read DP CTRL/STAT and check for error bits. If found, clear them.
// TODO - handle other errors.
int remote_swd_check_error(void)
{
  int ap_delay_clks = 10;
  uint8_t ack;
  uint32_t data;
  int ret = remote_swd_read_dp_reg(0x4, &data, &ack, ap_delay_clks, "CTRL/STAT");
  if(ret != 0) {
      fprintf(stderr, "Failed reading CTRL/STAT\n");
      return ret;
  }

  if(data & 0xA2) {
      // Clear errors.
      if(data & 0x80) {
          fprintf(stderr, "Write ERR\n");
      }
      if(data & 0x20) {
          fprintf(stderr, "STICKY ERR\n");
      }
      if(data & 0x02) {
          fprintf(stderr, "STICKY ORUN\n");
      }
      uint32_t val = 0x1E;
      remote_swd_write_dp_reg(0x0, val, &ack, ap_delay_clks, "ABORT");
  }
  return 0;
}

// ----------------------------------------------------------------------------
// These are called in response to packets received from the OpenOCD driver.

static void protocol_version(uint32_t* data)
{
    *data = REMOTE_SWD_PROTOCOL_VERSION;
}

static void version(uint32_t* data)
{
    *data = hw_version_ & 0xFFFF;
    *data |= ((uint32_t)sw_version_ & 0xFFFF) << 16;
}

static void serial_number(uint32_t* data)
{
    *data = serial_number_;
}

static int swd_reset(uint32_t flags, uint32_t data)
{
    bool srst = data & 0x01;
    bool open_drain = flags & FLAGS_SRST_OPEN_DRAIN;
    return set_srst_(srst, open_drain);
}

static int swd_speed(int speed)
{
    // TODO - not currently supported.
    LOG_INFO("Adjusting SWCLK speed is not currently supported.");
    return 0;
}

int remote_swd_switch_seq(enum swd_special_seq seq)
{
    // Issue a special sequence.
    switch (seq) {
        case LINE_RESET:
            LOG_DEBUG_IO("SWD line reset");
            remote_swd_write_sequence(swd_seq_line_reset, swd_seq_line_reset_len);
            break;
        case JTAG_TO_SWD:
            LOG_DEBUG_IO("JTAG-to-SWD");
            remote_swd_write_sequence(swd_seq_jtag_to_swd, swd_seq_jtag_to_swd_len);
            break;
        case SWD_TO_JTAG:
            LOG_DEBUG_IO("SWD-to-JTAG");
            remote_swd_write_sequence(swd_seq_swd_to_jtag, swd_seq_swd_to_jtag_len);
            break;
        case SWD_TO_DORMANT:
            LOG_DEBUG_IO("SWD-to-DORMANT");
            remote_swd_write_sequence(swd_seq_swd_to_dormant, swd_seq_swd_to_dormant_len);
            break;
        case DORMANT_TO_SWD:
            LOG_DEBUG_IO("DORMANT-to-SWD");
            remote_swd_write_sequence(swd_seq_dormant_to_swd, swd_seq_dormant_to_swd_len);
            break;
        default:
            LOG_ERROR("Sequence %d not supported", seq);
            return ERROR_FAIL;
    }
    return ERROR_OK;
}

// Run one command and generate a response.
int remote_swd_run_command(struct queued_command* command, struct queued_command* response)
{
    void* ptr = command;
    int nbytes = sizeof(*command);
    int ret;

    *response = *command;
    response->ack = 0;
    response->data = 0;

    switch(FLAGS_OP(command->flags)) {
        case FLAGS_OP_PROTOCOL:
            LOG_DEBUG("Got PROTOCOL");
            protocol_version(&response->data);
            break;
        case FLAGS_OP_VERSION:
            LOG_DEBUG("Got VERSION");
            version(&response->data);
            break;
        case FLAGS_OP_SERIAL_NUM:
            LOG_DEBUG("Got SERIAL_NUM");
            serial_number(&response->data);
            break;
        case FLAGS_OP_SPEED:
            LOG_DEBUG("Got SPEED");
            swd_speed(command->data);
            break;
        case FLAGS_OP_RESET:
            LOG_DEBUG("Got RESET");
            swd_reset(command->flags, command->data);
            break;
        case FLAGS_OP_SWITCH_SEQ:
            LOG_DEBUG("Got SWITCH_SEQ");
            remote_swd_switch_seq(command->data);
            break;
        case FLAGS_OP_WRITE_REG:
            LOG_DEBUG("Got WRITE_REG: cmd=%08x, data=%08x", command->cmd, command->data);
            remote_swd_write_reg(command->cmd, command->data, &response->ack, command->ap_delay_clks);
            break;
        case FLAGS_OP_READ_REG:
            LOG_DEBUG("Got READ_REG: cmd=%08x", command->cmd);
            ret = remote_swd_read_reg(command->cmd, &response->data, &response->ack, command->ap_delay_clks);
            if(ret != 0)
                LOG_DEBUG("Parity error on register read!");
            break;
        default:
            LOG_DEBUG("Got unknown command.");
            break;
    }
    return 0;
}

// Read one command from the socket if possible.
// Return 0 on success or <0 on error.
static int remote_swd_read_command(struct queued_command* command)
{
    int nbytes = sizeof(*command);
    if(socket_available_() >= nbytes) {
        LOG_DEBUG("Reading command, %d bytes...", nbytes);
        int ret = socket_read_(command, nbytes);
        LOG_DEBUG_IO("socket_read returned %d", ret);
        if(ret < 0)
            return ret;
        else
            return ret == nbytes ? 0 : -1;
    }
    return -1;
}

// Read any incoming commands, execute them, and send responses.
// Return zero on success or nothing to do, and <0 on error.
int remote_swd_process(void)
{
    int ret_cmd;

    // Process all queued commands or until an error or our response queue
    // is full.
    while(true) {
        struct queued_command command;
        struct queued_command* response = &responses[response_idx];

        ret_cmd = remote_swd_read_command(&command);
        if(ret_cmd < 0)
            break;

        // Packet uses network byte order.
        command.data = ntohl(command.data);

        ret_cmd = remote_swd_run_command(&command, response);
        if(ret_cmd < 0)
            break;

        // Packet uses network byte order.
        response->data = htonl(response->data);

        if(++response_idx == ARRAY_SIZE(responses))
            break;
    }

    // Send all responses at once.
    if(response_idx > 0) {
        int nbytes = response_idx * sizeof(responses[0]);
        void* ptr = responses;

        while(nbytes > 0) {
            LOG_DEBUG("Sending responses, %d bytes", nbytes);
            int ret = socket_write_(ptr, nbytes);
            LOG_DEBUG_IO("socket_write returned %d", ret);
            if(ret < 0) {
                // Error. Discard the responses.
                response_idx = 0;
                return ret;
            }
            nbytes -= ret;
            ptr += ret;
        }
        response_idx = 0;
    }
    return ret_cmd;
}

int remote_swd_server_init(uint32_t serial_number, uint16_t hw_version, uint16_t port_number,
        int (*swdio_swclk_init)(void),
        int (*swdio_input)(void),
        int (*swdio_output)(void),
        int (*swdio_write)(bool bit),
        bool (*swdio_read)(void),
        int (*swclk_send_pulse)(void),
        int (*set_srst)(bool val, bool open_drain))
{
    int ret = start_server(port_number);
    if(ret < 0) {
        fprintf(stderr, "Failed starting server. Exiting.\n");
        exit(1);
    }

    // Initialize the SWD library.
    return remote_swd_init(serial_number, hw_version,
            swdio_swclk_init, swdio_input, swdio_output, swdio_write, swdio_read,
            swclk_send_pulse, set_srst, socket_available, socket_read, socket_write);
}

int remote_swd_server_process(void)
{
  handle_server();
  handle_client();

  if(client_sockfd != 0)
      return remote_swd_process();
  else
      return 0;
}
