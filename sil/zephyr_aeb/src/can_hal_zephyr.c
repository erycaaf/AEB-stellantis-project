/**
 * @file  can_hal_zephyr.c
 * @brief CAN HAL — TCP virtual CAN bus for Docker SIL.
 *
 * Uses POSIX sockets directly (native_sim is a Linux process).
 * RX runs in a pthread to avoid interfering with the main loop.
 *
 * Wire format: 16 bytes [4B ID | 1B DLC | 3B pad | 8B data]
 */

#include "can_hal.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>

LOG_MODULE_REGISTER(can_hal, LOG_LEVEL_INF);

#define CAN_TCP_DEFAULT_HOST "127.0.0.1"
#define CAN_TCP_DEFAULT_PORT 29536
#define CAN_TCP_FRAME_SIZE   16

struct can_tcp_frame {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  pad[3];
    uint8_t  data[8];
};

static int sock_fd = -1;

/* RX ring buffer (lock-free single producer single consumer) */
#define RX_QUEUE_SIZE 128

struct rx_item {
    uint32_t id;
    uint8_t  data[8];
    uint8_t  dlc;
};

static struct rx_item rx_queue[RX_QUEUE_SIZE];
static volatile int rx_head = 0;
static volatile int rx_tail = 0;

/* RX pthread (not Zephyr thread — avoids scheduler conflicts) */
static pthread_t rx_pthread;

static void *rx_thread_fn(void *arg)
{
    (void)arg;
    LOG_INF("CAN RX pthread started");

    while (1) {
        uint8_t buf[CAN_TCP_FRAME_SIZE];
        int total = 0;

        while (total < CAN_TCP_FRAME_SIZE) {
            int n = recv(sock_fd, buf + total, CAN_TCP_FRAME_SIZE - total, 0);
            if (n <= 0) {
                usleep(100000);
                total = 0;
                continue;
            }
            total += n;
        }

        struct can_tcp_frame *frame = (struct can_tcp_frame *)buf;
        int next = (rx_head + 1) % RX_QUEUE_SIZE;
        if (next != rx_tail) {
            rx_queue[rx_head].id  = frame->id;
            rx_queue[rx_head].dlc = frame->dlc;
            uint8_t len = (frame->dlc <= 8U) ? frame->dlc : 8U;
            memcpy(rx_queue[rx_head].data, frame->data, len);
            rx_head = next;
        }
    }
    return NULL;
}

/* ── Public API ────────────────────────────────────────────────────────── */

int32_t can_hal_init(uint32_t baud_rate)
{
    (void)baud_rate;

    const char *host = getenv("CAN_TCP_HOST");
    const char *port_str = getenv("CAN_TCP_PORT");
    if (host == NULL) { host = CAN_TCP_DEFAULT_HOST; }
    int port = (port_str != NULL) ? atoi(port_str) : CAN_TCP_DEFAULT_PORT;

    LOG_INF("Connecting to CAN TCP bus at %s:%d", host, port);

    /* Note: Zephyr's POSIX subset on native_sim does not expose
     * getaddrinfo()/<netdb.h>, so the host MUST be a dotted IPv4 literal.
     * The Docker entrypoint resolves the service name with `getent hosts`
     * before launching the ECU and exports CAN_TCP_HOST as an IP. */

    for (int attempt = 0; attempt < 30; attempt++) {
        sock_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (sock_fd < 0) {
            LOG_ERR("socket() failed: %d", errno);
            usleep(1000000);
            continue;
        }

        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons((uint16_t)port);

        if (inet_pton(AF_INET, host, &addr.sin_addr) != 1) {
            LOG_WRN("Invalid IP: %s (attempt %d)", host, attempt + 1);
            close(sock_fd);
            sock_fd = -1;
            usleep(1000000);
            continue;
        }

        int ret = connect(sock_fd, (struct sockaddr *)&addr, sizeof(addr));
        if (ret == 0) {
            LOG_INF("CAN TCP connected (fd=%d)", sock_fd);
            pthread_create(&rx_pthread, NULL, rx_thread_fn, NULL);
            return 0;
        }

        LOG_WRN("Connect attempt %d failed (errno=%d)", attempt + 1, errno);
        close(sock_fd);
        sock_fd = -1;
        usleep(1000000);
    }

    LOG_ERR("CAN TCP connection failed after 30 attempts");
    return -1;
}

int32_t can_hal_add_rx_filter(uint32_t msg_id)
{
    (void)msg_id;
    return 0;
}

int32_t can_hal_send(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    if (sock_fd < 0 || data == NULL) {
        return -1;
    }

    struct can_tcp_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.id  = id;
    frame.dlc = dlc;
    uint8_t len = (dlc <= 8U) ? dlc : 8U;
    memcpy(frame.data, data, len);

    int n = send(sock_fd, &frame, sizeof(frame), MSG_NOSIGNAL);
    if (n != (int)sizeof(frame)) {
        return -1;
    }

    return 0;
}

/* ── Drain RX queue ────────────────────────────────────────────────────── */

extern void can_rx_process(void *state, uint32_t id,
                           const uint8_t *data, uint8_t dlc);

void can_hal_drain_rx(void *can_state)
{
    while (rx_tail != rx_head) {
        struct rx_item *item = &rx_queue[rx_tail];
        can_rx_process(can_state, item->id, item->data, item->dlc);
        rx_tail = (rx_tail + 1) % RX_QUEUE_SIZE;
    }
}
