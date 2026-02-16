// Copyright (c) 2015-2017, XMOS Ltd, All rights reserved

#pragma once

#include <ethernet.h>
#include <xtcp.h>

#include "aes67_internal.h"
#include "ptp_config.h"

#define PTP_ADJUST_PREC 30

enum ptp_cmd_t {
    PTP_GET_TIME_INFO,
    PTP_GET_TIME_INFO_MOD64,
    PTP_GET_GRANDMASTER,
    PTP_GET_STATE,
    PTP_GET_PDELAY,
    PTP_GET_AS_PATH,
    PTP_GET_PORT_INFO
};

typedef enum ptp_port_role_t {
    PTP_MASTER,
    PTP_UNCERTAIN,
    PTP_SLAVE,
    PTP_DISABLED
} ptp_port_role_t;

typedef struct ptp_path_delay_t {
    int valid;
    uint32_t pdelay;
    uint32_t lost_responses;
    uint32_t exchanges;
    uint32_t multiple_resp_count;
    uint32_t last_multiple_resp_seq_id;
    n80_t rcvd_source_identity;
} ptp_path_delay_t;

typedef struct ptp_port_info_t {
    int asCapable;
    ptp_port_role_t role_state;
    ptp_path_delay_t delay_info;
} ptp_port_info_t;

extern ptp_port_info_t ptp_port_info[PTP_NUM_PORTS];

extern uint32_t ptp_reference_local_ts;
extern ptp_timestamp ptp_reference_ptp_ts;
extern int32_t g_ptp_adjust;
extern int32_t g_inv_ptp_adjust;

extern int32_t g_ptp_l3_event_rx;
extern int32_t g_ptp_l3_general_rx;

// Synchronous PTP client functions
// --------------------------------

ptp_port_role_t ptp_get_state(chanend ptp_server);

/** Retrieve time information from the PTP server
 *
 *  This function gets an up-to-date structure of type `ptp_time_info` to use
 *  to convert local time to PTP time.
 *
 *  \param ptp_server chanend connected to the ptp_server
 *  \param info       structure to be filled with time information
 *
 **/
void ptp_get_propagation_delay(chanend ptp_server, uint32_t *pdelay);

void ptp_get_as_path(chanend ptp_server,
                     uint16_t port_num,
                     n64_t pathSequence[PTP_MAXIMUM_PATH_TRACE_TLV],
                     uint16_t *count);

void ptp_get_current_grandmaster(chanend ptp_server, uint8_t grandmaster[8]);

void ptp_get_port_info(chanend ptp_server,
                       uint16_t port_num,
                       ptp_port_info_t *port_info);

/** Initialize the inline ptp server.
 *
 *  \param i_eth_rx       interface connected to the ethernet server (receive)
 *  \param i_eth_tx       interface connected to the ethernet server (transmit)
 *  \param server_type The type of the server (``PTP_GRANDMASTER_CAPABLE``
 *                     or ``PTP_SLAVE_ONLY``)
 *
 *  This function initializes the PTP server when you want to use it inline
 *  combined with other event handling functions (i.e. share the resource in
 *  the ptp thread).
 *  It needs to be called in conjunction with do_ptp_server().
 *  Here is an example usage::
 *
 *     ptp_server_init(c_rx, c_tx, PTP_GRANDMASTER_CAPABLE);
 *     while (1) {
 *         select {
 *             do_ptp_server(c_tx, c_tx, ptp_client, num_clients);
 *             // Add your own cases here
 *         }
 *
 *     }
 *
 *  \sa do_ptp_server
 **/
void ptp_server_init(CLIENT_INTERFACE(ethernet_cfg_if, i_eth_cfg),
                     CLIENT_INTERFACE(ethernet_rx_if ?, i_eth_rx),
                     CLIENT_INTERFACE(xtcp_if ?, i_xtcp),
                     enum ptp_server_type server_type,
                     timer ptp_timer,
                     REFERENCE_PARAM(int, ptp_timeout));

#ifdef __XC__
void ptp_l2_recv_and_process_packet(client interface ethernet_rx_if i_eth_rx,
                                    client interface ethernet_tx_if i_eth_tx);

void ptp_l3_handle_event(client interface xtcp_if i_xtcp,
                         xtcp_event_type_t event,
                         int32_t id);

/* These functions are the workhorse functions for the actual protocol.
   They are implemented in ptp.c  */
void ptp_init(client interface ethernet_cfg_if,
              client interface ethernet_rx_if?,
              client interface xtcp_if?,
              enum ptp_server_type stype);
void ptp_reset(int port_num);
void ptp_recv(
    client interface ethernet_tx_if ?i_eth_rx,
    client interface xtcp_if ?i_xtcp,
    uint8_t buf[len],
    uint32_t ts,
    uint32_t src_port,
    uint32_t len);
void ptp_periodic(
    client interface ethernet_tx_if ?i_tx_if,
    client interface xtcp_if ?i_xtcp,
    uint32_t);
void ptp_get_reference_ptp_ts_mod_64(uint32_t &hi, uint32_t &lo);
void ptp_current_grandmaster(uint8_t grandmaster[8]);
void ptp_get_path_sequence(uint16_t port_num,
                           uint16_t *count,
                           n64_t pathSequence[PTP_MAXIMUM_PATH_TRACE_TLV]);

ptp_port_role_t ptp_current_state(void);

#endif
#ifdef __XC__
#pragma select handler
#endif
void ptp_process_client_request(chanend c, timer ptp_timer);
#define PTP_PERIODIC_TIME (10000) // 0.tfp1 milliseconds

#define do_ptp_server(i_eth_rx, i_eth_tx, i_xtcp, ptp_timer, ptp_timeout)      \
    case !isnull(i_eth_rx) => i_eth_rx.packet_ready():                         \
        ptp_l2_recv_and_process_packet(i_eth_rx, i_eth_tx);                    \
        break;                                                                 \
    case !isnull(i_xtcp) => i_xtcp.event_ready():                              \
        xtcp_event_type_t event;                                               \
        int32_t id;                                                            \
                                                                               \
        event = i_xtcp.get_event(id);                                          \
        ptp_l3_handle_event(i_xtcp, event, id);                                \
        break;                                                                 \
    case ptp_timer when timerafter(ptp_timeout) :> void:                       \
        ptp_periodic(i_eth_tx, i_xtcp, ptp_timeout);                           \
        ptp_timeout += PTP_PERIODIC_TIME;                                      \
        break

void ptp_get_local_time_info_mod64(REFERENCE_PARAM(ptp_time_info_mod64, info));

void ptp_output_test_clock(chanend ptp_link, port test_clock_port, int period);

// Parse PTP GM ID from string format (AA-BB-CC-DD-EE-FF-GG-HH) to network byte
// order n64_t
aes67_status_t parse_ptp_gmid(const char *unsafe gmid_str, REFERENCE_PARAM(n64_t, result));

extern int sync_lock;
