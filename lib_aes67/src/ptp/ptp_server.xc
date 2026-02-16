// Copyright (c) 2011-2017, XMOS Ltd, All rights reserved
// Portions Copyright (c) 2025, PADL Software Pty Ltd, All rights reserved
#include <xs1.h>
#include <xassert.h>
#include "ptp.h"
#include "ptp_internal.h"
#include "ptp_config.h"
#include "ptp_pdu.h"
#include "ethernet.h"
#include "debug_print.h"

#define PTP_PERIODIC_TIME (10000) // 0.1 milliseconds

#pragma select handler
void receive_ptp_cmd(chanend c, uint32_t &cmd) { cmd = inuchar(c); }

void ptp_server_init(client interface ethernet_cfg_if i_eth_cfg,
                     client interface ethernet_rx_if ?i_eth_rx,
                     client interface xtcp_if ?i_xtcp,
                     enum ptp_server_type server_type,
                     timer ptp_timer,
                     int &ptp_timeout) {
    ptp_timer :> ptp_timeout;

    ptp_init(i_eth_cfg, i_eth_rx, i_xtcp, server_type);
}

void ptp_l2_recv_and_process_packet(client interface ethernet_rx_if i_eth_rx,
                                    client interface ethernet_tx_if i_eth_tx) {
    uint8_t buf[MAX_PTP_MESG_LENGTH];

    ethernet_packet_info_t packet_info;
    i_eth_rx.get_packet(packet_info, buf, MAX_PTP_MESG_LENGTH);

    switch (packet_info.type) {
    case ETH_IF_STATUS:
        if (buf[0] == ETHERNET_LINK_UP) {
            ptp_reset(packet_info.src_ifnum);
        }
        break;
    case ETH_DATA:
        ptp_recv(i_eth_tx, null, buf, packet_info.timestamp,
                 packet_info.src_ifnum, packet_info.len);
        break;
    default:
        break;
    }
}

static void ptp_l3_recv_and_process_packet(client interface xtcp_if i_xtcp,
                                           xtcp_event_type_t event,
                                           int32_t id) {
    uint8_t buf[MAX_PTP_MESG_LENGTH];
    int32_t nrecv;
    xtcp_ipaddr_t ipaddr = {0};
    uint16_t port_number = 0;
    uint32_t timestamp;

    if (id == g_ptp_l3_event_rx || id == g_ptp_l3_general_rx) {
        nrecv = i_xtcp.recvfrom_timed(id, buf, MAX_PTP_MESG_LENGTH, ipaddr,
                                      port_number, timestamp);
        if (nrecv <= 0)
            return;

        ptp_recv(null, i_xtcp, buf, timestamp, 0, nrecv);
    }
}

void ptp_l3_handle_event(client interface xtcp_if i_xtcp,
                         xtcp_event_type_t event,
                         int32_t id) {
    switch (event) {
    case XTCP_RECV_FROM_DATA:
        ptp_l3_recv_and_process_packet(i_xtcp, event, id);
        break;
    case XTCP_IFUP:
        ptp_reset(0);
        break;
    default:
        break;
    }
}

static void ptp_give_requested_time_info(chanend c, timer ptp_timer) {
    master {
      c <: ptp_reference_local_ts;
      c <: ptp_reference_ptp_ts;
      c <: g_ptp_adjust;
      c <: g_inv_ptp_adjust;
    }
}

void ptp_get_local_time_info_mod64(ptp_time_info_mod64 &info) {
    uint32_t hi, lo;

    ptp_get_reference_ptp_ts_mod_64(hi, lo);
    info.local_ts = ptp_reference_local_ts;
    info.ptp_ts_hi = hi;
    info.ptp_ts_lo = lo;
    info.ptp_adjust = g_ptp_adjust;
    info.inv_ptp_adjust = g_inv_ptp_adjust;
}

#pragma select handler
void ptp_process_client_request(chanend c, timer ptp_timer) {
    uint8_t cmd;

    cmd = inuchar(c);
    (void)inuchar(c);
    (void)inuchar(c);
    (void)inct(c);
    switch (cmd) {
    case PTP_GET_TIME_INFO:
      ptp_give_requested_time_info(c, ptp_timer);
      break;
    case PTP_GET_TIME_INFO_MOD64: {
      uint32_t hi, lo;
      ptp_get_reference_ptp_ts_mod_64(hi, lo);
      master {
        c :> int;
        c <: ptp_reference_local_ts;
        c <: hi;
        c <: lo;
        c <: g_ptp_adjust;
        c <: g_inv_ptp_adjust;
      }
      break;
    }
    case PTP_GET_GRANDMASTER: {
      char grandmaster[8];
      ptp_current_grandmaster(grandmaster);
      master {
        for (int i = 0; i < 8; i++) {
            c <: grandmaster[i];
        }
      }
      break;
    }
    case PTP_GET_STATE: {
      ptp_port_role_t ptp_state = ptp_current_state();
      master {
          c <: ptp_state;
      }
      break;
    }
    case PTP_GET_PDELAY: {
      master {
          c <: 0;
      }
      break;
    }
    case PTP_GET_AS_PATH: {
      master {
          uint16_t count = 0, port_num;
          n64_t pathSequence[PTP_MAXIMUM_PATH_TRACE_TLV];

          c :> port_num;
          ptp_get_path_sequence(port_num, &count, pathSequence);
          c <: count;
          for (uint8_t i = 0; i < count; i++)
              c <: pathSequence[i];
      }
      break;
    }
    case PTP_GET_PORT_INFO: {
      master {
          uint16_t port_num;

          c :> port_num;
          if (port_num < PTP_NUM_PORTS) {
              c <: ptp_port_info[port_num];
          } else {
              ptp_port_info_t port_info = {0};
              c <: port_info;
          }
      }
      break;
    }
    }
}
