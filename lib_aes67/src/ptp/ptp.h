// Copyright (c) 2011-2017, XMOS Ltd, All rights reserved
// Copyright (c) 2025, PADL Software Pty Ltd, All rights reserved

#pragma once

#include <xccompat.h>
#include "ethernet.h"
#include "xtcp.h"

/** This type represents a timestamp in the gPTP clock domain with respect to
 *the epoch.
 *
 **/
typedef struct ptp_timestamp {
    uint32_t seconds[2];  /*!< The integer portion of the timestamp in units
                                 of seconds */
    uint32_t nanoseconds; /*!< The fractional portion of the timestamp in
                                 units of nanoseconds. */
} ptp_timestamp;

/**
 *  The ptp_ts field stores the seconds and nanoseconds fields separately
 *  so the nanoseconds field is always in the range 0-999999999
 */
struct ptp_time_info {
    uint32_t local_ts; /*!< A local timestamp based on the 100MHz
                                xCORE reference clock */
    ptp_timestamp ptp_ts;  /*!< A PTP timestamp in the gPTP clock domain
                             that matches the local timestamp */

    int ptp_adjust;     /*!< The adjustment required to convert from
                             local time to PTP time */
    int inv_ptp_adjust; /*!< The adjustment required to convert from
                                  PTP time to local time */
};

/** This type is used to relate local xCORE time with gPTP time.
 *  It can be retrieved from the PTP server using the ptp_get_time_info()
 *  function.
 **/
typedef struct ptp_time_info ptp_time_info;

/**
 *  The time stored in the PTP low and high words is the PTP time in
 *  nanoseconds.
 */
struct ptp_time_info_mod64 {
    uint32_t local_ts;
    uint32_t ptp_ts_hi;
    uint32_t ptp_ts_lo;
    int ptp_adjust;
    int inv_ptp_adjust;
};

/** This structure is used to relate local xCORE time with the least
 *  significant 64 bits of gPTP time. The 64 bits of time is the PTP
 *  time in nanoseconds from the epoch.
 *
 *  It can be retrieved from the PTP server using the ptp_get_time_info_mod64()
 *  function.
 **/
typedef struct ptp_time_info_mod64 ptp_time_info_mod64;

/** The type of a PTP server. Can be passed into the ptp_server() function.
 **/
enum ptp_server_type {
    PTP_GRANDMASTER_CAPABLE, /*!< The port is capable of being both PTP
                                Grandmaster and Slave role */
    PTP_SLAVE_ONLY           /*!< The port is capable of PTP Slave role only */
};

/** Retrieve time information from the PTP server
 *
 *  This function gets an up-to-date structure of type `ptp_time_info` to use
 *  to convert local time to PTP time.
 *
 *  \param ptp_server chanend connected to the ptp_server
 *  \param info       structure to be filled with time information
 *
 **/
void ptp_get_time_info(chanend ptp_server,
                       REFERENCE_PARAM(ptp_time_info, info));

/** Retrieve time information from the PTP server
 *
 *  This function gets an up-to-date structure of type `ptp_time_info_mod64`
 *  to use to convert local time to PTP time (modulo 64 bits).
 *
 *  \param ptp_server chanend connected to the ptp_server
 *  \param info       structure to be filled with time information
 *
 **/
void ptp_get_time_info_mod64(NULLABLE_RESOURCE(chanend, ptp_server),
                             REFERENCE_PARAM(ptp_time_info_mod64, info));

// Asynchronous PTP client functions
// --------------------------------

/** This function requests a `ptp_time_info` structure from the
    PTP server. This is an asynchronous call so needs to be completed
    later with a call to ptp_get_requested_time_info().

    \param ptp_server chanend connecting to the ptp server

 **/
void ptp_request_time_info(chanend ptp_server);

/** This function receives a `ptp_time_info` structure from the
    PTP server. This completes an asynchronous transaction initiated with a call
    to ptp_request_time_info(). The function can be placed in a select case
    which will activate when the PTP server is ready to send.

    \param ptp_server      chanend connecting to the PTP server
    \param info            a reference parameter to be filled with the time
                           information structure
**/
#ifdef __XC__
#pragma select handler
#endif
void ptp_get_requested_time_info(chanend ptp_server,
                                 REFERENCE_PARAM(ptp_time_info, info));

/** This function requests a `ptp_time_info_mod64` structure from the
    PTP server. This is an asynchronous call so needs to be completed
    later with a call to ptp_get_requested_time_info_mod64().

    \param ptp_server chanend connecting to the PTP server

 **/
void ptp_request_time_info_mod64(chanend ptp_server);

/** This function receives a `ptp_time_info_mod64` structure from the
    PTP server. This completes an asynchronous transaction initiated with a call
    to ptp_request_time_info_mod64().
    The function can be placed in a select case
    which will activate when the PTP server is ready to send.

    \param ptp_server      chanend connecting to the PTP server
    \param info            a reference parameter to be filled with the time
                           information structure
**/
#ifdef __XC__
#pragma select handler
#endif
void ptp_get_requested_time_info_mod64(chanend ptp_server,
                                       REFERENCE_PARAM(ptp_time_info_mod64,
                                                       info));

/** Convert a timestamp from the local xCORE timer to PTP time.
 *
 *  This function takes a 32-bit timestamp taken from an xCORE timer and
 *  converts it to PTP time.
 *
 *  \param ptp_ts         the PTP timestamp structure to be filled with the
 *                        converted time
 *  \param local_ts       the local timestamp to be converted
 *  \param info           a time information structure retrieved from the PTP
 *                        server
 **/
void local_timestamp_to_ptp(REFERENCE_PARAM(ptp_timestamp, ptp_ts),
                            uint32_t local_ts,
                            REFERENCE_PARAM(ptp_time_info, info));

/** Convert a timestamp from the local xCORE timer to the least significant
 *  32 bits of PTP time.
 *
 *  This function takes a 32-bit timestamp taken from an xCORE timer and
 *  converts it to the least significant 32 bits of global PTP time.
 *
 *  \param local_ts       the local timestamp to be converted
 *  \param info           a time information structure retrieved from the PTP
 *                        server
 *  \returns              the least significant 32-bits of PTP time in
 *                        nanoseconds
 **/
uint32_t local_timestamp_to_ptp_mod32(uint32_t local_ts,
                                      REFERENCE_PARAM(ptp_time_info_mod64,
                                                      info));

/** Convert a PTP timestamp to a local xCORE timestamp.
 *
 *  This function takes a PTP timestamp and converts it to a local
 *  32-bit timestamp that is related to the xCORE timer.
 *
 *  \param ts             the PTP timestamp to convert
 *  \param info           a time information structure retrieved from the PTP
 *                        server.
 *  \returns              the local timestamp
 **/
uint32_t ptp_timestamp_to_local(REFERENCE_PARAM(ptp_timestamp, ts),
                                REFERENCE_PARAM(ptp_time_info, info));

/** Convert a PTP timestamp to a local xCORE timestamp.
 *
 *  This function takes a PTP timestamp and converts it to a local
 *  32-bit timestamp that is related to the xCORE timer.
 *
 *  \param ts             the least significant 32 bits of a PTP timestamp to
 *convert \param info           a time information structure retrieved from the
 *PTP server. \returns              the local timestamp
 **/
uint32_t ptp_mod32_timestamp_to_local(uint32_t ts,
                                      REFERENCE_PARAM(ptp_time_info_mod64,
                                                      info));

/** Calculate an offset to a PTP timestamp.
 *
 *  This function adds and offset to a timestamp.
 *
 *  \param ptp_timestamp the timestamp to be offset; this argument is modified
 *                       by adding the offset
 *  \param offset        the offset to add in nanoseconds
 *
 */
void ptp_timestamp_offset(REFERENCE_PARAM(ptp_timestamp, ts), int offset);
