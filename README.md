# lib_aes67

This is proof-of-concept code: caveat emptor.

An AES67 audio-over-IP implementation for XMOS processors, providing networked audio streaming with precise timing synchronization.

## Overview

lib_aes67 implements the AES67 standard for interoperable IP audio networking, designed specifically for XMOS multicore processors. It provides both receiver and sender capabilities with PTP (IEEE 1588) clock synchronization, SAP/SDP session discovery, and real-time media clock recovery.

## Features

- **AES67 Standard Compliance**: implementation of AES67 audio-over-IP standard
- **Multi-stream Support**: Configurable number of simultaneous audio streams
- **Audio Formats**: Support for L16, L24, and L32 encoding formats
- **Channel Flexibility**: Up to 8 channels per stream
- **PTP Clock Synchronization**: IEEE 1588 Precision Time Protocol support
- **Session Discovery**: SAP (Session Announcement Protocol) and SDP (Session Description Protocol)
- **Real-time Performance**: Hardware-timed audio clock recovery with PID control
- **Network Multicast**: UDP multicast for efficient audio distribution

## API Usage

### Basic Setup

```c
#include <aes67.h>
#include <ptp.h>

// Initialize AES67 manager
void aes67_manager(server interface aes67_interface i_aes67[num_clients],
                   size_t num_clients,
                   client xtcp_if i_xtcp,
                   chanend media_control);

// Initialize I/O task with PTP synchronization
void aes67_io_task(chanend ?ptp_svr,
                   chanend buf_ctl[num_buf_ctl],
                   uint32_t num_buf_ctl,
                   out buffered port:32 p_fs,
                   REFERENCE_PARAM(const aes67_media_clock_pid_coefficients_t, pid_coefficients),
                   chanend media_control,
                   client interface ethernet_cfg_if i_eth_cfg,
                   client xtcp_if i_xtcp,
                   chanend c_ptp[num_ptp],
                   uint32_t num_ptp,
                   enum ptp_server_type server_type);
```

### Receiving Audio Streams

```c
// Subscribe to a named audio session
aes67_status_t status = i_aes67.subscribe(stream_id, "MyAudioSession");

// Get received audio samples
void aes67_get_receiver_samples(int32_t id,
                                uint32_t *output_buffer,
                                size_t len,
                                uint32_t local_timestamp);

// Get all receiver samples at once
void aes67_get_all_receiver_samples(uint32_t samples[len],
                                    size_t len,
                                    uint32_t local_timestamp);
```

### Sending Audio Streams

```c
// Advertise an audio stream
uint8_t ip_addr[4] = {239, 69, 83, 1}; // Multicast address
aes67_status_t status = i_aes67.advertise(stream_id, 
                                          "MyOutputStream",
                                          ip_addr,
                                          24,  // 24-bit samples
                                          2);  // 2 channels

// Submit audio samples for transmission
aes67_init_sender_buffers();

void aes67_submit_sender_samples(chanend media,
                                 int32_t id,
                                 uint32_t samples[len],
                                 size_t len,
                                 uint32_t timestamp);
```

## Network Configuration

### Multicast Addresses

- **SAP**: 239.255.255.255:9875 (Session Announcement)
- **Audio Streams**: User-configurable multicast addresses
- **PTP**: Standard PTP multicast addresses for timing

### Firewall Considerations

Ensure the following ports are open:
- UDP 9875 (SAP)
- UDP 5004 (Default RTP audio)
- UDP 319, 320 (PTP)

## Performance Considerations

### Real-time Requirements

- **Thread Priority**: IO task should run at high priority
- **Buffer Sizing**: Configure `AUDIO_OUTPUT_FIFO_WORD_SIZE` for your latency requirements
- **Tile Allocation**: Receiver/sender tasks must run on the same tile as IO task

### Memory Usage

- Audio buffers: ~20ms buffering per AES67 specification
- Network buffers: Configurable based on packet rates
- Stream metadata: Minimal overhead per stream

## Debugging

Enable debug output:

```c
#define DEBUG_MEDIA_CLOCK 2  // Detailed media clock debugging
```

## Building

The library integrates with XMOS development tools:

1. Include lib_aes67 in your project
2. Ensure lib_ethernet and lib_xtcp dependencies
3. Configure timing resources and network interfaces
4. Build with xcc compiler

## Requirements

- XMOS multicore processor (xCORE)
- XMOS Development Tools
- lib_ethernet (Ethernet MAC support)
- lib_xtcp (TCP/IP stack)
- Hardware timing resources (ports, timers)

