# lib\_aes67

This is proof-of-concept code: caveat emptor.

An AES67 audio-over-IP implementation for XMOS processors, providing networked audio streaming with precise timing synchronization.

## Overview

## Features

- **AES67 Standard Compliance**: implementation of AES67 audio-over-IP standard
- **Multi-stream Support**: Configurable (at compile time) number of simultaneous audio streams
- **Audio Formats**: Support for L16, L24, and L32 encodings
- **Channel Flexibility**: Configurable number of channels per stream (depending on resources)
- **PTP Clock Synchronization**: IEEE 1588 PTPv2 support
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
                   uint32_t flags);
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

### Ports Used

- UDP 9875 (SAP)
- UDP 5004 (Default RTP audio)
- UDP 319, 320 (PTP)

## Performance Considerations

### Real-time Requirements

- **Thread Priority**: IO task should run at high priority
- **Buffer Sizing**: Configure `AUDIO_OUTPUT_FIFO_WORD_SIZE` for your latency requirements
- **Tile Allocation**: Receiver/sender tasks must run on the same tile as IO task

## Dependencies

- lib\_ethernet (Ethernet MAC support)
- lib\_xtcp (TCP/IP stack)
- lwIP (TCP/IP stack)
