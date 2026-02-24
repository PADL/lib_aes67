# lib\_aes67

This is proof-of-concept code: caveat emptor.

An AES67 audio-over-IP implementation for XMOS processors, loosely based on the XMOS [lib\_tsn](https://github.com/xmos/lib_tsn) library.

At the time of writing, only single receiver usage has been validated, on a custom board with a CS2600 clock chip (different chips may need different PID coefficients). As we are testing it with a proprietary mixer firmware, there are presently no example applications.

## Overview

## API Usage

### Basic Setup

```c
#include <aes67.h>

// Initialize AES67 manager
void aes67_manager(server interface aes67_interface i_aes67[num_clients],
                   size_t num_clients,
                   client xtcp_if i_xtcp,
                   chanend media_control);

// Initialize I/O task with PTP synchronization
void aes67_io_task(chanend buf_ctl[num_buf_ctl],
                   uint32_t num_buf_ctl,
                   out buffered port:32 p_fs,
                   REFERENCE_PARAM(const aes67_media_clock_pid_coefficients_t, pid_coefficients),
                   chanend media_control,
                   client interface ethernet_cfg_if i_eth_cfg,
                   client xtcp_if i_xtcp,
                   uint32_t flags);
```

### Receiving Audio Streams

```c
// Subscribe to a named audio session
aes67_status_t status = i_aes67.subscribe(stream_id, "MyAudioSession");

// Get received audio samples, returns number of samples written
size_t aes67_get_receiver_samples(int32_t id,
                                uint32_t *output_buffer,
                                size_t len,
                                uint32_t local_timestamp);

// Get all receiver samples at once, returns number of samples written
size_t aes67_get_all_receiver_samples(uint32_t samples[len],
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

## Dependencies

- lib\_ethernet (Ethernet MAC support)
- lib\_xtcp (TCP/IP stack)
- lwIP (TCP/IP stack)
