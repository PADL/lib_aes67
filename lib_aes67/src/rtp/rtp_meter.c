#include "aes67_internal.h"

#if AES67_METERING
int32_t input_peaks[AES67_NUM_MEDIA_INPUTS];
int32_t output_peaks[AES67_NUM_MEDIA_OUTPUTS];

void aes67_meter_init(enum aes67_meter_type_t type) {
  switch (type) {
  case AES67_METER_INPUT:
    memset(input_peaks, 0, sizeof(input_peaks));
    break;
  case AES67_METER_OUTPUT:
    memset(output_peaks, 0, sizeof(output_peaks));
    break;
  }
}

int aes67_meter_get(enum aes67_meter_type_t type, size_t index, int32_t *peak) {
  switch (type) {
  case AES67_METER_INPUT:
    if (index >= AES67_NUM_MEDIA_INPUTS) {
      *peak = 0;
      return 0;
    } else {
      *peak = input_peaks[index];
      input_peaks[index] = 0;
      return 1;
    }
  case AES67_METER_OUTPUT:
    if (index >= AES67_NUM_MEDIA_OUTPUTS) {
      *peak = 0;
      return 0;
    } else {
      *peak = output_peaks[index];
      output_peaks[index] = 0;
      return 1;
    }
  default:
    return 0;
    break;
  }
}
#endif
