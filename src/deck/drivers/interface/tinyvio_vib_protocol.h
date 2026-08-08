/**
 * TinyVIO vibration blob format  (capture type TINYVIO_CAP_TYPE_VIB)
 * ==================================================================
 * v2: this header now defines ONLY the VIB blob's content. The transport —
 * register map, capture FSM/commands, WINDOW paging, app-channel framing — is
 * the unified v0x02 contract in tinyvio_deck_protocol.h (spec: research-vault
 * tiny-cvio/deck-comms-protocol.md). The v1 ad-hoc register map that used to
 * live here COLLIDED with the pose DATA block and is gone; the vib app serves
 * the unified register file like every other deck app.
 *
 * Blob layout (little-endian, paged from WINDOW bank 0):
 *   tinyvio_vib_header_t             self-describing shape/config header
 *   float welch_avg [6][n_bins]      averaged one-sided PSD per axis (sum/count)
 *   float spec_accel[spec_nrows][spec_out_bins]   axis-summed, band-averaged
 *   float spec_gyro [spec_nrows][spec_out_bins]
 * Axis order: accel xyz, gyro xyz. PSD scaling matches
 * scipy.signal.welch(scaling='density') — see tinycvio_core/vib_monitor.h.
 * The capture's blob_size/crc32 travel in CAPTURE_STATUS + the 'H'/'E' packets.
 */
#ifndef TINYVIO_VIB_PROTOCOL_H
#define TINYVIO_VIB_PROTOCOL_H

#include <stdint.h>

#define TINYVIO_VIB_MAGIC  0x42424956u  /* 'VIBB' LE — leads every VIB blob */

/* Leading header of the VIB blob (packed, little-endian). */
typedef struct __attribute__((packed)) {
    uint32_t magic;          /* TINYVIO_VIB_MAGIC                                */
    uint8_t  ready;          /* 1 = serialized + stable (kept for layout compat)  */
    uint8_t  accel_fsr_g;    /* FSR the capture ran at (2/4/8/16/32)              */
    uint8_t  lpf_div;        /* UI-LPF divisor (0 = NO_FILTER)                    */
    uint8_t  _pad;
    uint32_t blob_size;      /* total blob bytes incl. this header                */
    uint32_t crc32;          /* IEEE CRC-32 of the whole blob (== zlib.crc32)     */
    uint32_t welch_count;    /* windows averaged into the Welch PSD               */
    uint32_t window_n;       /* FFT length                                        */
    uint32_t n_bins;         /* one-sided bins = window_n/2 + 1                   */
    uint32_t spec_nrows;     /* spectrogram rows serialized                       */
    uint32_t spec_out_bins;  /* bands per spectrogram row                         */
    uint32_t odr_hz;         /* actual configured ODR                             */
    float    fs;             /* sample rate used for PSD scaling                  */
} tinyvio_vib_header_t;

#endif /* TINYVIO_VIB_PROTOCOL_H */
