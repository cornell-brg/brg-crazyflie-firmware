/**
 * TinyVIO vibration bulk-read protocol  (deck <-> Crazyflie <-> host)
 * ==================================================================
 * INDEPENDENT of the pose protocol (tinyvio_deck_protocol.h) — it only shares the
 * physical I2C slave address + the CRTP/app-channel transport, NOT the pose/Nav
 * register logic. The deck (deck_imu_vib.cc) computes the vibration PSD on-chip
 * and serves a serialized "blob" (averaged Welch PSD + coarse spectrogram) as an
 * I2C slave; the CF (tinyvioDeck.c) pages the blob over I2C and forwards it to the
 * host over the app-channel for an untethered post-land download.
 *
 * Pure C header (no Eigen / no C++), so it is shared verbatim by the deck (C++),
 * the CF firmware (C), and mirrors the host reassembler (dump_vib_radio.py).
 *
 * ── I2C register map (8-bit reg pointer, slave = TINYVIO_I2C_ADDR) ─────────────
 *   REG_HEADER  0x00  read  : tinyvio_vib_header_t (blob shape + size + crc)
 *   REG_BANK    0x40  write : u16 LE page index into the blob (window = bank*128)
 *   REG_CONTROL 0x44  write : u8 TINYVIO_VIB_CTL_* (freeze / reset)
 *   REG_WINDOW  0x80  read  : 128 B = blob[bank*128 + (reg - 0x80)]
 *
 * Paging: the blob is >256 B, so the master selects a 128-byte page via REG_BANK,
 * then burst-reads REG_WINDOW. Fully addressed (bank + window offset) — no
 * stateful auto-advance cursor in the ISR.
 *
 * ── Blob layout (little-endian f32, paged from bank 0) ────────────────────────
 *   welch_avg  : float[6][n_bins]                 averaged PSD per axis (sum/count)
 *   spec_accel : float[spec_nrows][spec_out_bins] axis-summed accel, band-averaged
 *   spec_gyro  : float[spec_nrows][spec_out_bins] axis-summed gyro,  band-averaged
 *   blob_size  = (6*n_bins + 2*spec_nrows*spec_out_bins) * sizeof(float)
 */
#ifndef TINYVIO_VIB_PROTOCOL_H
#define TINYVIO_VIB_PROTOCOL_H

#include <stdint.h>

#define TINYVIO_VIB_I2C_ADDR   0x42u        /* same slave addr as the pose app (separate app) */
#define TINYVIO_VIB_MAGIC      0x42424956u  /* 'VIBB' LE — presence + endianness sanity        */

/* I2C register pointers */
#define TINYVIO_VIB_REG_HEADER   0x00u
#define TINYVIO_VIB_REG_BANK     0x40u      /* write u16 LE page index                          */
#define TINYVIO_VIB_REG_CONTROL  0x44u      /* write u8 control                                 */
#define TINYVIO_VIB_REG_WINDOW   0x80u      /* read 128 B window                                */
#define TINYVIO_VIB_WINDOW_SIZE  128u

/* REG_CONTROL commands */
#define TINYVIO_VIB_CTL_FREEZE   0x01u      /* stop accumulating + serialize the blob (page-safe) */
#define TINYVIO_VIB_CTL_RESET    0x02u      /* clear accumulators, resume a fresh capture         */

/* Header served at REG_HEADER (packed, little-endian). */
typedef struct __attribute__((packed)) {
    uint32_t magic;          /* TINYVIO_VIB_MAGIC                                   */
    uint8_t  ready;          /* 1 = blob serialized + stable (paging is safe)       */
    uint8_t  accel_fsr_g;
    uint8_t  lpf_div;
    uint8_t  _pad;
    uint32_t blob_size;      /* bytes to page from REG_WINDOW                        */
    uint32_t crc32;          /* CRC-32 (IEEE, poly 0xEDB88320) of the whole blob    */
    uint32_t welch_count;    /* windows averaged into the Welch PSD                 */
    uint32_t window_n;       /* FFT length                                          */
    uint32_t n_bins;         /* one-sided bins = N/2 + 1                            */
    uint32_t spec_nrows;     /* spectrogram rows serialized                         */
    uint32_t spec_out_bins;  /* bands per spectrogram row                           */
    uint32_t odr_hz;
    float    fs;
} tinyvio_vib_header_t;

/* ── App-channel framing (CF -> host, APPCHANNEL_MTU = 31 B) ───────────────────
 * Packets are hand-packed (no struct, to dodge padding). First byte = kind:
 *   'H' header : [u8 'H'][u32 blob_size][u16 n_bins][u16 spec_nrows]
 *                [u16 spec_out_bins][u16 window_n][u32 welch_count][u16 odr_hz]
 *                [f32 fs][u32 crc32][u8 accel_fsr_g][u8 lpf_div]        (29 B)
 *   'D' data   : [u8 'D'][u32 offset][payload <= 25 B of blob bytes]    (<=30 B)
 *   'E' done   : [u8 'E'][u32 blob_size][u32 crc32]                     (9 B)
 *
 * NOTE: app-channel packets MUST be <= 30 B (CRTP_MAX_DATA_SIZE), NOT the 31 that
 * APPCHANNEL_MTU advertises — the CF firmware's two constants disagree by one byte
 * and crtpSendPacketBlock ASSERTs (and reboots) on 31. So 'D' payload caps at 25.
 */
#define TINYVIO_VIB_APP_HEADER 'H'
#define TINYVIO_VIB_APP_DATA   'D'
#define TINYVIO_VIB_APP_DONE   'E'
#define TINYVIO_VIB_APP_DATA_PAYLOAD 25u   /* blob bytes per 'D' packet (1 + 4 + 25 = 30, the CRTP cap) */

#endif /* TINYVIO_VIB_PROTOCOL_H */
