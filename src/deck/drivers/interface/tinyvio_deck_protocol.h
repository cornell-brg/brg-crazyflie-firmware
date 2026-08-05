/**
 * tinyvio_deck_protocol.h — canonical wire protocol for the TinyVIO deck ⇄ Crazyflie I2C link.
 *
 * SINGLE SOURCE OF TRUTH. This header is vendored VERBATIM into both codebases:
 *   - deck (RP2350, C++):  slave — publishes into this register file.
 *   - Crazyflie (STM32, C): master — polls this register file, feeds the estimator.
 * Keep the two copies byte-identical; bump TINYVIO_PROTO_VERSION on any layout change.
 *
 * ── Model ────────────────────────────────────────────────────────────────────
 * Memory-mapped register file over I2C. The RP2350 is the SLAVE at
 * TINYVIO_I2C_ADDR; the Crazyflie STM32 is the MASTER and initiates everything
 * (a slave cannot push — it publishes into registers, the master polls). Access
 * is the standard register-pointer protocol (i2cdevReadReg8 / i2cdevWriteReg8):
 * write the 1-byte register offset, then read/write with auto-increment.
 *
 * The register space is described by `tinyvio_regs_t`, a packed struct whose field
 * offsets ARE the register addresses (use offsetof / the TINYVIO_REG_* bases).
 * Four blocks, each on a clean 0x10 boundary with reserved tailroom:
 *
 *   0x00 IDENTITY      (RO, static)   who/version/caps — detect + compat check
 *   0x10 STATUS        (RO, dynamic)  lifecycle state + health + liveness
 *   0x20 CONTROL       (RW)           master commands the deck (ack-by-sequence)
 *   0x30 TIMESYNC      (RW)           master publishes its clock for offset est.
 *   0x40 DATA          (RO, dynamic)  the estimate — seqlock-guarded pose block
 *   0x80 CAPTURE_CTRL  (RW)   v2      capture commands + runtime IMU config
 *   0x90 CAPTURE_STAT  (RO)   v2      capture FSM state + blob size/CRC
 *   0xA0 TELEM         (RO)   v2      fixed-point live telemetry (the LOG menu)
 *   0xC0 WINDOW        (RO)   v2      64-B bulk page: blob[cap_bank*64 + i]
 *
 * Every deck app presents THIS register file; IDENTITY.capabilities declares
 * what it hosts (pose, telem, capture). One CF driver serves all deck apps.
 *
 * ── Conventions ──────────────────────────────────────────────────────────────
 *   - Little-endian, packed, fixed-width. Both ends are LE ARM with an FPU.
 *   - float32 (IEEE-754) for real values; unit meters, seconds, m/s.
 *   - The estimate is ODOMETRY: metric scale (known board geometry + IMU) in a
 *     gravity-aligned frame whose origin+yaw are fixed at VIO init. It DRIFTS —
 *     treat it as local state feedback for control, not a global reference.
 *   - Quaternion order follows tinycvio/OpenVINS (JPL). Confirm against the
 *     consumer's convention (e.g. TinyMPC) before wiring; the layout is 4 floats
 *     either way, only the semantics differ.
 *   - Coherent DATA reads use a seqlock: read the whole block in one burst and
 *     accept it only if seq_begin == seq_end (see tinyvio_data_is_coherent).
 *   - The DATA block IS TinyMPC's state feedback: (pos, vel, quat) + timestamp +
 *     quality. Angular velocity is intentionally absent — the flight controller
 *     takes ω from its own gyro (fast loop); the deck supplies the slow nav state.
 */
#ifndef TINYVIO_DECK_PROTOCOL_H
#define TINYVIO_DECK_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

#if defined(__cplusplus)
  extern "C" {
  #define TINYVIO_STATIC_ASSERT(cond, msg) static_assert((cond), msg)
#else
  #include <stdbool.h>
  #define TINYVIO_STATIC_ASSERT(cond, msg) _Static_assert((cond), msg)
#endif

#if defined(__BYTE_ORDER__) && (__BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__)
  #error "tinyvio_deck_protocol.h assumes a little-endian target"
#endif

/* ── Link constants ─────────────────────────────────────────────────────────── */
#define TINYVIO_I2C_ADDR       0x42u   /* 7-bit; clear of CF EEPROM 0x50-0x57      */
#define TINYVIO_MAGIC          0x5654u /* 'TV' LE — presence + endianness sanity   */
#define TINYVIO_WHO_AM_I       0x56u   /* 'V'                                      */
#define TINYVIO_PROTO_VERSION  0x02u   /* v2: quality→fault renames + CAPTURE/TELEM/
                                          WINDOW blocks. Spec: research-vault
                                          tiny-cvio/deck-comms-protocol.md          */

/* ── Block base register offsets ────────────────────────────────────────────── */
#define TINYVIO_REG_IDENTITY      0x00u
#define TINYVIO_REG_STATUS        0x10u
#define TINYVIO_REG_CONTROL       0x20u
#define TINYVIO_REG_TIMESYNC      0x30u
#define TINYVIO_REG_DATA          0x40u
/* 0x78..0x7F reserved: DATA growth headroom (v2)                                 */
#define TINYVIO_REG_CAPTURE_CTRL  0x80u  /* v2: capture commands (master-written)  */
#define TINYVIO_REG_CAPTURE_STAT  0x90u  /* v2: capture state (non-destructive)    */
#define TINYVIO_REG_TELEM         0xA0u  /* v2: fixed-point telemetry (LOG menu)   */
#define TINYVIO_REG_WINDOW        0xC0u  /* v2: 64-B bulk page window (read-only)  */
#define TINYVIO_WINDOW_SIZE       64u    /* WINDOW[i] = blob[cap_bank*64 + i]      */

/* ── Capabilities bitmap (identity.capabilities) ────────────────────────────── */
#define TINYVIO_CAP_POSE       (1u << 0) /* position valid                        */
#define TINYVIO_CAP_ATTITUDE   (1u << 1) /* quaternion valid                      */
#define TINYVIO_CAP_VELOCITY   (1u << 2) /* linear velocity valid                 */
#define TINYVIO_CAP_TELEM      (1u << 3) /* v2: TELEM block live (σ from cov)     */
#define TINYVIO_CAP_TIMESTAMP  (1u << 4) /* data.timestamp_us valid               */
#define TINYVIO_CAP_TIMESYNC   (1u << 5) /* honours TIMESYNC block                */
#define TINYVIO_CAP_CAPTURE    (1u << 6) /* v2: capture FSM + WINDOW paging       */

/* ── Health bitmap (status.health) — 1 = healthy ────────────────────────────── */
#define TINYVIO_HEALTH_CAMERA  (1u << 0)
#define TINYVIO_HEALTH_IMU     (1u << 1)
#define TINYVIO_HEALTH_CALIB   (1u << 2)
#define TINYVIO_HEALTH_CLOCK   (1u << 3)

/* ── Lifecycle state (status.state) ─────────────────────────────────────────── */
typedef enum {
    TINYVIO_STATE_BOOT        = 0, /* powered, pre-init                            */
    TINYVIO_STATE_INIT        = 1, /* bringing up camera/IMU/VIO                   */
    TINYVIO_STATE_CALIBRATING = 2, /* IMU bias / alignment                         */
    TINYVIO_STATE_READY       = 3, /* initialized, not yet tracking                */
    TINYVIO_STATE_TRACKING    = 4, /* producing valid pose  ← master may consume   */
    TINYVIO_STATE_DEGRADED    = 5, /* tracking, low confidence (down-weight)       */
    TINYVIO_STATE_LOST        = 6, /* tracking lost  ← master MUST stop consuming   */
    TINYVIO_STATE_ERROR       = 7, /* fault, see error_code                        */
} tinyvio_state_t;

/* ── Commands (control.cmd, master-written) ─────────────────────────────────── */
typedef enum {
    TINYVIO_CMD_NONE        = 0,
    TINYVIO_CMD_RESET       = 1, /* soft-reset the VIO pipeline                     */
    TINYVIO_CMD_START       = 2, /* begin tracking                                 */
    TINYVIO_CMD_STOP        = 3, /* stop / idle                                    */
    TINYVIO_CMD_ENTER_CALIB = 4, /* (re)calibrate                                  */
    TINYVIO_CMD_SET_ORIGIN  = 5, /* zero the pose origin to current pose           */
} tinyvio_cmd_t;

/* ── Error codes (status.error_code) ────────────────────────────────────────── */
typedef enum {
    TINYVIO_ERR_NONE        = 0,
    TINYVIO_ERR_CAMERA      = 1,
    TINYVIO_ERR_IMU         = 2,
    TINYVIO_ERR_DIVERGED    = 3, /* filter diverged                                */
    TINYVIO_ERR_INTERNAL    = 4,
    TINYVIO_ERR_MISSEATED   = 5, /* v2: PadInit consistency check failed — deck    */
                                 /*     mis-seated on the pad. REFUSE TAKEOFF.     */
} tinyvio_error_t;

/* ══ v2: CAPTURE — the one-shot record lifecycle ═════════════════════════════ */
/* Single buffer, consume-before-overwrite. 3-state FSM; the HOST owns the       */
/* consume-confirm (deck cannot know delivery): page → CRC-verify → RELEASE.     */
/* Orthogonal to the estimator FSM: est-RESET never touches capture state.       */

/* Capture FSM (capture_stat.cap_state) */
typedef enum {
    TINYVIO_CAP_IDLE      = 0, /* no capture; buffer overwritable                  */
    TINYVIO_CAP_CAPTURING = 1, /* accumulating (live counters advisory)            */
    TINYVIO_CAP_READY     = 2, /* frozen+serialized; blob_size/crc32 stable; page  */
                               /* freely; START refused until RELEASE/DISCARD      */
} tinyvio_cap_state_t;

/* Capture commands (capture_ctrl.cap_cmd, master-written) */
typedef enum {
    TINYVIO_CAP_CMD_NONE    = 0,
    TINYVIO_CAP_CMD_START   = 1, /* begin capture of cap_type w/ cap_odr/lpf/fsr   */
    TINYVIO_CAP_CMD_STOP    = 2, /* gate off → serialize → CRC → READY (set last)  */
    TINYVIO_CAP_CMD_RELEASE = 3, /* host confirms consumed → IDLE                  */
    TINYVIO_CAP_CMD_DISCARD = 4, /* release without download → IDLE                */
} tinyvio_cap_cmd_t;

/* Capture types (capture_ctrl.cap_type). Which types an app hosts is declared   */
/* per-app via IDENTITY.capabilities + refused with BAD_TYPE otherwise.          */
typedef enum {
    TINYVIO_CAP_TYPE_NONE       = 0,
    TINYVIO_CAP_TYPE_VIB        = 1, /* onboard-FFT vibration blob (vib app)      */
    TINYVIO_CAP_TYPE_DIAG       = 2, /* end-of-run ledgers (VIO app)              */
    TINYVIO_CAP_TYPE_FRAMETELEM = 3, /* per-frame timing ledger (TELEM builds)    */
} tinyvio_cap_type_t;

/* Command result (capture_ctrl.cap_cmd_status) — unlike the est handshake,      */
/* this one can REFUSE. Valid once cap_cmd_ack_seq == cap_cmd_seq.               */
typedef enum {
    TINYVIO_CAP_ST_NONE              = 0, /* no command processed yet             */
    TINYVIO_CAP_ST_OK                = 1,
    TINYVIO_CAP_ST_REFUSED_UNDRAINED = 2, /* START while READY: RELEASE/DISCARD first */
    TINYVIO_CAP_ST_REFUSED_BUSY      = 3, /* command invalid while CAPTURING      */
    TINYVIO_CAP_ST_BAD_TYPE          = 4, /* type not hosted by this app          */
    TINYVIO_CAP_ST_BAD_STATE         = 5, /* command meaningless in current state */
    TINYVIO_CAP_ST_BAD_CONFIG        = 6, /* odr/lpf/fsr combination rejected     */
} tinyvio_cap_cmd_status_t;

/* IMU ODR codes (capture_ctrl.cap_odr_code) — u8-encodable rates.               */
/* 6400 is defined but the ICM-45686 dual accel+gyro FIFO path cannot sustain    */
/* it (silicon limit, verified) — decks refuse it with BAD_CONFIG.               */
typedef enum {
    TINYVIO_ODR_KEEP = 0,  /* keep the app's current/default rate                 */
    TINYVIO_ODR_400  = 1,
    TINYVIO_ODR_800  = 2,
    TINYVIO_ODR_1600 = 3,
    TINYVIO_ODR_3200 = 4,
    TINYVIO_ODR_6400 = 5,
} tinyvio_odr_code_t;

static inline uint16_t tinyvio_odr_hz(uint8_t code) {
    switch (code) {
        case TINYVIO_ODR_400:  return 400u;
        case TINYVIO_ODR_800:  return 800u;
        case TINYVIO_ODR_1600: return 1600u;
        case TINYVIO_ODR_3200: return 3200u;
        case TINYVIO_ODR_6400: return 6400u;
        default:               return 0u;   /* KEEP / invalid */
    }
}

/* ══ v2: app-channel bulk framing (CF → host, generalized over capture types) ═ */
/* ⚠ Packets MUST be ≤ 30 B: the CF firmware's APPCHANNEL_MTU says 31 but        */
/* crtpSendPacketBlock ASSERTs (and reboots the CF) at >CRTP_MAX_DATA_SIZE=30.   */
/*   'H' header: [u8 'H'][u8 cap_type][u16 cap_label][u32 blob_size][u32 crc32]  */
/*              = 12 B, once, before any data. Type-specific shape lives in the  */
/*              blob's own leading header (blobs are self-describing).           */
/*   'D' data:   [u8 'D'][u32 offset][payload ≤ 25 B]  — reassemble by offset.   */
/*   'E' done:   [u8 'E'][u32 blob_size][u32 crc32]    = 9 B, integrity check.   */
#define TINYVIO_APP_HEADER        'H'
#define TINYVIO_APP_DATA          'D'
#define TINYVIO_APP_DONE          'E'
#define TINYVIO_APP_DATA_PAYLOAD  25u  /* 1 + 4 + 25 = 30 = the CRTP hard cap    */

/* ── Register blocks (packed; offsets ARE register addresses) ───────────────── */
#if defined(__GNUC__)
  #define TINYVIO_PACKED __attribute__((packed))
#else
  #define TINYVIO_PACKED
#endif

typedef struct TINYVIO_PACKED {
    uint16_t magic;         /* 0x00  TINYVIO_MAGIC                                 */
    uint8_t  who_am_i;      /* 0x02  TINYVIO_WHO_AM_I                              */
    uint8_t  proto_version; /* 0x03  TINYVIO_PROTO_VERSION                         */
    uint16_t fw_version;    /* 0x04  deck fw: (major<<8)|minor                     */
    uint8_t  hw_revision;   /* 0x06  board rev                                     */
    uint8_t  capabilities;  /* 0x07  TINYVIO_CAP_* bitmap                          */
    uint8_t  _reserved[8];  /* 0x08..0x0F                                          */
} tinyvio_identity_t;

typedef struct TINYVIO_PACKED {
    uint8_t  state;         /* 0x10  tinyvio_state_t                               */
    uint8_t  health;        /* 0x11  TINYVIO_HEALTH_* bitmap (sensor-level, 1=OK)  */
    uint8_t  error_code;    /* 0x12  tinyvio_error_t                               */
    uint8_t  fault;         /* 0x13  v2 rename (was track_quality): the Vitals     */
                            /*       estimator fault byte — 0 = HEALTHY. No        */
                            /*       synthetic confidence metric; graded            */
                            /*       uncertainty = TELEM std_pos/std_att.          */
    uint32_t alive_counter; /* 0x14  free-running, +1 per VIO cycle (liveness)     */
    uint8_t  _reserved[8];  /* 0x18..0x1F                                          */
} tinyvio_status_t;

typedef struct TINYVIO_PACKED {
    uint8_t  cmd;           /* 0x20  tinyvio_cmd_t (master writes)                 */
    uint8_t  cmd_seq;       /* 0x21  master increments once per command            */
    uint8_t  cmd_ack_seq;   /* 0x22  deck echoes the applied cmd_seq (RO to master)*/
    uint8_t  config_mode;   /* 0x23  reserved mode selector                        */
    uint16_t config_rate_hz;/* 0x24  master's requested output-rate hint           */
    uint8_t  _reserved[10]; /* 0x26..0x2F                                          */
} tinyvio_control_t;

typedef struct TINYVIO_PACKED {
    uint64_t host_time_us;  /* 0x30  master writes its clock (for offset est.)     */
    uint8_t  _reserved[8];  /* 0x38..0x3F                                          */
} tinyvio_timesync_t;

/* DATA = TinyMPC state feedback. To change it: edit this struct, update the
 * static_asserts below + NavState/load/store in tinyvio_protocol.hpp, and bump
 * TINYVIO_PROTO_VERSION. Offsets are relative to the block base (0x40). */
typedef struct TINYVIO_PACKED {
    uint16_t seq_begin;     /* +0x00  seqlock: bumped BEFORE the deck writes payload*/
    uint16_t _reserved0;    /* +0x02  align timestamp                              */
    uint64_t timestamp_us;  /* +0x04  deck monotonic time-of-validity              */
    float    pos[3];        /* +0x0C  x,y,z    (m)                                 */
    float    vel[3];        /* +0x18  vx,vy,vz (m/s)                               */
    float    quat[4];       /* +0x24  orientation (JPL; confirm vs consumer)       */
    uint8_t  fault;         /* +0x34  v2 rename (was quality): Vitals fault byte,  */
                            /*        0 = healthy — matches what the code always   */
                            /*        wrote here; the doc now agrees with the code */
    uint8_t  _reserved1;    /* +0x35                                               */
    uint16_t seq_end;       /* +0x36  == seq_begin when the frame is coherent      */
} tinyvio_data_t;

/* v2: CAPTURE_CONTROL — master-written capture commands + runtime IMU config.
 * CAP_START(VIB) applies odr/lpf/fsr with the bench-grid-validated reconfig
 * sequence: stop streaming → set rate/FSR → apply LPF (div=0 must ACTIVELY
 * reset to NO_FILTER — sticky-LPF bug) → flush_fifo() → restart → gate on. */
typedef struct TINYVIO_PACKED {
    uint8_t  cap_cmd;        /* +0x00  tinyvio_cap_cmd_t (master writes)           */
    uint8_t  cap_type;       /* +0x01  tinyvio_cap_type_t for START                */
    uint16_t cap_label;      /* +0x02  test id, echoed into status + 'H' packet    */
    uint8_t  cap_cmd_seq;    /* +0x04  master increments once per command          */
    uint8_t  cap_cmd_ack_seq;/* +0x05  deck echoes the processed cmd_seq           */
    uint8_t  cap_cmd_status; /* +0x06  tinyvio_cap_cmd_status_t for that command   */
    uint8_t  _reserved0;     /* +0x07                                              */
    uint16_t cap_bank;       /* +0x08  WINDOW page select (a control write)        */
    uint8_t  cap_odr_code;   /* +0x0A  tinyvio_odr_code_t (0 = keep current)       */
    uint8_t  cap_lpf_div;    /* +0x0B  UI-LPF divisor: 0=NO_FILTER,4,8,16,32,64,128*/
    uint8_t  cap_fsr_g;      /* +0x0C  accel FSR in g: 0=keep, 2,4,8,16,32         */
    uint8_t  _reserved1[3];  /* +0x0D..0x0F                                        */
} tinyvio_capture_ctrl_t;

/* v2: CAPTURE_STATUS — the non-destructive "is a capture ready?" poll.
 * blob_size/crc32 are written during freeze and STABLE-WHEN-READY (cap_state
 * is set to READY last). cap_count/cap_elapsed_s are ADVISORY while capturing
 * (may tear mid-burst; never decision inputs). */
typedef struct TINYVIO_PACKED {
    uint8_t  cap_state;      /* +0x00  tinyvio_cap_state_t                         */
    uint8_t  cap_type;       /* +0x01  type of the current/last capture            */
    uint16_t cap_label;      /* +0x02  label echo                                  */
    uint32_t blob_size;      /* +0x04  bytes pageable via WINDOW (stable @READY)   */
    uint32_t crc32;          /* +0x08  IEEE CRC-32 of the blob (== zlib.crc32)     */
    uint16_t cap_count;      /* +0x0C  live progress (e.g. Welch windows) ADVISORY */
    uint16_t cap_elapsed_s;  /* +0x0E  seconds since START                ADVISORY */
} tinyvio_capture_stat_t;

/* v2: TELEM — fixed-point live telemetry backing the CF LOG menu. One burst
 * read; ADVISORY (single-field atomicity; a straddled burst may mix frames).
 * Encodings clamp-never-wrap (a wrapped value would read small exactly when
 * diverging). Ranges sized just past the Vitals fatal rails — beyond a rail
 * the fault byte has already fired. Pose is NOT here (full-f32 in DATA). */
#define TINYVIO_TELEM_STDPOS_LSB_M     1.0e-4f /* 0.1 mm; caps 6.55 m (rail 5 m)  */
#define TINYVIO_TELEM_STDATT_LSB_RAD   1.0e-5f /* 0.01 mrad; caps ~37.5°          */
#define TINYVIO_TELEM_BG_LSB_RADS      1.0e-5f /* 0.01 mrad/s; ±327 (rail 100)    */
#define TINYVIO_TELEM_BA_LSB_MS2       1.0e-4f /* 0.1 mm/s²; ±3.27 (rail 2.0)     */
typedef struct TINYVIO_PACKED {
    uint16_t frame_id;       /* +0x00  wraps                                       */
    uint16_t std_pos[3];     /* +0x02  σ position,  LSB TINYVIO_TELEM_STDPOS_LSB_M */
    uint16_t std_att[3];     /* +0x08  σ attitude,  LSB TINYVIO_TELEM_STDATT_LSB_RAD*/
    int16_t  bg[3];          /* +0x0E  gyro bias,   LSB TINYVIO_TELEM_BG_LSB_RADS  */
    int16_t  ba[3];          /* +0x14  accel bias,  LSB TINYVIO_TELEM_BA_LSB_MS2   */
    uint8_t  n_folded;       /* +0x1A  folds selected this frame                   */
    uint8_t  n_accepted;     /* +0x1B  folds accepted (starvation pair)            */
    uint16_t update_us;      /* +0x1C  update latency                              */
    uint8_t  fault;          /* +0x1E  Vitals byte copy (one-burst telem read)     */
    uint8_t  clones;         /* +0x1F  live clone count                            */
} tinyvio_telem_t;

typedef struct TINYVIO_PACKED {
    tinyvio_identity_t     identity;     /* 0x00 */
    tinyvio_status_t       status;       /* 0x10 */
    tinyvio_control_t      control;      /* 0x20 */
    tinyvio_timesync_t     timesync;     /* 0x30 */
    tinyvio_data_t         data;         /* 0x40 */
    uint8_t                _reserved[8]; /* 0x78  DATA growth headroom             */
    tinyvio_capture_ctrl_t capture_ctrl; /* 0x80  v2                               */
    tinyvio_capture_stat_t capture_stat; /* 0x90  v2                               */
    tinyvio_telem_t        telem;        /* 0xA0  v2                               */
    uint8_t                window[TINYVIO_WINDOW_SIZE]; /* 0xC0  v2 (RO)           */
} tinyvio_regs_t;

/* ── Layout guards (fail the build if the map ever drifts) ──────────────────── */
TINYVIO_STATIC_ASSERT(sizeof(tinyvio_identity_t) == 0x10, "identity block != 16B");
TINYVIO_STATIC_ASSERT(sizeof(tinyvio_status_t)   == 0x10, "status block != 16B");
TINYVIO_STATIC_ASSERT(sizeof(tinyvio_control_t)  == 0x10, "control block != 16B");
TINYVIO_STATIC_ASSERT(sizeof(tinyvio_timesync_t) == 0x10, "timesync block != 16B");
TINYVIO_STATIC_ASSERT(offsetof(tinyvio_regs_t, status)   == TINYVIO_REG_STATUS,   "status base");
TINYVIO_STATIC_ASSERT(offsetof(tinyvio_regs_t, control)  == TINYVIO_REG_CONTROL,  "control base");
TINYVIO_STATIC_ASSERT(offsetof(tinyvio_regs_t, timesync) == TINYVIO_REG_TIMESYNC, "timesync base");
TINYVIO_STATIC_ASSERT(offsetof(tinyvio_regs_t, data)     == TINYVIO_REG_DATA,     "data base");
TINYVIO_STATIC_ASSERT(sizeof(tinyvio_data_t) == 0x38, "data block size drift");
TINYVIO_STATIC_ASSERT(offsetof(tinyvio_data_t, timestamp_us) == 0x04, "data.timestamp offset");
TINYVIO_STATIC_ASSERT(offsetof(tinyvio_data_t, pos)     == 0x0C, "data.pos offset");
TINYVIO_STATIC_ASSERT(offsetof(tinyvio_data_t, vel)     == 0x18, "data.vel offset");
TINYVIO_STATIC_ASSERT(offsetof(tinyvio_data_t, quat)    == 0x24, "data.quat offset");
TINYVIO_STATIC_ASSERT(offsetof(tinyvio_data_t, seq_end) == 0x36, "data.seq_end offset");
/* v2 blocks */
TINYVIO_STATIC_ASSERT(sizeof(tinyvio_capture_ctrl_t) == 0x10, "capture_ctrl block != 16B");
TINYVIO_STATIC_ASSERT(sizeof(tinyvio_capture_stat_t) == 0x10, "capture_stat block != 16B");
TINYVIO_STATIC_ASSERT(sizeof(tinyvio_telem_t)        == 0x20, "telem block != 32B");
TINYVIO_STATIC_ASSERT(offsetof(tinyvio_regs_t, capture_ctrl) == TINYVIO_REG_CAPTURE_CTRL, "capture_ctrl base");
TINYVIO_STATIC_ASSERT(offsetof(tinyvio_regs_t, capture_stat) == TINYVIO_REG_CAPTURE_STAT, "capture_stat base");
TINYVIO_STATIC_ASSERT(offsetof(tinyvio_regs_t, telem)        == TINYVIO_REG_TELEM,        "telem base");
TINYVIO_STATIC_ASSERT(offsetof(tinyvio_regs_t, window)       == TINYVIO_REG_WINDOW,       "window base");
TINYVIO_STATIC_ASSERT(offsetof(tinyvio_capture_ctrl_t, cap_bank)     == 0x08, "cap_bank offset");
TINYVIO_STATIC_ASSERT(offsetof(tinyvio_capture_ctrl_t, cap_odr_code) == 0x0A, "cap_odr offset");
TINYVIO_STATIC_ASSERT(offsetof(tinyvio_capture_stat_t, blob_size)    == 0x04, "blob_size offset");
TINYVIO_STATIC_ASSERT(offsetof(tinyvio_capture_stat_t, crc32)        == 0x08, "crc32 offset");
TINYVIO_STATIC_ASSERT(offsetof(tinyvio_telem_t, std_pos) == 0x02, "telem.std_pos offset");
TINYVIO_STATIC_ASSERT(offsetof(tinyvio_telem_t, bg)      == 0x0E, "telem.bg offset");
TINYVIO_STATIC_ASSERT(offsetof(tinyvio_telem_t, fault)   == 0x1E, "telem.fault offset");
TINYVIO_STATIC_ASSERT(sizeof(tinyvio_regs_t) == 0x100, "register file must be exactly 256B in v2");

/* ── Helpers (shared by both sides) ─────────────────────────────────────────── */

/* DATA is coherent iff both seqlock ends match (single-burst read caught no update). */
static inline bool tinyvio_data_is_coherent(const tinyvio_data_t* d) {
    return d->seq_begin == d->seq_end;
}

/* Master compat check: identity is a real, layout-matching TinyVIO deck. */
static inline bool tinyvio_identity_ok(const tinyvio_identity_t* id) {
    return id->magic == TINYVIO_MAGIC &&
           id->who_am_i == TINYVIO_WHO_AM_I &&
           id->proto_version == TINYVIO_PROTO_VERSION;
}

/* Master gate: is the estimate safe to feed the estimator this cycle? */
static inline bool tinyvio_pose_usable(const tinyvio_status_t* s) {
    return s->state == TINYVIO_STATE_TRACKING || s->state == TINYVIO_STATE_DEGRADED;
}

/* v2: TELEM fixed-point encoders — CLAMP, never wrap. A wrapped value would
 * display a diverging filter as a small number at exactly the moment it
 * matters; a pegged max unambiguously reads "at or beyond cap". */
static inline uint16_t tinyvio_telem_enc_u16(float v, float lsb) {
    float q = v / lsb;
    if (q <= 0.0f)      return 0u;
    if (q >= 65535.0f)  return 65535u;
    return (uint16_t)(q + 0.5f);
}
static inline int16_t tinyvio_telem_enc_i16(float v, float lsb) {
    float q = v / lsb;
    if (q <= -32768.0f) return (int16_t)-32768;
    if (q >=  32767.0f) return (int16_t) 32767;
    return (int16_t)(q >= 0.0f ? q + 0.5f : q - 0.5f);
}
static inline float tinyvio_telem_dec(int32_t raw, float lsb) { return (float)raw * lsb; }

#if defined(__cplusplus)
}  /* extern "C" */
#endif

#endif /* TINYVIO_DECK_PROTOCOL_H */
