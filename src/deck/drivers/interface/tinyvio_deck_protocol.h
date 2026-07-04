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
 *   0x00 IDENTITY  (RO, static)   who/version/caps — detect + compat check
 *   0x10 STATUS    (RO, dynamic)  lifecycle state + health + liveness
 *   0x20 CONTROL   (RW)           master commands the deck (ack-by-sequence)
 *   0x30 TIMESYNC  (RW)           master publishes its clock for offset estimation
 *   0x40 DATA      (RO, dynamic)  the estimate — seqlock-guarded pose block
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
#define TINYVIO_PROTO_VERSION  0x01u   /* bump on ANY breaking layout/semantic chg */

/* ── Block base register offsets ────────────────────────────────────────────── */
#define TINYVIO_REG_IDENTITY   0x00u
#define TINYVIO_REG_STATUS     0x10u
#define TINYVIO_REG_CONTROL    0x20u
#define TINYVIO_REG_TIMESYNC   0x30u
#define TINYVIO_REG_DATA       0x40u

/* ── Capabilities bitmap (identity.capabilities) ────────────────────────────── */
#define TINYVIO_CAP_POSE       (1u << 0) /* position valid                        */
#define TINYVIO_CAP_ATTITUDE   (1u << 1) /* quaternion valid                      */
#define TINYVIO_CAP_VELOCITY   (1u << 2) /* linear velocity valid                 */
/* bit 3 reserved (was covariance) */
#define TINYVIO_CAP_TIMESTAMP  (1u << 4) /* data.timestamp_us valid               */
#define TINYVIO_CAP_TIMESYNC   (1u << 5) /* honours TIMESYNC block                */

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
} tinyvio_error_t;

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
    uint8_t  health;        /* 0x11  TINYVIO_HEALTH_* bitmap                       */
    uint8_t  error_code;    /* 0x12  tinyvio_error_t                               */
    uint8_t  track_quality; /* 0x13  0..255 tracking confidence                    */
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
    uint8_t  quality;       /* +0x34  0..255 tracking confidence                   */
    uint8_t  _reserved1;    /* +0x35                                               */
    uint16_t seq_end;       /* +0x36  == seq_begin when the frame is coherent      */
} tinyvio_data_t;

typedef struct TINYVIO_PACKED {
    tinyvio_identity_t identity; /* 0x00 */
    tinyvio_status_t   status;   /* 0x10 */
    tinyvio_control_t  control;  /* 0x20 */
    tinyvio_timesync_t timesync; /* 0x30 */
    tinyvio_data_t     data;     /* 0x40 */
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
TINYVIO_STATIC_ASSERT(sizeof(tinyvio_regs_t) <= 0x100, "register file exceeds 256B");

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

#if defined(__cplusplus)
}  /* extern "C" */
#endif

#endif /* TINYVIO_DECK_PROTOCOL_H */
