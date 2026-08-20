/**
 * tinyvioDeck.c - Crazyflie deck driver for the TinyVIO deck (RP2354).
 *
 * The TinyVIO deck is an RP2354 companion running visual-inertial odometry. It
 * talks to the Crazyflie STM32 over the deck I2C bus (I2C1_DEV, expansion header
 * SDA=P1.4 / SCL=P1.5). The RP2350 is the I2C *slave* at TINYVIO_I2C_ADDR; this
 * driver is the *master*. It polls the deck's register file (the canonical
 * protocol, tinyvio_deck_protocol.h) and surfaces the nav-state estimate to
 * cfclient — the consumer that will later feed TinyMPC / the CF estimator.
 *
 * The deck has NO OneWire deck-ID EEPROM, so the CF cannot auto-detect it. This
 * driver must be FORCE-LOADED:
 *   make menuconfig -> Expansion deck configuration:
 *       [*] Support the TinyVIO deck          (CONFIG_DECK_TINYVIO=y)
 *       Force load specified custom deck driver = "tinyvio"  (CONFIG_DECK_FORCE)
 *
 * v0x02 protocol (spec: research-vault tiny-cvio/deck-comms-protocol.md):
 *   LOG  `tinyvio`  — pose (px/py/pz), est-FSM state, fault byte, TELEM decode
 *                     (stdPos/stdAtt/bg/ba, fold pair), capture state, download
 *                     progress. ROS selects fields+rate via LogConfig.
 *   PARAM `tinyvio` — est commands (cmd) + capture commands (capCmd/capType/
 *                     capLabel/capOdr/capLpf/capFsr, relayed to CAPTURE_CTRL)
 *                     + capDl (page the READY blob out over the app-channel).
 * Capability-gated: IDENTITY.capabilities decides whether CAPTURE/TELEM blocks
 * are polled, so one driver serves every deck app (pose, vib, future).
 */

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "system.h"
#include "deck.h"
#include "log.h"
#include "param.h"
#include "i2cdev.h"
#include "supervisor.h"              /* arm state -> autonomous capture stop   */

#include "tinyvio_deck_protocol.h"   /* vendored from the tiny-vio repo (v0x02) */
#include "app_channel.h"             /* CF -> host bulk download transport    */

#define DEBUG_MODULE "TINYVIO"
#include "debug.h"

#define TINYVIO_TASK_NAME        "TINYVIO"
/* 4x minimal: the vib download's i2cdev + appchannel call chain needs the headroom
 * (2x hardfaulted on the first 128-B window read). */
#define TINYVIO_TASK_STACKSIZE   (4 * configMINIMAL_STACK_SIZE)
#define TINYVIO_TASK_PRI         3
#define TINYVIO_UPDATE_PERIOD_MS 20   /* 50 Hz — TinyMPC-class consume rate */

static bool isInit = false;
static bool isVerified = false;
static uint8_t deckCaps = 0;      /* IDENTITY.capabilities, cached at init        */

/* Telemetry (LOG) */
static uint8_t  i2cOk = 0;        /* last poll fully ACKed                        */
static uint8_t  state = 0;        /* deck lifecycle (tinyvio_state_t)             */
static uint8_t  fault = 0;        /* Vitals estimator fault byte (0 = healthy)    */
static uint8_t  coherent = 0;     /* last DATA read passed the seqlock            */
static uint8_t  aliveOk = 0;      /* deck alive_counter is advancing              */
static float    px = 0, py = 0, pz = 0;  /* position estimate (m)                 */
/* Deck time-of-validity for px/py/pz, split into halves: LogDataGeneric carries
 * values as float32, whose 24-bit mantissa would quantise a u32 us count to
 * ~256 us. Each u16 is exactly representable, so the host gets exact us back
 * via (hi << 16) | lo. */
static uint16_t tUsHi = 0, tUsLo = 0;
static uint32_t lastAlive = 0;

/* TELEM block decode (LOG floats; only polled when deck has CAP_TELEM) */
static float    stdPosX = 0, stdPosY = 0, stdPosZ = 0;  /* σ position (m)         */
static float    stdAttX = 0, stdAttY = 0, stdAttZ = 0;  /* σ attitude (rad)       */
static float    bgX = 0, bgY = 0, bgZ = 0;              /* gyro bias (rad/s)      */
static float    baX = 0, baY = 0, baZ = 0;              /* accel bias (m/s²)      */
static uint8_t  nFolded = 0, nAccepted = 0;             /* fold starvation pair   */
static uint16_t updateUs = 0;
static uint8_t  clones = 0;

/* Command channel (PARAM): set `cmd` to a TINYVIO_CMD_* to issue it once. */
static uint8_t reqCmd = 0;
static uint8_t lastReqCmd = 0;
static uint8_t cmdSeq = 0;
static uint8_t cmdAck = 0;

/* Capture command channel (PARAM → CAPTURE_CTRL relay; only with CAP_CAPTURE).
 * Host sets capType/capLabel/capOdr/capLpf/capFsr, then writes capCmd (a
 * TINYVIO_CAP_CMD_*) — relayed once with a fresh seq, then auto-cleared. */
static uint8_t  capCmd = 0;       /* param trigger (auto-clears)                  */
static uint8_t  capType = TINYVIO_CAP_TYPE_VIB;
static uint16_t capLabel = 0;
static uint8_t  capOdr = 0;       /* tinyvio_odr_code_t; 0 = keep                 */
static uint8_t  capLpf = 0;       /* UI-LPF divisor; 0 = NO_FILTER                */
static uint8_t  capFsr = 0;       /* accel FSR g; 0 = keep                        */
static uint8_t  capCmdSeq = 0;    /* local seq, mirrors deck's ack               */
/* Capture status mirror (LOG) */
static uint8_t  capState = 0;     /* tinyvio_cap_state_t                          */
static uint8_t  capStatus = 0;    /* tinyvio_cap_cmd_status_t of last command     */
static uint8_t  capAck = 0;       /* deck's cap_cmd_ack_seq                       */
static uint32_t capBlobSz = 0;
static uint16_t capCount = 0;     /* live progress (advisory)                     */

static void tinyvioTask(void *param);
static void probeIdentity(void);


static void tinyvioDeckInit(DeckInfo *info) {
  if (isInit) {
    return;
  }

  probeIdentity();   /* non-fatal on failure: the task retries until verified */

  xTaskCreate(tinyvioTask, TINYVIO_TASK_NAME, TINYVIO_TASK_STACKSIZE,
              NULL, TINYVIO_TASK_PRI, NULL);


  isInit = true;
}

static bool tinyvioDeckTest(void) {
  /* IMPORTANT: return true unconditionally. A failed deck test() prevents
   * systemStart() (system.c) -> canStartMutex is never given -> every
   * systemWaitStart() task (including this one) deadlocks. A companion VIO deck
   * must never gate the CF system self-test; health is reported via LOG. */
  return isInit;
}

/* Probe + verify the deck identity; on success cache capabilities and seed the
 * capture-command seq from the deck's last ack. Called at init AND retried from
 * the task until verified — a lost power-on race must not permanently disable
 * the capture path (review MAJOR-4). Seeding capCmdSeq from cap_cmd_ack_seq
 * prevents seq aliasing after a CF reboot: our first command's seq is then
 * guaranteed != the deck's persisted last seq (review MAJOR-5). */
static void probeIdentity(void) {
  tinyvio_identity_t id;
  memset(&id, 0, sizeof(id));
  bool ok = i2cdevReadReg8(I2C1_DEV, TINYVIO_I2C_ADDR, TINYVIO_REG_IDENTITY,
                           sizeof(id), (uint8_t *)&id);
  isVerified = ok && tinyvio_identity_ok(&id);
  deckCaps = isVerified ? id.capabilities : 0;   /* gates CAPTURE/TELEM polling */
  DEBUG_PRINT("TinyVIO deck: ok=%d magic=0x%04x who=0x%02x proto=%u caps=0x%02x -> %s\n",
              (int)ok, id.magic, id.who_am_i, id.proto_version, id.capabilities,
              isVerified ? "verified" : "unverified (will retry)");
  if (isVerified && (deckCaps & TINYVIO_CAP_CAPTURE)) {
    tinyvio_capture_ctrl_t cc;
    if (i2cdevReadReg8(I2C1_DEV, TINYVIO_I2C_ADDR, TINYVIO_REG_CAPTURE_CTRL,
                       sizeof(cc), (uint8_t *)&cc)) {
      capCmdSeq = cc.cap_cmd_ack_seq;   /* next command = ack+1, never aliases */
    }
  }
}

/* ── Capture blob download (v0x02, generalized over capture types) ────────────
 * Pages the READY blob out of the deck's WINDOW and forwards it to the host on
 * the app-channel. Param `tinyvio.capDl`; blocking in tinyvioTask (post-land or
 * landed-between-tests). Download does NOT change deck state — the host sends
 * capCmd = RELEASE after CRC-verifying. Blobs are self-describing (type header
 * leads the blob), so this code is type-agnostic.
 * ⚠ Every app-channel packet MUST be ≤ 30 B: APPCHANNEL_MTU claims 31 but
 * crtpSendPacketBlock ASSERTs (and reboots the CF) above CRTP_MAX_DATA_SIZE=30.
 * TINYVIO_APP_DATA_PAYLOAD (25) encodes that cap. */
static uint8_t  capDl      = 0;   /* param: write 1 to start a download (auto-clear) */
static uint8_t  capDlDone  = 0;   /* log: 1 = last download completed OK             */
static uint32_t capDlBytes = 0;   /* log: bytes forwarded so far                     */

/* Buffers kept OFF the (small) tinyvioTask stack — a window + packet frame plus
 * the nested i2cdev/appchannel call chain overflowed 2x-minimal stack once
 * (hardfault). Single caller (tinyvioTask), so file-scope static is safe. */
static uint8_t s_capWin[TINYVIO_WINDOW_SIZE];
static uint8_t s_capPkt[30];      /* the CRTP hard cap */

static void capDownloadRun(void) {
  capDlDone = 0;
  capDlBytes = 0;

  /* 1. The capture must already be frozen: cap_state == READY (host sent STOP). */
  tinyvio_capture_stat_t cs;
  memset(&cs, 0, sizeof(cs));
  if (!i2cdevReadReg8(I2C1_DEV, TINYVIO_I2C_ADDR, TINYVIO_REG_CAPTURE_STAT,
                      sizeof(cs), (uint8_t *)&cs) ||
      cs.cap_state != TINYVIO_CAP_READY) {
    DEBUG_PRINT("cap dl: not READY (state=%u)\n", (unsigned)cs.cap_state);
    return;
  }
  if (cs.blob_size == 0u || cs.blob_size > 200000u) {
    DEBUG_PRINT("cap dl: bad blob_size %lu\n", (unsigned long)cs.blob_size);
    return;
  }

  /* 2. Generic 'H': [u8 'H'][u8 type][u16 label][u32 blob_size][u32 crc32]. */
  s_capPkt[0] = TINYVIO_APP_HEADER;
  s_capPkt[1] = cs.cap_type;
  memcpy(s_capPkt + 2,  &cs.cap_label, 2);
  memcpy(s_capPkt + 4,  &cs.blob_size, 4);
  memcpy(s_capPkt + 8,  &cs.crc32,     4);
  appchannelSendDataPacketBlock(s_capPkt, 12);

  /* 3. Page: select a 64-B bank (cap_bank in CAPTURE_CTRL), burst-read WINDOW,
   * forward as 'D' [u8 'D'][u32 offset][<=25 B] chunks, reassembled by offset. */
  uint32_t off = 0;
  while (off < cs.blob_size) {
    uint16_t bank = (uint16_t)(off / TINYVIO_WINDOW_SIZE);
    uint8_t bb[2] = { (uint8_t)(bank & 0xFFu), (uint8_t)(bank >> 8) };
    i2cdevWriteReg8(I2C1_DEV, TINYVIO_I2C_ADDR,
                    TINYVIO_REG_CAPTURE_CTRL + offsetof(tinyvio_capture_ctrl_t, cap_bank),
                    2, bb);

    uint32_t rem = cs.blob_size - off;
    uint8_t wlen = (rem < TINYVIO_WINDOW_SIZE) ? (uint8_t)rem : (uint8_t)TINYVIO_WINDOW_SIZE;
    if (!i2cdevReadReg8(I2C1_DEV, TINYVIO_I2C_ADDR, TINYVIO_REG_WINDOW, wlen, s_capWin)) {
      DEBUG_PRINT("cap dl: window read fail @%lu\n", (unsigned long)off);
      return;
    }
    for (uint32_t w = 0; w < wlen; ) {
      uint8_t n = ((wlen - w) < TINYVIO_APP_DATA_PAYLOAD)
                    ? (uint8_t)(wlen - w) : (uint8_t)TINYVIO_APP_DATA_PAYLOAD;
      uint32_t poff = off + w;
      s_capPkt[0] = TINYVIO_APP_DATA;
      memcpy(s_capPkt + 1, &poff, 4);
      memcpy(s_capPkt + 5, s_capWin + w, n);
      appchannelSendDataPacketBlock(s_capPkt, 5 + n);
      w += n;
    }
    off += wlen;
    capDlBytes = off;
  }

  /* 4. 'E' done marker: [u8 'E'][u32 blob_size][u32 crc32]. */
  s_capPkt[0] = TINYVIO_APP_DONE;
  memcpy(s_capPkt + 1, &cs.blob_size, 4);
  memcpy(s_capPkt + 5, &cs.crc32,     4);
  appchannelSendDataPacketBlock(s_capPkt, 9);
  capDlDone = 1;
  DEBUG_PRINT("cap dl: type %u, %lu B forwarded (crc 0x%08lx)\n",
              (unsigned)cs.cap_type, (unsigned long)cs.blob_size,
              (unsigned long)cs.crc32);
}

static void tinyvioTask(void *param) {
  systemWaitStart();

  TickType_t lastWake = xTaskGetTickCount();
  uint32_t cycle = 0;
  while (1) {
    cycle++;
    /* Identity retry: a lost power-on race must not leave the deck unverified
     * (and the capture path dead) until a CF reboot. Re-probe at 1 Hz. */
    if (!isVerified && (cycle % 50u) == 0u) {
      probeIdentity();
    }

    /* Autonomous capture stop on the armed -> disarmed edge (i.e. landed). The deck cannot
     * detect this itself, and this path does not need the radio: a link loss makes the
     * supervisor land and disarm, which lands here and freezes the blob. Seeded from the
     * live arm state so a driver init while already armed does not fire a spurious stop. */
    {
      static bool prevArmed = false;
      static bool armSeeded = false;
      const bool armedNow = supervisorIsArmed();
      if (!armSeeded) { prevArmed = armedNow; armSeeded = true; }
      if (prevArmed && !armedNow && capState == TINYVIO_CAP_CAPTURING) {
        capCmd = TINYVIO_CAP_CMD_STOP;   /* relayed below, this same pass */
      }
      prevArmed = armedNow;
    }

    /* Capture blob download (blocking, landed). Runs before the poll cycle. */
    if (capDl) {
      capDl = 0;
      if (deckCaps & TINYVIO_CAP_CAPTURE) capDownloadRun();
      lastWake = xTaskGetTickCount();   /* resync after the long blocking run */
    }

    bool ok = true;

    /* (1) Status: lifecycle + liveness + estimator fault byte. */
    tinyvio_status_t st;
    ok &= i2cdevReadReg8(I2C1_DEV, TINYVIO_I2C_ADDR, TINYVIO_REG_STATUS,
                         sizeof(st), (uint8_t *)&st);
    if (ok) {
      state    = st.state;
      fault    = st.fault;
      aliveOk  = (st.alive_counter != lastAlive) ? 1 : 0;   /* stalled deck -> 0 */
      lastAlive = st.alive_counter;
    }

    /* (2) DATA: seqlock-guarded pose. One burst read; retry once if torn. */
    tinyvio_data_t d;
    uint8_t got_coherent = 0;
    for (int attempt = 0; attempt < 2 && !got_coherent; attempt++) {
      ok &= i2cdevReadReg8(I2C1_DEV, TINYVIO_I2C_ADDR, TINYVIO_REG_DATA,
                           sizeof(d), (uint8_t *)&d);
      got_coherent = tinyvio_data_is_coherent(&d) ? 1 : 0;
    }
    coherent = got_coherent;

    /* Consume the estimate only when coherent AND the deck says it's usable.
     * (This is where a later revision hands (p,v,q) to the estimator/TinyMPC.) */
    if (got_coherent && tinyvio_pose_usable(&st)) {
      px = d.pos[0]; py = d.pos[1]; pz = d.pos[2];
      /* The deck's own time-of-validity, published alongside the pose it belongs
       * to. Without it the only timestamps downstream are the STM32 log clock
       * (when WE sampled, up to TINYVIO_UPDATE_PERIOD_MS late) and the ROS
       * receive time (later still, and jittery) — neither says when the deck
       * believed this pose was true. Assigned inside the coherence gate so the
       * stamp cannot be paired with a pose from a different frame.
       * Low 32 bits only: the wrap is ~71.6 min. */
      const uint32_t t32 = (uint32_t)d.timestamp_us;
      tUsLo = (uint16_t)(t32 & 0xFFFFu);
      tUsHi = (uint16_t)(t32 >> 16);
    }

    /* (3) Estimator command channel: issue once, read back the ack. */
    if (reqCmd != lastReqCmd) {
      cmdSeq++;
      uint8_t cbuf[2] = { reqCmd, cmdSeq };   /* control.cmd, control.cmd_seq */
      ok &= i2cdevWriteReg8(I2C1_DEV, TINYVIO_I2C_ADDR, TINYVIO_REG_CONTROL, 2, cbuf);
      lastReqCmd = reqCmd;
    }
    ok &= i2cdevReadByte(I2C1_DEV, TINYVIO_I2C_ADDR,
                         TINYVIO_REG_CONTROL + offsetof(tinyvio_control_t, cmd_ack_seq),
                         &cmdAck);

    /* (4) Capture command relay + status mirror (only with CAP_CAPTURE). */
    if (deckCaps & TINYVIO_CAP_CAPTURE) {
      if (capCmd != TINYVIO_CAP_CMD_NONE) {
        /* Two-write order: config bytes FIRST (odr/lpf/fsr @ +0x0A..0x0C), then
         * cmd/type/label/seq (@ +0x00..0x04) as one burst ENDING with the fresh
         * seq — the deck processes on seq change, so the command becomes visible
         * only after its config is already in place. */
        capCmdSeq++;
        uint8_t cfg[3] = { capOdr, capLpf, capFsr };
        ok &= i2cdevWriteReg8(I2C1_DEV, TINYVIO_I2C_ADDR,
                              TINYVIO_REG_CAPTURE_CTRL +
                              offsetof(tinyvio_capture_ctrl_t, cap_odr_code),
                              3, cfg);
        uint8_t cmdb[5];
        cmdb[0] = capCmd;
        cmdb[1] = capType;
        memcpy(cmdb + 2, &capLabel, 2);
        cmdb[4] = capCmdSeq;
        ok &= i2cdevWriteReg8(I2C1_DEV, TINYVIO_I2C_ADDR,
                              TINYVIO_REG_CAPTURE_CTRL, 5, cmdb);
        capCmd = 0;   /* auto-clearing trigger (vibtest.startTest idiom) */
      }
      /* Mirror ack/status + capture state for LOG (each 20 ms cycle). */
      tinyvio_capture_ctrl_t cc;
      if (i2cdevReadReg8(I2C1_DEV, TINYVIO_I2C_ADDR, TINYVIO_REG_CAPTURE_CTRL,
                         sizeof(cc), (uint8_t *)&cc)) {
        capAck    = cc.cap_cmd_ack_seq;
        capStatus = cc.cap_cmd_status;
      }
      tinyvio_capture_stat_t cs;
      if (i2cdevReadReg8(I2C1_DEV, TINYVIO_I2C_ADDR, TINYVIO_REG_CAPTURE_STAT,
                         sizeof(cs), (uint8_t *)&cs)) {
        capState  = cs.cap_state;
        capBlobSz = cs.blob_size;
        capCount  = cs.cap_count;
      }
    }

    /* (5) TELEM decode → LOG floats (only with CAP_TELEM; advisory block). */
    if (deckCaps & TINYVIO_CAP_TELEM) {
      tinyvio_telem_t tm;
      if (i2cdevReadReg8(I2C1_DEV, TINYVIO_I2C_ADDR, TINYVIO_REG_TELEM,
                         sizeof(tm), (uint8_t *)&tm)) {
        stdPosX = tinyvio_telem_dec(tm.std_pos[0], TINYVIO_TELEM_STDPOS_LSB_M);
        stdPosY = tinyvio_telem_dec(tm.std_pos[1], TINYVIO_TELEM_STDPOS_LSB_M);
        stdPosZ = tinyvio_telem_dec(tm.std_pos[2], TINYVIO_TELEM_STDPOS_LSB_M);
        stdAttX = tinyvio_telem_dec(tm.std_att[0], TINYVIO_TELEM_STDATT_LSB_RAD);
        stdAttY = tinyvio_telem_dec(tm.std_att[1], TINYVIO_TELEM_STDATT_LSB_RAD);
        stdAttZ = tinyvio_telem_dec(tm.std_att[2], TINYVIO_TELEM_STDATT_LSB_RAD);
        bgX = tinyvio_telem_dec(tm.bg[0], TINYVIO_TELEM_BG_LSB_RADS);
        bgY = tinyvio_telem_dec(tm.bg[1], TINYVIO_TELEM_BG_LSB_RADS);
        bgZ = tinyvio_telem_dec(tm.bg[2], TINYVIO_TELEM_BG_LSB_RADS);
        baX = tinyvio_telem_dec(tm.ba[0], TINYVIO_TELEM_BA_LSB_MS2);
        baY = tinyvio_telem_dec(tm.ba[1], TINYVIO_TELEM_BA_LSB_MS2);
        baZ = tinyvio_telem_dec(tm.ba[2], TINYVIO_TELEM_BA_LSB_MS2);
        nFolded   = tm.n_folded;
        nAccepted = tm.n_accepted;
        updateUs  = tm.update_us;
        clones    = tm.clones;
      }
    }

    i2cOk = ok ? 1 : 0;
    vTaskDelayUntil(&lastWake, M2T(TINYVIO_UPDATE_PERIOD_MS));
  }
}

static const DeckDriver tinyvio_deck = {
  .vid = 0,   /* force-loaded by name; no OW deck ID on this board */
  .pid = 0,
  .name = "tinyvio",

  .usedPeriph = DECK_USING_I2C,

  .init = tinyvioDeckInit,
  .test = tinyvioDeckTest,
};

DECK_DRIVER(tinyvio_deck);

LOG_GROUP_START(tinyvio)
/** @brief Nonzero if the last poll to the deck fully ACKed */
LOG_ADD(LOG_UINT8, i2cOk, &i2cOk)
/** @brief Deck lifecycle state (tinyvio_state_t) */
LOG_ADD(LOG_UINT8, state, &state)
/** @brief Nonzero if the last DATA read passed the seqlock (coherent frame) */
LOG_ADD(LOG_UINT8, coherent, &coherent)
/** @brief Nonzero if the deck's alive_counter is advancing */
LOG_ADD(LOG_UINT8, aliveOk, &aliveOk)
/** @brief Vitals estimator fault byte (0 = healthy; bits per vitals.h) */
LOG_ADD(LOG_UINT8, fault, &fault)
/** @brief Position estimate x (m, deck odometry frame) */
LOG_ADD(LOG_FLOAT, px, &px)
/** @brief Position estimate y (m, deck odometry frame) */
LOG_ADD(LOG_FLOAT, py, &py)
/** @brief Position estimate z (m, deck odometry frame) */
LOG_ADD(LOG_FLOAT, pz, &pz)
/** @brief Deck time-of-validity, high half. Reassemble host-side:
 *  t_us = (tUsHi << 16) | tUsLo   (deck us, low 32 bits; wraps ~71.6 min).
 *  Log both halves in the SAME block as px/py/pz — that pairs each pose with the
 *  instant the deck says it was true, not the instant the STM32 sampled it. */
LOG_ADD(LOG_UINT16, tUsHi, &tUsHi)
/** @brief Deck time-of-validity, low half. See tUsHi. */
LOG_ADD(LOG_UINT16, tUsLo, &tUsLo)
/** @brief Deck's echoed estimator command-ack sequence */
LOG_ADD(LOG_UINT8, cmdAck, &cmdAck)
/** @brief Capture FSM state (0 idle / 1 capturing / 2 ready) */
LOG_ADD(LOG_UINT8, capState, &capState)
/** @brief Result of the last capture command (tinyvio_cap_cmd_status_t) */
LOG_ADD(LOG_UINT8, capStatus, &capStatus)
/** @brief Deck's echoed capture command-ack sequence */
LOG_ADD(LOG_UINT8, capAck, &capAck)
/** @brief Size of the READY capture blob (bytes) */
LOG_ADD(LOG_UINT32, capBlobSz, &capBlobSz)
/** @brief Live capture progress (e.g. Welch windows; advisory) */
LOG_ADD(LOG_UINT16, capCount, &capCount)
/** @brief Nonzero once the last capture download completed */
LOG_ADD(LOG_UINT8, capDlDone, &capDlDone)
/** @brief Bytes forwarded so far in the current/last capture download */
LOG_ADD(LOG_UINT32, capDlBytes, &capDlBytes)
/** @brief Position std-dev x/y/z (m, from the deck TELEM block) */
LOG_ADD(LOG_FLOAT, stdPosX, &stdPosX)
LOG_ADD(LOG_FLOAT, stdPosY, &stdPosY)
LOG_ADD(LOG_FLOAT, stdPosZ, &stdPosZ)
/** @brief Attitude std-dev x/y/z (rad) */
LOG_ADD(LOG_FLOAT, stdAttX, &stdAttX)
LOG_ADD(LOG_FLOAT, stdAttY, &stdAttY)
LOG_ADD(LOG_FLOAT, stdAttZ, &stdAttZ)
/** @brief Gyro bias estimate x/y/z (rad/s) */
LOG_ADD(LOG_FLOAT, bgX, &bgX)
LOG_ADD(LOG_FLOAT, bgY, &bgY)
LOG_ADD(LOG_FLOAT, bgZ, &bgZ)
/** @brief Accel bias estimate x/y/z (m/s^2) */
LOG_ADD(LOG_FLOAT, baX, &baX)
LOG_ADD(LOG_FLOAT, baY, &baY)
LOG_ADD(LOG_FLOAT, baZ, &baZ)
/** @brief Board folds selected this frame */
LOG_ADD(LOG_UINT8, nFolded, &nFolded)
/** @brief Board folds accepted this frame (starvation pair with nFolded) */
LOG_ADD(LOG_UINT8, nAccepted, &nAccepted)
/** @brief EKF update latency (us) */
LOG_ADD(LOG_UINT16, updateUs, &updateUs)
/** @brief Live clone count */
LOG_ADD(LOG_UINT8, clones, &clones)
LOG_GROUP_STOP(tinyvio)

PARAM_GROUP_START(tinyvio)
/** @brief Write a TINYVIO_CMD_* value to issue that command to the deck once */
PARAM_ADD(PARAM_UINT8, cmd, &reqCmd)
/** @brief Capture command: write a TINYVIO_CAP_CMD_* to relay it once (auto-clears) */
PARAM_ADD(PARAM_UINT8, capCmd, &capCmd)
/** @brief Capture type for START (tinyvio_cap_type_t; 1 = VIB) */
PARAM_ADD(PARAM_UINT8, capType, &capType)
/** @brief Test label baked into the capture (u16) */
PARAM_ADD(PARAM_UINT16, capLabel, &capLabel)
/** @brief IMU ODR code for START (tinyvio_odr_code_t; 0 = keep) */
PARAM_ADD(PARAM_UINT8, capOdr, &capOdr)
/** @brief IMU UI-LPF divisor for START (0 = NO_FILTER, 4/8/16/32/64/128) */
PARAM_ADD(PARAM_UINT8, capLpf, &capLpf)
/** @brief Accel FSR (g) for START (0 = keep, 2/4/8/16/32) */
PARAM_ADD(PARAM_UINT8, capFsr, &capFsr)
/** @brief Write 1 to page the READY capture blob to the host (app-channel) */
PARAM_ADD(PARAM_UINT8, capDl, &capDl)
PARAM_GROUP_STOP(tinyvio)


PARAM_GROUP_START(deck)
/** @brief Nonzero if the TinyVIO deck driver is initialized (force-loaded) */
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcTinyVIO, &isInit)
PARAM_GROUP_STOP(deck)
