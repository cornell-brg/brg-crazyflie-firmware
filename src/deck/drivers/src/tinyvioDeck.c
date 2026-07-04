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
 * cfclient: log group `tinyvio` (i2cOk, state, px/py/pz, quality, alive, coherent);
 * param group `tinyvio` (cmd -> issues a TINYVIO_CMD_* to the deck).
 */

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "system.h"
#include "deck.h"
#include "log.h"
#include "param.h"
#include "i2cdev.h"

#include "tinyvio_deck_protocol.h"   /* vendored from the tiny-vio repo */

#define DEBUG_MODULE "TINYVIO"
#include "debug.h"

#define TINYVIO_TASK_NAME        "TINYVIO"
#define TINYVIO_TASK_STACKSIZE   (2 * configMINIMAL_STACK_SIZE)
#define TINYVIO_TASK_PRI         3
#define TINYVIO_UPDATE_PERIOD_MS 20   /* 50 Hz — TinyMPC-class consume rate */

static bool isInit = false;
static bool isVerified = false;

/* Telemetry (LOG) */
static uint8_t  i2cOk = 0;        /* last poll fully ACKed                        */
static uint8_t  state = 0;        /* deck lifecycle (tinyvio_state_t)             */
static uint8_t  quality = 0;      /* tracking confidence 0..255                   */
static uint8_t  coherent = 0;     /* last DATA read passed the seqlock            */
static uint8_t  aliveOk = 0;      /* deck alive_counter is advancing              */
static float    px = 0, py = 0, pz = 0;  /* position estimate (m)                 */
static uint32_t lastAlive = 0;

/* Command channel (PARAM): set `cmd` to a TINYVIO_CMD_* to issue it once. */
static uint8_t reqCmd = 0;
static uint8_t lastReqCmd = 0;
static uint8_t cmdSeq = 0;
static uint8_t cmdAck = 0;

static void tinyvioTask(void *param);

static void tinyvioDeckInit(DeckInfo *info) {
  if (isInit) {
    return;
  }

  tinyvio_identity_t id;
  memset(&id, 0, sizeof(id));
  bool ok = i2cdevReadReg8(I2C1_DEV, TINYVIO_I2C_ADDR, TINYVIO_REG_IDENTITY,
                           sizeof(id), (uint8_t *)&id);
  isVerified = ok && tinyvio_identity_ok(&id);
  DEBUG_PRINT("TinyVIO deck: ok=%d magic=0x%04x who=0x%02x proto=%u caps=0x%02x -> %s\n",
              (int)ok, id.magic, id.who_am_i, id.proto_version, id.capabilities,
              isVerified ? "verified" : "unverified (non-fatal, task keeps polling)");

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

static void tinyvioTask(void *param) {
  systemWaitStart();

  TickType_t lastWake = xTaskGetTickCount();
  while (1) {
    bool ok = true;

    /* (1) Status: lifecycle + liveness. */
    tinyvio_status_t st;
    ok &= i2cdevReadReg8(I2C1_DEV, TINYVIO_I2C_ADDR, TINYVIO_REG_STATUS,
                         sizeof(st), (uint8_t *)&st);
    if (ok) {
      state    = st.state;
      quality  = st.track_quality;
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
    }

    /* (3) Command channel: issue a new command once, then read back the ack. */
    if (reqCmd != lastReqCmd) {
      cmdSeq++;
      uint8_t cbuf[2] = { reqCmd, cmdSeq };   /* control.cmd, control.cmd_seq */
      ok &= i2cdevWriteReg8(I2C1_DEV, TINYVIO_I2C_ADDR, TINYVIO_REG_CONTROL, 2, cbuf);
      lastReqCmd = reqCmd;
    }
    ok &= i2cdevReadByte(I2C1_DEV, TINYVIO_I2C_ADDR,
                         TINYVIO_REG_CONTROL + offsetof(tinyvio_control_t, cmd_ack_seq),
                         &cmdAck);

    i2cOk = ok ? 1 : 0;
    vTaskDelayUntil(&lastWake, M2T(TINYVIO_UPDATE_PERIOD_MS));
  }
}

static const DeckDriver tinyvio_deck = {
  .vid = 0,   /* force-loaded by name; no OW deck ID on this board */
  .pid = 0,
  .name = "tinyvio",

  .usedPeriph = DECK_USING_I2C,
  .usedGpio = 0,

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
/** @brief Tracking confidence 0..255 */
LOG_ADD(LOG_UINT8, quality, &quality)
/** @brief Position estimate x (m, deck odometry frame) */
LOG_ADD(LOG_FLOAT, px, &px)
/** @brief Position estimate y (m, deck odometry frame) */
LOG_ADD(LOG_FLOAT, py, &py)
/** @brief Position estimate z (m, deck odometry frame) */
LOG_ADD(LOG_FLOAT, pz, &pz)
/** @brief Deck's echoed command-ack sequence */
LOG_ADD(LOG_UINT8, cmdAck, &cmdAck)
LOG_GROUP_STOP(tinyvio)

PARAM_GROUP_START(tinyvio)
/** @brief Write a TINYVIO_CMD_* value to issue that command to the deck once */
PARAM_ADD(PARAM_UINT8, cmd, &reqCmd)
PARAM_GROUP_STOP(tinyvio)

PARAM_GROUP_START(deck)
/** @brief Nonzero if the TinyVIO deck driver is initialized (force-loaded) */
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcTinyVIO, &isInit)
PARAM_GROUP_STOP(deck)
