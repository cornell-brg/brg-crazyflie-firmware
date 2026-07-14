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

#ifdef CONFIG_DECK_TINYVIO_VIBTEST
/* ───────────────────────── Bench vibration-test sequencer ─────────────────
 * Autonomous throttle-sweep state machine for characterising motor/prop
 * vibration on the deck IMU. Drives all 4 motors through a stepped throttle
 * sweep via the motorPowerSet override (bypasses arming — the same path the MSP
 * motor test uses, msp.c) while pulsing a GPIO wired CF->deck to trigger + mark
 * the deck's IMU capture. BENCH ONLY: the drone MUST be rigidly clamped, props
 * on, area clear. Launch from cfclient: `param set vibtest.startTest 1`.
 *
 * Timeline (one deck capture window): preroll (motors off = baseline) -> nSteps
 * throttle setpoints (dwell each) -> postroll (motors off). A rising edge on the
 * trigger arms the deck capture (first edge) and marks each setpoint boundary,
 * so the deck records where every setpoint starts. The total MUST fit the deck's
 * capture window (default 8 s): preroll + nSteps*dwell + postroll <= window.
 */
#define VIBTEST_TRIG_PIN  DECK_GPIO_IO1   /* CF -> deck GP4 (CF_IO_1); pulse-marker mode + grid "go" line */
#define VIBTEST_DONE_PIN  DECK_GPIO_IO2   /* deck -> CF GP5 (CF_IO_2); grid "done" line (deck-imu-grid handshake) */
#define VIBTEST_NMOTORS   4

typedef enum { VIB_IDLE = 0, VIB_PREROLL, VIB_STEP, VIB_POSTROLL } vibState_t;

/* Params (vibtest group) — safe defaults; total = 500 + 5*1200 + 500 = 7 s < 8 s window. */
static uint8_t  vibStart   = 0;      /* write 1 to launch (auto-cleared) */
static uint8_t  vibAbort   = 0;      /* write 1 to cut motors + stop      */
static uint16_t vibThrMin  = 5000;   /* first setpoint PWM (0..65535)     */
static uint16_t vibThrMax  = 25000;  /* last setpoint PWM                 */
static uint8_t  vibNSteps  = 5;      /* number of setpoints               */
static uint16_t vibDwellMs = 1200;   /* dwell per setpoint                */
static uint16_t vibPreMs   = 500;    /* preroll, motors off (baseline)    */
static uint16_t vibPostMs  = 500;    /* postroll, motors off              */
static uint16_t vibThrCap  = 30000;  /* HARD safety clamp on any setpoint */
static uint8_t  vibGrid    = 0;      /* 0 = pulse-marker sweep (deck-imu-noise);
                                        1 = 2-wire handshake grid (deck-imu-grid) */
static uint16_t vibSettleMs = 700;   /* RPM settle before "go" (grid mode)         */

/* LOG */
static uint8_t  vibStateLog = VIB_IDLE;
static uint16_t vibThrLog   = 0;     /* current commanded throttle */
static uint8_t  vibStepLog  = 0;

/* motorPowerSet override handles (fetched once the param system is up). */
static paramVarId_t vibEnParam;
static paramVarId_t vibMotParam[VIBTEST_NMOTORS];
static bool         vibParamsReady = false;

/* SM working state */
static vibState_t vibSt = VIB_IDLE;
static TickType_t vibPhaseStart = 0;
static uint8_t    vibStep = 0;

static void vibMotorsOverride(uint16_t pwm) {
  if (!vibParamsReady) return;
  paramSetInt(vibEnParam, 1);   /* per-motor override; persists vs the stabilizer */
  for (int m = 0; m < VIBTEST_NMOTORS; m++) paramSetInt(vibMotParam[m], pwm);
  vibThrLog = pwm;
}
static void vibMotorsRelease(void) {
  if (!vibParamsReady) return;
  for (int m = 0; m < VIBTEST_NMOTORS; m++) paramSetInt(vibMotParam[m], 0);
  paramSetInt(vibEnParam, 0);
  vibThrLog = 0;
}
static void vibTrigPulse(void) {  /* rising edge -> deck arms (1st) + logs a marker */
  digitalWrite(VIBTEST_TRIG_PIN, LOW);
  vTaskDelay(M2T(2));
  digitalWrite(VIBTEST_TRIG_PIN, HIGH);
}
static uint16_t vibLevel(uint8_t k) {
  uint16_t lvl = (vibNSteps <= 1)
      ? vibThrMin
      : vibThrMin + (uint16_t)((uint32_t)(vibThrMax - vibThrMin) * k / (vibNSteps - 1));
  return lvl > vibThrCap ? vibThrCap : lvl;
}

static void vibtestSetup(void) {
  vibEnParam     = paramGetVarId("motorPowerSet", "enable");
  vibMotParam[0] = paramGetVarId("motorPowerSet", "m1");
  vibMotParam[1] = paramGetVarId("motorPowerSet", "m2");
  vibMotParam[2] = paramGetVarId("motorPowerSet", "m3");
  vibMotParam[3] = paramGetVarId("motorPowerSet", "m4");
  vibParamsReady = PARAM_VARID_IS_VALID(vibEnParam) && PARAM_VARID_IS_VALID(vibMotParam[0]) &&
                   PARAM_VARID_IS_VALID(vibMotParam[1]) && PARAM_VARID_IS_VALID(vibMotParam[2]) &&
                   PARAM_VARID_IS_VALID(vibMotParam[3]);
  pinMode(VIBTEST_TRIG_PIN, OUTPUT);
  digitalWrite(VIBTEST_TRIG_PIN, LOW);
  /* Pull DOWN: a floating/undriven "done" line must read LOW, or the CF reads a
   * spurious high and blows through the handshake without ever waiting. */
  pinMode(VIBTEST_DONE_PIN, INPUT_PULLDOWN);
}

/* Grid mode: 2-wire level handshake with deck-imu-grid. For each throttle:
 * set + settle -> raise "go" (IO_1) -> block until the deck raises "done"
 * (IO_2, held) -> drop "go" (ack) -> wait deck clears "done" -> next. The deck
 * cycles all IMU configs between go and done, so the CF never guesses timing.
 * Runs blocking in the vibtest task (dedicated). Bench-only; motors spin. */
static void vibtestGridSweep(void) {
  if (!vibParamsReady) { DEBUG_PRINT("vibtest grid: params unavailable\n"); return; }
  DEBUG_PRINT("vibtest GRID: %u RPMs %u..%u, settle %ums\n",
              vibNSteps, vibThrMin, vibThrMax, vibSettleMs);
  digitalWrite(VIBTEST_TRIG_PIN, LOW);                 /* "go" idle low */
  for (uint8_t i = 0; i < vibNSteps; i++) {
    if (vibAbort) break;
    vibMotorsOverride(vibLevel(i));
    vibStepLog = i;
    vTaskDelay(M2T(vibSettleMs));                      /* let RPM stabilise */
    digitalWrite(VIBTEST_TRIG_PIN, HIGH);              /* "go" */
    /* Wait "done" with a timeout (deck config cycle ~16s). If it never comes,
     * the deck fw/wiring is wrong -> release motors instead of spinning forever. */
    TickType_t t0 = xTaskGetTickCount();
    while (!digitalRead(VIBTEST_DONE_PIN) && !vibAbort) {
      if (xTaskGetTickCount() - t0 > M2T(30000)) {
        DEBUG_PRINT("vibtest grid: TIMEOUT waiting deck 'done' (IO_2) — check deck-imu-grid fw + IO_2/GP5 wiring\n");
        vibAbort = 1; break;
      }
      vTaskDelay(M2T(5));
    }
    digitalWrite(VIBTEST_TRIG_PIN, LOW);               /* ack -> deck advances */
    while (digitalRead(VIBTEST_DONE_PIN) && !vibAbort)  vTaskDelay(M2T(5));  /* wait deck clear */
  }
  vibMotorsRelease();
  digitalWrite(VIBTEST_TRIG_PIN, LOW);
  DEBUG_PRINT("vibtest GRID: done -> motors off\n");
}

static void vibtestTick(void) {
  const TickType_t now = xTaskGetTickCount();

  /* Abort from any active state -> motors off immediately. */
  if (vibAbort && vibSt != VIB_IDLE) {
    vibMotorsRelease();
    vibSt = VIB_IDLE; vibAbort = 0; vibStateLog = vibSt;
    DEBUG_PRINT("vibtest: ABORT -> motors off\n");
    return;
  }

  switch (vibSt) {
    case VIB_IDLE:
      if (vibStart) {
        vibStart = 0;
        if (!vibParamsReady) { DEBUG_PRINT("vibtest: motorPowerSet params unavailable\n"); break; }
        vibMotorsOverride(0);   /* enable override, motors at 0 for the preroll */
        vibTrigPulse();         /* arm the deck capture (marker 0 = window start) */
        vibStep = 0; vibPhaseStart = now; vibSt = VIB_PREROLL;
        DEBUG_PRINT("vibtest: START %u steps %u..%u, dwell %ums (clamp %u)\n",
                    vibNSteps, vibThrMin, vibThrMax, vibDwellMs, vibThrCap);
      }
      break;

    case VIB_PREROLL:
      if (now - vibPhaseStart >= M2T(vibPreMs)) {
        vibTrigPulse();                 /* mark setpoint 0 */
        vibMotorsOverride(vibLevel(0));
        vibStep = 0; vibPhaseStart = now; vibSt = VIB_STEP;
      }
      break;

    case VIB_STEP:
      if (now - vibPhaseStart >= M2T(vibDwellMs)) {
        vibStep++;
        if (vibStep >= vibNSteps) {
          vibMotorsOverride(0);         /* postroll: motors off */
          vibTrigPulse();
          vibPhaseStart = now; vibSt = VIB_POSTROLL;
        } else {
          vibTrigPulse();               /* mark next setpoint */
          vibMotorsOverride(vibLevel(vibStep));
          vibPhaseStart = now;
        }
      }
      break;

    case VIB_POSTROLL:
      if (now - vibPhaseStart >= M2T(vibPostMs)) {
        vibMotorsRelease();
        vibSt = VIB_IDLE;
        DEBUG_PRINT("vibtest: DONE -> ramp is off; dump the deck ring over SWD\n");
      }
      break;
  }
  vibStateLog = vibSt; vibStepLog = vibStep;
}

/* Dedicated task: ticks the sequencer at 50 Hz. Kept separate from the VIO
 * consumer task so it only exists in bench builds and stays decoupled. */
#define VIBTEST_TASK_NAME       "TINYVIO_VIB"
#define VIBTEST_TASK_STACKSIZE  (2 * configMINIMAL_STACK_SIZE)
#define VIBTEST_TASK_PRI        2
#define VIBTEST_PERIOD_MS       20

static void vibtestTaskFn(void *param) {
  systemWaitStart();
  vibtestSetup();
  TickType_t lastWake = xTaskGetTickCount();
  while (1) {
    if (vibStart && vibGrid) {           /* grid handshake: blocking sweep */
      vibStart = 0;
      vibtestGridSweep();
      lastWake = xTaskGetTickCount();    /* resync after the long blocking run */
    } else {
      vibtestTick();                     /* pulse-marker tick SM (grid off) */
    }
    vTaskDelayUntil(&lastWake, M2T(VIBTEST_PERIOD_MS));
  }
}
#endif  /* CONFIG_DECK_TINYVIO_VIBTEST */

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

#ifdef CONFIG_DECK_TINYVIO_VIBTEST
  xTaskCreate(vibtestTaskFn, VIBTEST_TASK_NAME, VIBTEST_TASK_STACKSIZE,
              NULL, VIBTEST_TASK_PRI, NULL);
#endif

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
#ifdef CONFIG_DECK_TINYVIO_VIBTEST
  .usedGpio = DECK_USING_IO_1 | DECK_USING_IO_2,   /* IO_1 "go"/trigger, IO_2 "done" */
#else
  .usedGpio = 0,
#endif

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

#ifdef CONFIG_DECK_TINYVIO_VIBTEST
/* Bench motor-vibration sweep. DRONE MUST BE RIGIDLY CLAMPED, props on. */
LOG_GROUP_START(vibtest)
/** @brief Sequencer state: 0=idle 1=preroll 2=step 3=postroll */
LOG_ADD(LOG_UINT8, state, &vibStateLog)
/** @brief Current commanded throttle PWM (0..65535) */
LOG_ADD(LOG_UINT16, throttle, &vibThrLog)
/** @brief Current setpoint index */
LOG_ADD(LOG_UINT8, step, &vibStepLog)
LOG_GROUP_STOP(vibtest)

PARAM_GROUP_START(vibtest)
/** @brief Write 1 to launch the throttle sweep (auto-cleared). CLAMPED DRONE ONLY. */
PARAM_ADD(PARAM_UINT8, startTest, &vibStart)
/** @brief Write 1 to abort the sweep and cut motors */
PARAM_ADD(PARAM_UINT8, abort, &vibAbort)
/** @brief First setpoint PWM (0..65535) */
PARAM_ADD(PARAM_UINT16, throttleMin, &vibThrMin)
/** @brief Last setpoint PWM (0..65535) */
PARAM_ADD(PARAM_UINT16, throttleMax, &vibThrMax)
/** @brief Number of throttle setpoints in the sweep */
PARAM_ADD(PARAM_UINT8, nSteps, &vibNSteps)
/** @brief Dwell per setpoint (ms) */
PARAM_ADD(PARAM_UINT16, dwellMs, &vibDwellMs)
/** @brief Preroll with motors off (ms) — baseline + spin-up margin */
PARAM_ADD(PARAM_UINT16, prerollMs, &vibPreMs)
/** @brief Postroll with motors off (ms) — spin-down margin */
PARAM_ADD(PARAM_UINT16, postrollMs, &vibPostMs)
/** @brief HARD safety clamp applied to every setpoint PWM */
PARAM_ADD(PARAM_UINT16, throttleCap, &vibThrCap)
/** @brief 0 = pulse-marker sweep (deck-imu-noise); 1 = 2-wire handshake grid (deck-imu-grid) */
PARAM_ADD(PARAM_UINT8, grid, &vibGrid)
/** @brief Grid mode: RPM settle time before "go" (ms) */
PARAM_ADD(PARAM_UINT16, settleMs, &vibSettleMs)
PARAM_GROUP_STOP(vibtest)
#endif  /* CONFIG_DECK_TINYVIO_VIBTEST */

PARAM_GROUP_START(deck)
/** @brief Nonzero if the TinyVIO deck driver is initialized (force-loaded) */
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcTinyVIO, &isInit)
PARAM_GROUP_STOP(deck)
