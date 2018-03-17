#include <Adafruit_SleepyDog.h>
#include <SPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <lmic.h>
#include <hal/hal.h>

#define ONE_WIRE_BUS 20
#define VBAT_PIN A9
#define DIO1_PIN 1

//#define DEBUG 1

#ifdef DEBUG
#define REJOIN_WAIT_MS 30000
#define JOIN_RETRY_MS 30000
#define SLEEP_TIME_MS 10000
#define _sleep(ms) delay(ms)
#define DBG(msg) Serial.println((msg))
#else
#define REJOIN_WAIT_MS 3600000
#define QUICK_JOIN_RETRY_MS 3600000
#define JOIN_RETRY_MS 28800000 /* 8 hours */
#define SLEEP_TIME_MS 3600000
#define _sleep(ms) watchdog_sleep(ms)
#define DBG(msg)
#endif

OneWire one_wire(ONE_WIRE_BUS);
DallasTemperature temp_sensor(&one_wire);

/* AppEUI in little endian */
static const u1_t PROGMEM APPEUI[8] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8); }

/* DevEUI in little endian */
static const u1_t PROGMEM DEVEUI[8] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
void os_getDevEui (u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }

/* AppKey */
static const u1_t PROGMEM APPKEY[16] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16); }

static osjob_t sendjob;

/* Pin mapping */
const struct lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = { 7, DIO1_PIN, LMIC_UNUSED_PIN },
};

/*
 * Watchdog can only sleep for 8 seconds so a loop is needed for longer
 * durations. This will generate some overhead so the sleep times might
 * need to be adjusted if you want more accuracy.
 */
void watchdog_sleep(uint32_t ms)
{
    int t;

    while (ms > 0) {
        if (ms > 8000) {
            t = 8000;
        } else {
            t = ms;
        }

        ms -= t;
        Watchdog.sleep(t);
    }
}

void onEvent(ev_t ev)
{
    DBG(String(os_getTime()) + ": ");

    switch(ev) {
        case EV_JOINING:
            DBG(F("EV_JOINING"));
            break;
        case EV_JOINED:
            DBG(F("EV_JOINED"));
            os_setCallback(&sendjob, do_send);
            break;
        case EV_JOIN_FAILED:
            DBG(F("EV_JOIN_FAILED"));
            reset_lmic();
            _sleep(JOIN_RETRY_MS);
            LMIC_startJoining();
            break;
        case EV_REJOIN_FAILED:
            DBG(F("EV_REJOIN_FAILED"));
            reset_lmic();
            _sleep(REJOIN_WAIT_MS);
            /* NOTE: Does not generate EV_JOINING */
            LMIC_startJoining();
            break;
        case EV_TXCOMPLETE:
            DBG(F("EV_TXCOMPLETE"));
            if (LMIC.txrxFlags & TXRX_ACK) {
                DBG(F("Received ACK"));
            }

            _sleep(SLEEP_TIME_MS);

            os_setCallback(&sendjob, do_send);
            break;
        case EV_RESET:
            DBG(F("EV_RESET"));
            break;
        case EV_LINK_DEAD:
            DBG(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            DBG(F("EV_LINK_ALIVE"));
            break;
        default:
            DBG(F("Unknown event"));
            break;
    }
}

/*
 * Get the battery voltage.
 *
 * NOTE: If battery is not connected, it will read > 4V anyway
 */
float get_battery_voltage()
{
    float batt_voltage = analogRead(VBAT_PIN) * 2 * 3.3 / 1024;
    return batt_voltage;
}

/*
 * Get temperature in celsius
 */
float get_temp()
{
    temp_sensor.requestTemperatures();

    float temp = temp_sensor.getTempCByIndex(0);
    if (temp == DEVICE_DISCONNECTED_C) {
        DBG(F("Failed to get temperature"));
    }

    return temp;
}

void do_send(osjob_t *j)
{
    String msg = "{\"temp\":" + String(get_temp()) +
        ",\"vbat\":" + String(get_battery_voltage()) + "}";

    DBG(msg);

    if (LMIC.opmode & OP_TXRXPEND) {
        DBG(F("Packet pending, not sending"));
    } else {
        if (LMIC_setTxData2(1, msg.c_str(), msg.length(), 0) != 0) {
            DBG(F("Failed to enqueue packet"));
        } else {
            DBG(F("Packet enqueued"));
        }
    }
}

void reset_lmic()
{
    /* Reset MAC */
    LMIC_reset();

    /* Let LMIC compensate for +/- 1% clock error */
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
}

void setup()
{
    delay(5000); /* Makes it easier to flash */

    Serial.begin(9600);
    DBG(F("Starting"));

    temp_sensor.begin();

    os_init();

    reset_lmic();

    LMIC_startJoining();
}

void loop()
{
    os_runloop_once();
}
