#ifndef BLE_BMI160_H__
#define BLE_BMI160_H__

#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_ble_gatt.h"
#include "nrf_sdh_ble.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_BMI160_BLE_OBSERVER_PRIO 2
#define BLE_BMI160_UUID_BASE                        \
  { 0xa8, 0xb4, 0xc9, 0x4a, 0xad, 0x6c, 0x11, 0xe6, \
    0x9a, 0xd8, 0x5d, 0x07, 0xa0, 0x9d, 0x94, 0x6b }

#define BLE_BMI160_UUID_SERVICE 0x3144
#define BLE_BMI160_UUID_COM_CHAR 0x3145
#define BLE_BMI160_UUID_VALUE_CHAR 0x3146

#define BLE_BMI160_DEF(_name)       \
  static ble_bmi160_t _name;        \
  NRF_SDH_BLE_OBSERVER(_name##_obs, \
      BLE_BMI160_BLE_OBSERVER_PRIO, \
      ble_bmi160_on_ble_evt, &_name)

typedef enum {
  BLE_BMI160_EVT_NOTIFICATION_ENABLED,
  BLE_BMI160_EVT_NOTIFICATION_DISABLED,
  BLE_BMI160_EVT_TX_COMPLETE
} ble_bmi160_evt_type_t;

typedef struct
{
  ble_bmi160_evt_type_t evt_type; /**< Type of event. */
} ble_bmi160_evt_t;

typedef struct ble_bmi160_s ble_bmi160_t;

typedef void (*ble_bmi160_evt_handler_t)(ble_bmi160_t *p_bmi160, ble_bmi160_evt_t *p_evt);

typedef struct
{
  ble_bmi160_evt_handler_t evt_handler;
  ble_srv_cccd_security_mode_t bmi160_attr_md;
} ble_bmi160_init_t;

struct ble_bmi160_s {
  ble_bmi160_evt_handler_t evt_handler;
  uint16_t service_handle;
  ble_gatts_char_handles_t bmi_handles;
  uint8_t uuid_type;
  uint16_t conn_handle;
  uint8_t max_bmi_len;
};

uint32_t ble_bmi160_init(ble_bmi160_t *p_bmi160, ble_bmi160_init_t const *p_bmi160_init);
void ble_bmi160_on_gatt_evt(ble_bmi160_t *p_bmi160, nrf_ble_gatt_evt_t const *p_gatt_evt);
void ble_bmi160_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context);
uint32_t ble_bmi160_measurement_send(ble_bmi160_t *p_bmi160, uint16_t accel,  uint16_t gyro);

#ifdef __cplusplus
}
#endif

#endif // BLE_BMI160_H__

/** @} */