#include "ble_bmi160.h"
#include "ble_srv_common.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "sdk_common.h"
#include <string.h>

#define OPCODE_LENGTH 1
#define HANDLE_LENGTH 2
//#define MAX_PACKET_LEN (uint8_t)(20)
#define PACKET_CUTOFF 4
uint8_t MAX_PACKET_LEN = 4;


static __INLINE uint8_t int16_encode(int16_t value, int8_t *p_encoded_data) {
  p_encoded_data[0] = (uint8_t)((value & 0x00FF) >> 0);
  p_encoded_data[1] = (int8_t)((value & 0xFF00) >> 8);
  return sizeof(int16_t);
}
static void on_connect(ble_bmi160_t *p_bmi160, ble_evt_t const *p_ble_evt) {
  p_bmi160->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}

static void on_disconnect(ble_bmi160_t *p_bmi160, ble_evt_t const *p_ble_evt) {
  UNUSED_PARAMETER(p_ble_evt);
  p_bmi160->conn_handle = BLE_CONN_HANDLE_INVALID;
}

static void on_complete(ble_bmi160_t *p_bmi160, ble_evt_t const *p_ble_evt) {
  UNUSED_PARAMETER(p_ble_evt);
  UNUSED_PARAMETER(p_bmi160);
  //  NRF_LOG_INFO("on_complete\r\n");
  if (p_bmi160->evt_handler != NULL) {
    ble_bmi160_evt_t evt;
    evt.evt_type = BLE_BMI160_EVT_TX_COMPLETE;
    p_bmi160->evt_handler(p_bmi160, &evt);
  }
}

static void on_bmi_cccd_write(ble_bmi160_t *p_bmi160, ble_gatts_evt_write_t const *p_evt_write) {
  if (p_evt_write->len == 2) {
    // CCCD written, update notification state
    if (p_bmi160->evt_handler != NULL) {
      ble_bmi160_evt_t evt;
      if (ble_srv_is_notification_enabled(p_evt_write->data)) {
        evt.evt_type = BLE_BMI160_EVT_NOTIFICATION_ENABLED;
      } else {
        evt.evt_type = BLE_BMI160_EVT_NOTIFICATION_DISABLED;
      }
      p_bmi160->evt_handler(p_bmi160, &evt);
    }
  }
}

static void on_write(ble_bmi160_t *p_bmi160, ble_evt_t const *p_ble_evt) {
  ble_gatts_evt_write_t const *p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

  if (p_evt_write->handle == p_bmi160->bmi_handles.cccd_handle) {
    on_bmi_cccd_write(p_bmi160, p_evt_write);
  }
}

void ble_bmi160_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context) {
  ble_bmi160_t *p_bmi160 = (ble_bmi160_t *)p_context;
  switch (p_ble_evt->header.evt_id) {
  case BLE_GAP_EVT_CONNECTED:
    on_connect(p_bmi160, p_ble_evt);
    printf("\n Min Conn: %d, Max Conn: %d, Latency: %d, timeout: %d \r\n",
        p_ble_evt->evt.gap_evt.params.connected.conn_params.min_conn_interval * 1250 / 1000,
        p_ble_evt->evt.gap_evt.params.connected.conn_params.max_conn_interval * 1250 / 1000,
        p_ble_evt->evt.gap_evt.params.connected.conn_params.slave_latency * 1250 / 1000,
        p_ble_evt->evt.gap_evt.params.connected.conn_params.conn_sup_timeout * 10);

    break;
  case BLE_GAP_EVT_DISCONNECTED:
    on_disconnect(p_bmi160, p_ble_evt);
    break;
  case BLE_GATTS_EVT_HVN_TX_COMPLETE:
    on_complete(p_bmi160, p_ble_evt);
    break;
  case BLE_GATTS_EVT_WRITE:
    on_write(p_bmi160, p_ble_evt);
    break;

  case BLE_GAP_EVT_CONN_PARAM_UPDATE:
    printf("\n Min Conn: %d, Max Conn: %d, Latency: %d, timeout: %d \r\n",
        p_ble_evt->evt.gap_evt.params.connected.conn_params.min_conn_interval * 1250 / 1000,
        p_ble_evt->evt.gap_evt.params.connected.conn_params.max_conn_interval * 1250 / 1000,
        p_ble_evt->evt.gap_evt.params.connected.conn_params.slave_latency * 1250 / 1000,
        p_ble_evt->evt.gap_evt.params.connected.conn_params.conn_sup_timeout * 10);

    break;
  default:
    // No implementation needed.
    break;
  }
}

static uint8_t bmi_encode(ble_bmi160_t *p_bmi160, uint16_t  accel, uint16_t gyro, uint8_t *p_encoded_buffer) {

  uint8_t len = 0;
  len += uint16_encode(accel, &p_encoded_buffer[len]);
  len += uint16_encode(gyro, &p_encoded_buffer[len]);
  return len;
}

static uint32_t bmi_data_char_add(ble_bmi160_t *p_bmi160, const ble_bmi160_init_t *p_bmi160_init) {
  ble_gatts_char_md_t char_md;
  ble_gatts_attr_md_t cccd_md;
  ble_gatts_attr_t attr_char_value;
  ble_uuid_t ble_uuid;
  ble_gatts_attr_md_t attr_md;
  uint8_t encoded_initial_bmi[MAX_PACKET_LEN];
  
//  struct bmi160_sensor_data accel;
//  struct bmi160_sensor_data gyro;

  memset(&cccd_md, 0, sizeof(cccd_md));

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
  cccd_md.write_perm = p_bmi160_init->bmi160_attr_md.cccd_write_perm;
  cccd_md.vloc = BLE_GATTS_VLOC_STACK;

  memset(&char_md, 0, sizeof(char_md));

  char_md.char_props.notify = 1;
  char_md.p_char_user_desc = NULL;
  char_md.p_char_pf = NULL;
  char_md.p_user_desc_md = NULL;
  char_md.p_cccd_md = &cccd_md;
  char_md.p_sccd_md = NULL;

  ble_uuid.type = p_bmi160->uuid_type;
  ble_uuid.uuid = BLE_BMI160_UUID_VALUE_CHAR;

  //  BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_HEART_RATE_MEASUREMENT_CHAR);

  memset(&attr_md, 0, sizeof(attr_md));

  attr_md.read_perm = p_bmi160_init->bmi160_attr_md.read_perm;
  attr_md.write_perm = p_bmi160_init->bmi160_attr_md.write_perm;
  attr_md.vloc = BLE_GATTS_VLOC_STACK;
  attr_md.rd_auth = 0;
  attr_md.wr_auth = 0;
  attr_md.vlen = MAX_PACKET_LEN;

  memset(&attr_char_value, 0, sizeof(attr_char_value));

uint16_t accel = 1;
uint16_t gyro = 1;


  attr_char_value.p_uuid = &ble_uuid;
  attr_char_value.p_attr_md = &attr_md;
  attr_char_value.init_len = MAX_PACKET_LEN;
  attr_char_value.init_offs = 0;
  attr_char_value.max_len = MAX_PACKET_LEN;
  attr_char_value.p_value = encoded_initial_bmi;

  return sd_ble_gatts_characteristic_add(p_bmi160->service_handle,
      &char_md,
      &attr_char_value,
      &p_bmi160->bmi_handles);
}

uint32_t ble_bmi160_init(ble_bmi160_t *p_bmi160, const ble_bmi160_init_t *p_bmi160_init) {
  uint32_t err_code;
  ble_uuid_t ble_uuid;

  // Initialize service structure
  p_bmi160->evt_handler = p_bmi160_init->evt_handler;
  p_bmi160->conn_handle = BLE_CONN_HANDLE_INVALID;
  p_bmi160->max_bmi_len = MAX_PACKET_LEN;

  ble_uuid128_t base_uuid = {BLE_BMI160_UUID_BASE};
  err_code = sd_ble_uuid_vs_add(&base_uuid, &p_bmi160->uuid_type);
  if (err_code != NRF_SUCCESS) {
    return err_code;
  }

  // Add service
  //  BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_BATTERY_SERVICE);
  ble_uuid.type = p_bmi160->uuid_type;
  ble_uuid.uuid = BLE_BMI160_UUID_SERVICE;

  err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
      &ble_uuid,
      &p_bmi160->service_handle);

  if (err_code != NRF_SUCCESS) {
    return err_code;
  }

  err_code = bmi_data_char_add(p_bmi160, p_bmi160_init);
  if (err_code != NRF_SUCCESS) {
    return err_code;
  }

  return NRF_SUCCESS;
}

uint32_t ble_bmi160_measurement_send(ble_bmi160_t *p_bmi160,  uint16_t accel, uint16_t  gyro) {
  uint32_t err_code;
  
  // Send value if connected and notifying
  if (p_bmi160->conn_handle != BLE_CONN_HANDLE_INVALID) {
    uint8_t encoded_bmi[MAX_PACKET_LEN];
    uint16_t len;
    uint16_t hvx_len;
    ble_gatts_hvx_params_t hvx_params;

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_bmi160->bmi_handles.value_handle;
    hvx_params.type = BLE_GATT_HVX_NOTIFICATION;
    hvx_params.offset = 0;

    len = bmi_encode(p_bmi160, accel, gyro, encoded_bmi);
    

//    len = 4;
    NRF_LOG_INFO("ahahahaha %d", len);
    len = len;
    hvx_len = len;
    
//    NRF_LOG_INFO("ttteest %d",len);
    /*++++++++++++++++++*/
//   for(uint8_t i = 0; i<=16; i++){
//    encoded_bmi[i] = quaternion_my[i];
//   } 
//   len = 16;
//   hvx_len = 16;
   //encoded_bmi[19] = 1;
    /*++++++++++++++++++*/


    hvx_params.p_len = &hvx_len;
    hvx_params.p_data = encoded_bmi;
    err_code = sd_ble_gatts_hvx(p_bmi160->conn_handle, &hvx_params);
    //memset(&hvx_params, 0, sizeof(hvx_params));

    if ((err_code == NRF_SUCCESS) && (hvx_len != len)) {
      err_code = NRF_ERROR_DATA_SIZE;
    }
  } else {
    err_code = NRF_ERROR_INVALID_STATE;
  }

  return err_code;
}

void ble_bmi160_on_gatt_evt(ble_bmi160_t *p_bmi160, nrf_ble_gatt_evt_t const *p_gatt_evt) {
  if ((p_bmi160->conn_handle == p_gatt_evt->conn_handle) && (p_gatt_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)) {
    p_bmi160->max_bmi_len = p_gatt_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
  }
}