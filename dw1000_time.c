#include "dw1000_time.h"

dw1000_timestamp_t DW1000_Time_MicrosecondsToTimestamp(double time_us) {
  return (dw1000_timestamp_t)(time_us * DW1000_TIME_RES_INV);
}

double DW1000_Time_TimestampToMicroseconds(dw1000_timestamp_t timestamp) {
  return timestamp*DW1000_TIME_RES;
}

double DW1000_Time_TimestampToMeters(dw1000_timestamp_t timestamp) {
  return timestamp*DW1000_TIME_DISTANCE_OF_RADIO;
}
