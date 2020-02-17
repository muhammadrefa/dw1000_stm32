#ifndef DW1000_TIME_H_
#define DW1000_TIME_H_

typedef uint64_t dw1000_timestamp_t;

const double DW1000_TIME_RES = 0.000015650040064103f;
const double DW1000_TIME_RES_INV = 63897.6f;

const double DW1000_TIME_DISTANCE_OF_RADIO = 0.0046917639786159f;
const double DW1000_TIME_DISTANCE_OF_RADIO_INV = 213.139451293f;

dw1000_timestamp_t DW1000_Time_MicrosecondsToTimestamp(double time_us);
double DW1000_Time_TimestampToMicroseconds(dw1000_timestamp_t timestamp);
double DW1000_Time_TimestampToMeters(dw1000_timestamp_t timestamp);
#endif /* DW1000_TIME_H_ */
