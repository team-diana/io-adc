#include "adc.h"
#include "dask.h"

#include <iostream>

#include <stdio.h>

int main() {

//     I16 card = -1;
//     I16 err = 0;
//     I16 dio = -1;
//     std::string message;
//     int range_imu;
//     F32 max_voltage;
//     F32 max_voltage_range;
//
//
//     SET_MAX_RANGE(enable_imu || enable_sosp || enable_imu_2, range_imu, max_voltage);
//
//     if ((card = Register_Card(PCI_9116, 0)) < 0) {
//         std::cout <<  "ADC Register_Card error= " <<  card << std::endl;
//         message = "ADC Register_Card error";
//         exit(1);
//     }
//
//       if (err = AI_9116_Config(card, P9116_AI_SingEnded|P9116_AI_UserCMMD, P9116_AI_SoftPolling ,0, 0, 0) != NoError) {
//         printf("Config error : error_code: %d \n", err);
//         std::stringstream ms;
//         ms << "Config error Error: error_code: " << int(err);
//         message = ms.str();
//     }
//
//     if ((dio = Register_Card(PCI_7432, 0)) < 0) {
//         std::cout <<  "DIO Register_Card error= " <<  dio << std::endl;
//         message = "DIO Register_Card error";
//         exit(1);
//     }
//
//     int i = 66;
//     I16 chan_data_raw;
//     I16 chan_raw;
//     F32 chan_voltage;
//     I16 chan_data;
//
//     while(true) {
//
//         if ((err = AI_ReadChannel(card, i, range_imu, (U16*) &chan_data_raw)) != NoError) {
//             printf("AI_ReadChannel Ch#%d error : error_code: %d \n", i, err);
//             std::stringstream ms;
//             ms << "AI_ReadChannel Error Ch#" << i << " error : error_code: " << int(err);
//             message = ms.str();
//         }
//
//         chan_data = chan_data_raw;
//         chan_voltage = (((F32) chan_data) / AG_MAX_VALUE) * max_voltage;
//         chan_data = int(chan_voltage * 1000);
//
//         double total = 0.0;
//         I16 cdata = chan_data_raw;
//         printf("value is %f\n", chan_voltage);
//     }

}
