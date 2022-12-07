
#include "filters.h"
#include "bdshot.h"
#include "setup.h"

// This is only a sketch how to use RPM filters
// You should take care of time management and updating samples

int main()
{
    float gyro_measurements[3]; // X,Y,Z
    RPM_filter_t rpm_filter_gyro;

    setup();
    setup_NVIC();
    preset_bb_BDshot_buffers();
    RPM_filter_init(&rpm_filter_gyro, FREQUENCY_OF_SAMPLING_HZ);

    while (1)
    {
        /*
            here should be some kind of scheduler to provide expected frequencies
            gyro_measurements should be updated
        */

        // send BDshot frame and receive ESC response (updating motors rpm values is inside):
        update_motors();

        // update coefficients of notches for new rpms:
        RPM_filter_update(&rpm_filter_gyro);

        // next apply RPM filtering:
        gyro_measurements[0] = RPM_filter_apply(&rpm_filter_gyro, 0, gyro_measurements[0]);
        gyro_measurements[1] = RPM_filter_apply(&rpm_filter_gyro, 1, gyro_measurements[1]);
        gyro_measurements[2] = RPM_filter_apply(&rpm_filter_gyro, 2, gyro_measurements[2]);
    }
    return 0;
}