#include "gps_spoofing_recovery.hpp"
#include <cmath>
#include <drivers/drv_hrt.h>
#include <cinttypes>
#include <lib/parameters/param.h>

GpsSpoofingRecovery::GpsSpoofingRecovery(const GpsSpoofingDetection &gps_spoofing_detection) :
	_gps_spoofing_detection(gps_spoofing_detection)
{

}

GpsSpoofingRecovery::~GpsSpoofingRecovery() = default;


bool GpsSpoofingRecovery::update() {
	if (_gps_spoofing_detection.spoofing_detected) {
		//PX4_INFO("GPS SPOOFING RECOVERY ACTIVATED");
		if (!_gps_disabled) {
			param_t param_handle = param_find("EKF2_GPS_CTRL");
			if (param_handle != PARAM_INVALID) {
				param_set(param_handle, "0");
				PX4_INFO("EKF2 GPS fusion disabled");
				_gps_disabled = true;
			} else {
				PX4_ERR("Failed to find EKF2_GPS_CTRL parameter");
			}
		}

		// Post the opticalFlowPos to the aux gps


	}

	return false;
}


