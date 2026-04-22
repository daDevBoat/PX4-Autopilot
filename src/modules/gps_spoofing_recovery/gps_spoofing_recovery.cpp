#include "gps_spoofing_recovery.hpp"
#include <cmath>
#include <drivers/drv_hrt.h>
#include <cinttypes>
#include <lib/parameters/param.h>

GpsSpoofingRecovery::GpsSpoofingRecovery(GpsSpoofingDetection &gps_spoofing_detection) :
	_gps_spoofing_detection(gps_spoofing_detection)
{

}

GpsSpoofingRecovery::~GpsSpoofingRecovery() = default;


bool GpsSpoofingRecovery::update() {
	if (_gps_spoofing_detection.spoofing_detected) {
		//PX4_INFO("GPS SPOOFING RECOVERY ACTIVATED");
		//param_t param_handle_gps = param_find("EKF2_GPS_CTRL");
		//har* param_value = new char[2];
		//param_get(param_handle_gps, param_value);
		//PX4_INFO("Current EKF2_GPS_CTRL value: %c", param_value[0]);


		if (!_gps_disabled) {
			param_t param_handle = param_find("EKF2_GPS_CTRL");
			if (param_handle != PARAM_INVALID) {
				int32_t disable_value = 0;
				param_set(param_handle, &disable_value);
				PX4_INFO("EKF2 GPS fusion disabled");
				_gps_disabled = true;
			} else {
				PX4_ERR("Failed to find EKF2_GPS_CTRL parameter");
			}
		}

		// Post the opticalFlowPos to the aux gps
		GpsSpoofingRecovery::publishAuxGlobalPosition();


	}

	return false;
}

void GpsSpoofingRecovery::publishAuxGlobalPosition() {
	aux_global_position_s aux_pos{};
	aux_pos.timestamp = hrt_absolute_time();
	aux_pos.lat = _gps_spoofing_detection.getFlowPosition()[0];
	aux_pos.lon = _gps_spoofing_detection.getFlowPosition()[1];
	aux_pos.alt = 0.0; // altitude is not estimated from optical flow
	_aux_global_position_pub.publish(aux_pos);
}
