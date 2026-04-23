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

			param_handle = param_find("EKF2_AGP0_ID");
			if (param_handle != PARAM_INVALID) {
				int32_t aux_gps_id = 3; // ID for aux_global_position source
				param_set(param_handle, &aux_gps_id);
				PX4_INFO("EKF2 auxiliary GPS set to aux_global_position");
			} else {
				PX4_ERR("Failed to find EKF2_AGP0_ID parameter");
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
	aux_pos.id = 3;
	aux_pos.source = 2;
	_aux_global_position_pub.publish(aux_pos);
}
