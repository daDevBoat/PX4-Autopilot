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
			/*
			if (param_handle != PARAM_INVALID) {
				int32_t disable_value = 0;
				param_set(param_handle, &disable_value);
				PX4_INFO("EKF2 GPS fusion disabled");
				_gps_disabled = true;
			} else {
				PX4_ERR("Failed to find EKF2_GPS_CTRL parameter");
			}
			*/
			param_handle = param_find("EKF2_AGP0_ID");
			if (param_handle != PARAM_INVALID) {
				int32_t aux_gps_id = 3; // ID for aux_global_position source
				param_set(param_handle, &aux_gps_id);
				PX4_INFO("EKF2 auxiliary GPS set to aux_global_position");
			} else {
				PX4_ERR("Failed to find EKF2_AGP0_ID parameter");
			}

			param_handle = param_find("EKF2_AGP0_CTRL");
			if (param_handle != PARAM_INVALID) {
				int32_t enable_vertical_value = 1; // enable aux_global_position fusion
				param_set(param_handle, &enable_vertical_value);
				PX4_INFO("EKF2 auxiliary GPS fusion enabled");
			} else {
				PX4_ERR("Failed to find EKF2_AGP0_CTRL parameter");
			}
		}

		// Post the opticalFlowPos to the aux gps
		GpsSpoofingRecovery::publishAuxGlobalPosition();
		//GpsSpoofingRecovery::publishSensorGPS();


	}

	return false;
}

void GpsSpoofingRecovery::publishAuxGlobalPosition() {
	aux_global_position_s aux_pos{};
	aux_pos.timestamp = hrt_absolute_time();
	aux_pos.timestamp_sample = aux_pos.timestamp;
	aux_pos.lat = _gps_spoofing_detection.getFlowPosition()[0];
	aux_pos.lon = _gps_spoofing_detection.getFlowPosition()[1];
	aux_pos.alt = 0.0; // altitude is not estimated from optical flow
	aux_pos.id = 3;
	aux_pos.source = 2;
	aux_pos.eph = 1.0f;
    	aux_pos.epv = 100.0f;
	_aux_global_position_pub.publish(aux_pos);
}

void GpsSpoofingRecovery::publishSensorGPS() {
	sensor_gps_s gps_msg{};
	gps_msg.timestamp = hrt_absolute_time();
	gps_msg.latitude_deg = _gps_spoofing_detection.getFlowPosition()[0];
	gps_msg.longitude_deg = _gps_spoofing_detection.getFlowPosition()[1];
	gps_msg.altitude_msl_m = 0.0; // altitude is not estimated from optical flow
	gps_msg.satellites_used = 20;
	gps_msg.eph = 0.1; // set a default horizontal position accuracy
	gps_msg.epv = 0.1; // set a default vertical position accuracy
	gps_msg.s_variance_m_s = 0.1; // set a default speed variance
	gps_msg.c_variance_rad = 0.001; // set a default course variance
	gps_msg.fix_type = 3; // set fix type to 3D fix
	_sensor_gps_pub.publish(gps_msg);
}
