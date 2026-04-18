
#include "gps_spoofing_detection.hpp"
#include <cmath>
#include <drivers/drv_hrt.h>
#include <cinttypes>


GpsSpoofingDetection::GpsSpoofingDetection() :
	_spoofing_detected(false),
	_sensitivity_threshold(10.0),
	_update_count(0)
{
}

GpsSpoofingDetection::~GpsSpoofingDetection() = default;

// input for optical flow sensor information
bool GpsSpoofingDetection::checkForOpticalFlowVelUpdate() {

	if (_vehicle_optical_flow_vel_sub.update()) {
		const vehicle_optical_flow_vel_s &optical_flow_estimate = _vehicle_optical_flow_vel_sub.get();
		if (_prev_optical_flow.timestamp_sample == 0) {
			_prev_optical_flow = optical_flow_estimate;
		} else {
			_prev_optical_flow = _optical_flow;
		}
		_optical_flow = optical_flow_estimate;
		_ofv_valid = true;

		return true;
	}

	return false;
}

bool GpsSpoofingDetection::checkForOpticalFlowUpdate() {

	if (_vehicle_optical_flow_sub.update()) {
		const vehicle_optical_flow_s &optical_flow = _vehicle_optical_flow_sub.get();
		_raw_optical_flow = optical_flow;
		_of_valid = true;
		return true;
	}

	return false;
}

// input for GPS sensor information
bool GpsSpoofingDetection::checkForGPSUpdate() {

	if (_vehicle_gps_position_sub.update()) {
		const sensor_gps_s &gps = _vehicle_gps_position_sub.get();

		if (_prev_gps.timestamp == 0) {
			_prev_gps = gps;
		} else {
			_prev_gps = _gps;
		}
		_gps = gps;
		_gps_valid = true;

		return true;
	}
	return false;
}

bool GpsSpoofingDetection::checkForMissionResultUpdate() {
	if (_mission_result_sub.update()) {
		const mission_result_s &result = _mission_result_sub.get();
		_mission_result = result;
		return true;
	}
	return false;
}

bool GpsSpoofingDetection::update() {
	_spoofing_detected = analyzeSignal();
	return _spoofing_detected;
}

float GpsSpoofingDetection::opticalFlowDistance() {
    float dt = (_optical_flow.timestamp_sample - _prev_optical_flow.timestamp_sample) / 1000000.0f;
    float dx = (_optical_flow.vel_ne_filtered[0]);
    float dy = (_optical_flow.vel_ne_filtered[1]);
    return sqrt(dx * dx + dy * dy) * dt;


}


float GpsSpoofingDetection::opticalFlowDistance2() {
	float scaling_factor = 6.7f; // Adjust this factor based on sensor characteristics
	float dx = (_raw_optical_flow.pixel_flow[0] - _raw_optical_flow.delta_angle[0]) * _raw_optical_flow.distance_m * scaling_factor;
	float dy = (_raw_optical_flow.pixel_flow[1] - _raw_optical_flow.delta_angle[1]) * _raw_optical_flow.distance_m * scaling_factor;
	return sqrt(dx * dx + dy * dy);


}


float GpsSpoofingDetection::opticalFlowDistance3() {
	float linear_velocity_x = _raw_optical_flow.pixel_flow[1] * _raw_optical_flow.distance_m;
	float linear_velocity_y = -1 *_raw_optical_flow.pixel_flow[0] * _raw_optical_flow.distance_m;

	float displacement_x = linear_velocity_x * (_raw_optical_flow.integration_timespan_us / 1000000.0f);
	float displacement_y = linear_velocity_y * (_raw_optical_flow.integration_timespan_us / 1000000.0f);

	return sqrt(displacement_x * displacement_x + displacement_y * displacement_y);
}


double GpsSpoofingDetection::GPSDistance(double lon_a, double lat_a, double lon_b, double lat_b) {
	double pi = M_PI;
	double lon_a_rad = (lon_a / 180) * pi;
	double lon_b_rad = (lon_b / 180) * pi;
	double lat_a_rad = (lat_a / 180) * pi;
	double lat_b_rad = (lat_b / 180) * pi;

	double sin_lat = sin((lat_a_rad - lat_b_rad) / 2);
	double sin_lon = sin((lon_a_rad - lon_b_rad) / 2);
	double p = (sin_lat * sin_lat) + cos(lat_a_rad) * cos(lat_b_rad) * (sin_lon * sin_lon);
	return (2 * CONSTANTS_RADIUS_OF_EARTH * asin(sqrt(p)));
}

bool GpsSpoofingDetection::SSDGOF() {
	return abs(_total_distance_gps - (double) _total_distance_flow) > _sensitivity_threshold;
}

bool GpsSpoofingDetection::CUSUM(double of_distance, double gps_distance) {
	double diff = of_distance - gps_distance;
	double baseline_diff = 0;
	double k = 0.05; // smaller k = faster detection, more false alarms
	double thresh = 1.5; // smaller threshold = faster detection, more false

	PX4_INFO("CUSUM diff: %f", diff);

	s_pos = std::max(0.0, s_pos + diff - baseline_diff - k);
	s_neg = std::max(0.0, s_neg - diff + baseline_diff - k);

	PX4_INFO("CUSUM s_pos: %f, s_neg: %f", s_pos, s_neg);

	if (s_pos > thresh || s_neg > thresh) {
		return true; // spoofing detected
	}
	return false; // no spoofing detected
}



bool GpsSpoofingDetection::analyzeSignal() {
	if((hrt_absolute_time() - _gps.timestamp_sample) < 200000 || (hrt_absolute_time() - _optical_flow.timestamp_sample) < 200000) {
		return false;
	}


	GpsSpoofingDetection::checkForOpticalFlowVelUpdate();
	GpsSpoofingDetection::checkForGPSUpdate();
	GpsSpoofingDetection::checkForMissionResultUpdate();
	GpsSpoofingDetection::checkForOpticalFlowUpdate();


	_update_count++;


	if (_ofv_valid && _gps_valid && _of_valid) {

		float of_distance = GpsSpoofingDetection::opticalFlowDistance3();
		_total_distance_flow += of_distance;

		if (_gps.vel_m_s > 0.1f) {
			double gps_distance = GpsSpoofingDetection::GPSDistance(_prev_gps.longitude_deg, _prev_gps.latitude_deg, _gps.longitude_deg, _gps.latitude_deg);
			_total_distance_gps += gps_distance;

			// CUSUM test
			if ((CUSUM(of_distance, gps_distance))) {
				//PX4_ERR("CUSUM GPS SPOOFING DETECTED");
			}

			// SSD-GOF test
			if (SSDGOF()) {
				//PX4_ERR("SSD-GOF GPS SPOOFING DETECTED");
			}
		}

		PX4_INFO("gyro angle x: %f, gyro angle y: %f, gyro angle z: %f", (double) _raw_optical_flow.delta_angle[0], (double) _raw_optical_flow.delta_angle[1], (double) _raw_optical_flow.delta_angle[2]);
		PX4_INFO("px_flow x: %f, px_flow y: %f, ground: %f, int_time: %u, flow_dist: %f", (double) _raw_optical_flow.pixel_flow[0], (double) _raw_optical_flow.pixel_flow[1], (double) _raw_optical_flow.distance_m, _raw_optical_flow.integration_timespan_us, (double) _total_distance_flow);
		PX4_INFO("vel: %f, gps_dist: %f\n\n", (double) _gps.vel_m_s, (double) _total_distance_gps);

	}

	_spoofing_detected = GpsSpoofingDetection::SSDGOF();

	return false;
}

