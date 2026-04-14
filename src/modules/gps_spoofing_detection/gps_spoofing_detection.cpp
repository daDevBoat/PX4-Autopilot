
#include "gps_spoofing_detection.hpp"


GpsSpoofingDetection::GpsSpoofingDetection() :
	_spoofing_detected(false),
	_sensitivity_threshold(0.5f),
	_last_latitude(0.0),
	_last_longitude(0.0),
	_last_accuracy(0.0f),
	_update_count(0)
{

}

GpsSpoofingDetection::~GpsSpoofingDetection() = default;

// input for optical flow sensor information
bool GpsSpoofingDetection::checkForOpticalFlowEstimateUpdate() {

	if (_vehicle_optical_flow_estimate_sub.update()) {
		const vehicle_optical_flow_vel_s &optical_flow_estimate = _vehicle_optical_flow_estimate_sub.get();
		//save the filtered velocity information for use in comparison computation
		_of_vel_n = optical_flow_estimate.vel_ne_filtered[0];
		_of_vel_e = optical_flow_estimate.vel_ne_filtered[1];
				// process the optical flow estimate for spoofing detection
		_of_valid = true;

		return true;
	}

	return false;
}

// input for GPS sensor information
bool GpsSpoofingDetection::checkForGPSUpdate() {

	if (_vehicle_gps_position_sub.update()) {
		const sensor_gps_s &gps = _vehicle_gps_position_sub.get();
		_gps_vel_n = gps.vel_n_m_s;
		_gps_vel_e = gps.vel_e_m_s;
		_gps_valid = true;

		return true;
	}
	return false;
}

bool GpsSpoofingDetection::update() {
	_update_count++;

	_spoofing_detected = analyze_signal();
	return _spoofing_detected;
}

bool GpsSpoofingDetection::analyze_signal() {
	// Add anomly detetion logic here using optical flow etc
	GpsSpoofingDetection::checkForOpticalFlowEstimateUpdate();
	GpsSpoofingDetection::checkForGPSUpdate();

	//PX4_INFO(_of_valid ? "OF valid" : "OF not valid");
	//PX4_INFO(_gps_valid ? "GPS valid" : "GPS not valid");

	if (_of_valid && _gps_valid) {
		//PX4_INFO("HELLO");
		const float diff_n = _gps_vel_n - _of_vel_n;
		const float diff_e = _gps_vel_e - _of_vel_e;
		const float vel_error = sqrtf(diff_n * diff_n + diff_e * diff_e);
		_of_valid = false;
		_gps_valid = false;

		PX4_INFO("Velocity error: %.2f m/s", (double)vel_error);


		return vel_error > _sensitivity_threshold;

	}

	return false;
}

void GpsSpoofingDetection::reset() {
	_spoofing_detected = false;
	_update_count = 0;
}

bool GpsSpoofingDetection::is_spoofing_detected() const {
	return _spoofing_detected;
}
