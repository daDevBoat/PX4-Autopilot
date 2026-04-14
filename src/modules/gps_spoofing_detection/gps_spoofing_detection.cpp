
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
		_flow_x = optical_flow_estimate.flow_rate_compensated[0];
		_flow_y = optical_flow_estimate.flow_rate_compensated[1];
				// process the optical flow estimate for spoofing detection
		_ofe_valid = true;

		return true;
	}

	return false;
}

bool GpsSpoofingDetection::checkForOpticalFlowUpdate() {

	if (_vehicle_optical_flow_sub.update()) {
		const vehicle_optical_flow_s &optical_flow = _vehicle_optical_flow_sub.get();
		//save the filtered velocity information for use in comparison computation
		_ground_distance = optical_flow.distance_m;
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

	_spoofing_detected = analyzeSignal();
	return _spoofing_detected;
}

float GpsSpoofingDetection::opticalFlowDistance(float ground_distance, float flow_x, float flow_y) {
    float dx = (ground_distance * flow_x) / 10000.0f;
    float dy = (ground_distance * flow_y) / 10000.0f;
    return sqrt(dx * dx + dy * dy);
}



bool GpsSpoofingDetection::analyzeSignal() {
	// Add anomly detetion logic here using optical flow etc
	GpsSpoofingDetection::checkForOpticalFlowEstimateUpdate();
	GpsSpoofingDetection::checkForOpticalFlowUpdate();
	GpsSpoofingDetection::checkForGPSUpdate();



	//PX4_INFO(_of_valid ? "OF valid" : "OF not valid");
	//PX4_INFO(_gps_valid ? "GPS valid" : "GPS not valid");

	if (_ofe_valid && _of_valid && _gps_valid) {
		float of_distance = GpsSpoofingDetection::opticalFlowDistance(_ground_distance, _flow_x, _flow_y);
		_total_distance_flow += of_distance;
		PX4_INFO("flow_x: %f, flow_y: %f, ground_distance: %f, flow_dist: %f", (double) _flow_x, (double) _flow_y, (double) _ground_distance, (double) _total_distance_flow);

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
