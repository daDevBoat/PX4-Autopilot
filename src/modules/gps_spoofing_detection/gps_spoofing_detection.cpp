
#include "gps_spoofing_detection.hpp"
#include <cmath>
#include <drivers/drv_hrt.h>
#include <cinttypes>


GpsSpoofingDetection::GpsSpoofingDetection() :
	_spoofing_detected(false),
	_sensitivity_threshold(0.5f),
	_last_latitude(0.0),
	_last_longitude(0.0),
	_last_accuracy(0.0f),
	_update_count(0)
{
	//_vehicle_gps_position_sub.set_interval_ms(200);
	//_vehicle_optical_flow_estimate_sub.set_interval_ms(200);
	//_vehicle_optical_flow_sub.set_interval_ms(200);

}

GpsSpoofingDetection::~GpsSpoofingDetection() = default;

// input for optical flow sensor information
bool GpsSpoofingDetection::checkForOpticalFlowEstimateUpdate() {

	if (_vehicle_optical_flow_estimate_sub.update()) {
		const vehicle_optical_flow_vel_s &optical_flow_estimate = _vehicle_optical_flow_estimate_sub.get();
		//save the filtered velocity information for use in comparison computation
		_flow_x = optical_flow_estimate.flow_rate_compensated[0];
		_flow_y = optical_flow_estimate.flow_rate_compensated[1];

		if (_prev_optical_flow.timestamp_sample == 0) {
			_prev_optical_flow = optical_flow_estimate;
		} else {
			_prev_optical_flow = _optical_flow;
		}

		_optical_flow = optical_flow_estimate;

		_ofe_valid = true;

		return true;
	}

	return false;
}

bool GpsSpoofingDetection::checkForOpticalFlowUpdate() {

	if (_vehicle_optical_flow_sub.update()) {
		const vehicle_optical_flow_s &optical_flow = _vehicle_optical_flow_sub.get();
		_ground_distance = optical_flow.distance_m;
		_opt_flow = optical_flow;
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

float GpsSpoofingDetection::opticalFlowDistance(float ground_distance, float flow_x, float flow_y) {
    float dt = (_optical_flow.timestamp_sample - _prev_optical_flow.timestamp_sample) / 1000000.0f;
    float dx = (_optical_flow.vel_ne_filtered[0]);
    float dy = (_optical_flow.vel_ne_filtered[1]);

    //float dx = ground_distance * (_opt_flow.pixel_flow[0] - _opt_flow.delta_angle[0]);
    //float dy = ground_distance * (_opt_flow.pixel_flow[1] - _opt_flow.delta_angle[1]);
    return sqrt(dx * dx + dy * dy) * dt;


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



bool GpsSpoofingDetection::analyzeSignal() {
	if((hrt_absolute_time() - _gps.timestamp_sample) < 200000 || (hrt_absolute_time() - _optical_flow.timestamp_sample) < 200000) {
		return false;
	}


	GpsSpoofingDetection::checkForOpticalFlowEstimateUpdate();
	GpsSpoofingDetection::checkForOpticalFlowUpdate();
	GpsSpoofingDetection::checkForGPSUpdate();
	GpsSpoofingDetection::checkForMissionResultUpdate();

	// If first step is done (should be takeoff to certain height)
	if (_mission_result.timestamp > 0 && _mission_result.seq_reached == 0 && _do_once) {
		_total_distance_flow = 0.0f;
		_total_distance_gps = 0.0;
		_do_once = false;
	}

	_update_count++;


	//PX4_INFO(_of_valid ? "OF valid" : "OF not valid");
	//PX4_INFO(_gps_valid ? "GPS valid" : "GPS not valid");

	if (_ofe_valid && _of_valid && _gps_valid) {

		float of_distance = GpsSpoofingDetection::opticalFlowDistance(_ground_distance, _flow_x, _flow_y);
		_total_distance_flow += of_distance;

		if (_gps.vel_m_s > 0.1f) {
			double gps_distance = GpsSpoofingDetection::GPSDistance(_prev_gps.longitude_deg, _prev_gps.latitude_deg, _gps.longitude_deg, _gps.latitude_deg);
			_total_distance_gps += gps_distance;
		}


		//if (_update_count % 5 == 0) {
			PX4_INFO("fx: %f, fy: %f, ground_distance: %f, quaility: %u, flow_dist: %f", (double) _opt_flow.pixel_flow[0], (double) _opt_flow.pixel_flow[1], (double) _ground_distance, (unsigned) _opt_flow.quality, (double) _total_distance_flow);
			PX4_INFO("vel: %f, gps_dist: %f\n\n", (double) _gps.vel_m_s, (double) _total_distance_gps);
		//}

	}

	if (abs(_total_distance_gps - (double) _total_distance_flow) > 10) {
		PX4_ERR("GPS SPOOFING DETECTED");
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
