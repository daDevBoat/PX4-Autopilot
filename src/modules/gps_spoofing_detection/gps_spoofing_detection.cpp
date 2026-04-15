
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
		//save the filtered velocity information for use in comparison computation
		_ground_distance = optical_flow.distance_m;
				// process the optical flow estimate for spoofing detection
		_opt_flow = optical_flow;
		if(_ground_distance >= 9.8f) {
			_reached_height = true;
		}
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

bool GpsSpoofingDetection::update() {
	_spoofing_detected = analyzeSignal();
	return _spoofing_detected;
}

float GpsSpoofingDetection::opticalFlowDistance(float ground_distance, float flow_x, float flow_y) {

    //float dx = ground_distance * _opt_flow.pixel_flow[0];
    //float dy = ground_distance * _opt_flow.pixel_flow[1];

    float dx = ground_distance * (_opt_flow.pixel_flow[0] - _opt_flow.delta_angle[0]);
    float dy = ground_distance * (_opt_flow.pixel_flow[1] - _opt_flow.delta_angle[1]);
    return sqrt(dx * dx + dy * dy);


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

float GpsSpoofingDetection::GPSDistance2(float lon_a, float lat_a, float lon_b, float lat_b) {
    float pi = (float) M_PI;
    const float R = (float) CONSTANTS_RADIUS_OF_EARTH;
    float phi1 = lat_a * pi / 180.0f;
    float phi2 = lat_b * pi / 180.0f;
    float delta_phi = (lat_b - lat_a) * pi / 180.0f;
    float delta_lambda = (lon_b - lon_a) * pi / 180.0f;

    float a = sin(delta_phi / 2) * sin(delta_phi / 2) +
               cos(phi1) * cos(phi2) *
               sin(delta_lambda / 2) * sin(delta_lambda / 2);
    float c = 2 * atan2(sqrt(a), sqrt(1 - a));

    return R * c; // Distance in meters
}



bool GpsSpoofingDetection::analyzeSignal() {
	if((hrt_absolute_time() - _gps.timestamp_sample) < 200000 || (hrt_absolute_time() - _optical_flow.timestamp_sample) < 200000) {
		return false;
	}


	GpsSpoofingDetection::checkForOpticalFlowEstimateUpdate();
	GpsSpoofingDetection::checkForOpticalFlowUpdate();
	GpsSpoofingDetection::checkForGPSUpdate();

	_update_count++;

	if (_reached_height && _do_once) {
		_total_distance_flow = 0.0f;
		_total_distance_gps = 0.0;
		_do_once = false;
	}

	//PX4_INFO(_of_valid ? "OF valid" : "OF not valid");
	//PX4_INFO(_gps_valid ? "GPS valid" : "GPS not valid");

	if (_ofe_valid && _of_valid && _gps_valid) {

		float of_distance = GpsSpoofingDetection::opticalFlowDistance(_ground_distance, _flow_x, _flow_y);
		_total_distance_flow += of_distance;

		double gps_distance = GpsSpoofingDetection::GPSDistance(_prev_gps.longitude_deg, _prev_gps.latitude_deg, _gps.longitude_deg, _gps.latitude_deg);
		_total_distance_gps += gps_distance; //- (_gps.timestamp_sample - _prev_gps.timestamp_sample) * (double)_gps.s_variance_m_s;

		//if (_update_count % 5 == 0) {
			PX4_INFO("fx: %f, fy: %f, ground_distance: %f, quaility: %u, flow_dist: %f", (double) _opt_flow.pixel_flow[0], (double) _opt_flow.pixel_flow[1], (double) _ground_distance, (unsigned) _opt_flow.quality, (double) _total_distance_flow);
			PX4_INFO("lon_a: %f, lat_a: %f, lon_b: %f, lat_b: %f, gps_dist: %f", (double) _prev_gps.longitude_deg, (double) _prev_gps.latitude_deg, (double) _gps.longitude_deg, (double) _gps.latitude_deg, (double) _total_distance_gps);
		//}

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
