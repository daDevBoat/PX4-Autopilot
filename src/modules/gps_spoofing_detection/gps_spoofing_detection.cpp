#include <stdint.h>
#include <stdbool.h>

#include "gps_spoofing_detection.hpp"


GpsSpoofingDetection::GpsSpoofingDetection() :
	_initialized(false),
	_spoofing_detected(false),
	_sensitivity_threshold(0.5f),
	_last_latitude(0.0),
	_last_longitude(0.0),
	_last_accuracy(0.0f),
	_update_count(0)
{

}

bool GpsSpoofingDetection::init() {
	_initialized = true;
	return true;
}

bool GpsSpoofingDetection::update(double latitude, double longitude, float accuracy) {
	if (!_initialized) {
		return false;
	}

	_last_latitude = latitude;
	_last_longitude = longitude;
	_last_accuracy = accuracy;
	_update_count++;

	_spoofing_detected = analyze_signal();
	return _spoofing_detected;
}

bool GpsSpoofingDetection::analyze_signal() {
	// Add anomly detetion logic here using optical flow etc

	return false;
}

void GpsSpoofingDetection::reset() {
	_spoofing_detected = false;
	_update_count = 0;
}

bool GpsSpoofingDetection::is_spoofing_detected() const {
	return _spoofing_detected;
}
