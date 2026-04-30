
#include "gps_spoofing_detection.hpp"
#include <cmath>
#include <drivers/drv_hrt.h>
#include <cinttypes>

#include <lib/parameters/param.h>
#include <modules/commander/px4_custom_mode.h>


GpsSpoofingDetection::GpsSpoofingDetection() :
	spoofing_detected(false),
	_sensitivity_threshold(10.0),
	_update_count(0)
{

	param_t param_handle = param_find("EKF2_GPS_CTRL");
	if (param_handle != PARAM_INVALID) {
		int32_t default_value = 7;
		param_set(param_handle, &default_value);
		PX4_INFO("EKF2 GPS fusion enabled");
	} else {
		PX4_ERR("Failed to find EKF2_GPS_CTRL parameter");
	}

	_gps_spoof_plan_param = param_find("GPS_SPOOF_PLAN");

	if (_gps_spoof_plan_param == PARAM_INVALID) {
		PX4_ERR("Failed to find GPS_SPOOF_PLAN parameter");
	}

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

// input for GPS sensor information
bool GpsSpoofingDetection::checkForGPSUpdate() {

	if (_vehicle_gps_position_sub.update()) {
		const sensor_gps_s &gps = _vehicle_gps_position_sub.get();

		if (_prev_gps.timestamp == 0) {
			_prev_gps = gps;
			_initial_gps = gps;
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
	analyzeSignal();

	int plan = getSpoofPlan();
	bool reject_gps = spoofing_detected && plan != 0;

	gps_spoofing_status_s status{};
	status.timestamp = hrt_absolute_time();
	status.spoofing_detected = spoofing_detected;

	if (reject_gps) {
		status.action = gps_spoofing_status_s::SPOOFING_ACTION_REJECT_GPS;
	} else {
		status.action = gps_spoofing_status_s::SPOOFING_ACTION_NONE;
	}

	status.confidence = spoofing_detected ? 1.0f : 0.0f;
	_gps_spoofing_status_pub.publish(status);

	if (spoofing_detected && !_recovery_command_sent) {
		publishRecoveryCommand(plan);
		_recovery_command_sent = true;

	} else if (!spoofing_detected) {
		_recovery_command_sent = false;
	}

	return spoofing_detected;
}

int GpsSpoofingDetection::getSpoofPlan() {
	int plan = 1; 	// default to ReturnHome

	if (_gps_spoof_plan_param != PARAM_INVALID) {
		param_get(_gps_spoof_plan_param, &plan);
	}

	return plan;
}

void GpsSpoofingDetection::publishRecoveryCommand(int plan) {
	int32_t auto_sub_mode = -1;

	switch (plan) {
	case 1: // ReturnHome
		auto_sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_RTL;
		break;

	case 3: // LandImmediately
		auto_sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_LAND;
		break;

	case 0: // NoAction
	case 2: // ContinueMission
	default:
		return;
	}

	vehicle_command_s command{};
	command.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	command.param1 = 1.f; // base mode VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED
	command.param2 = static_cast<float>(PX4_CUSTOM_MAIN_MODE_AUTO);
	command.param3 = static_cast<float>(auto_sub_mode);
	command.target_system = 1;
	command.target_component = 1;
	command.source_system = 1;
	command.source_component = 1;
	command.from_external = false;
	command.timestamp = hrt_absolute_time();
	_vehicle_command_pub.publish(command);
}

float GpsSpoofingDetection::opticalFlowDistance() {
    float dt = (_optical_flow.timestamp_sample - _prev_optical_flow.timestamp_sample) / 1000000.0f;
    float dx = (_optical_flow.vel_ne_filtered[0]);
    float dy = (_optical_flow.vel_ne_filtered[1]);
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

bool GpsSpoofingDetection::SSDGOF() {
	if (abs(_total_distance_gps - (double) _total_distance_flow) > _sensitivity_threshold) {
		PX4_ERR("GPS SPOOFING DETECTED");
		return true;
	}
	return false;
}
bool GpsSpoofingDetection::CUSUM(double of_distance, double gps_distance) {
	double diff = of_distance - gps_distance;
	double baseline_diff = -0.004073260416666667;
	/* standard deviation of the difference about std_diff = 0.11901189104473191
	when k = 0,1235. we ignore about one standard deviation of normal OF-GPS difference per sample*/
	double k = 0.0015; // smaller k = faster detection, more false alarms
	/* max CUSUM reached in control flights was about 1.347, so pick a threshold slightly above this */
	double thresh = 6.819999999999999; // smaller threshold = faster detection, more false
	bool spoofing_detected = false;

	
	//LOGGING 
	const char *path = "/home/isabella-lopiano/bachelor-project/PX4-Autopilot/active_gps_spoofing_log_turn.csv";

	
	
	PX4_INFO("CUSUM diff: %f", diff);
	
	s_pos = std::max(0.0, s_pos + diff - baseline_diff - k);
	s_neg = std::max(0.0, s_neg - diff + baseline_diff - k);
	
	PX4_INFO("CUSUM s_pos: %f, s_neg: %f", s_pos, s_neg);
	
	if (s_pos > thresh || s_neg > thresh) {
		spoofing_detected = true;
	}

	FILE *fp = fopen(path, "a");

	if (fp == nullptr) {
		PX4_ERR("Failed to open %s", path);
		return false;
	}

	if (first_run) {
		fprintf(fp, "%.6f,%.6f,%.6f,%f,%.6f,%.6f\n",
			0.0,
			0.0,
			0.0,
			0.0,
			0.0,
			0.0);
			first_run = false;
	}

	fprintf(fp, "%.6f,%.6f,%.6f,%d,%.6f,%.6f\n",
		of_distance,
		gps_distance,
		diff,
		spoofing_detected ? 1 : 0,
		s_pos,
		s_neg);

	fclose(fp);


	if (spoofing_detected) {
		return true;
	}

	return false; // no spoofing detected
}

void::GpsSpoofingDetection::calculateFlowPosition() {
	if (!_flow_pos_initialised) {
		if (_initial_gps.timestamp == 0) {
			return;
		}
		_flow_lat_deg = _initial_gps.latitude_deg;
		_flow_lon_deg = _initial_gps.longitude_deg;
		_flow_pos_initialised = true;
	}

	uint64_t t = _optical_flow.timestamp_sample;
	uint64_t t_prev = _prev_optical_flow.timestamp_sample;

	if (t_prev == 0 || t <= t_prev) {
    		_prev_optical_flow = _optical_flow;
    		return;
	}

	double dt = (t - t_prev) * 1e-6;

	if (dt < 0.001 || dt > 1.0) {
		_prev_optical_flow = _optical_flow;
		return;
	}

	float vel_x = _optical_flow.vel_ne_filtered[0];
	float vel_y = _optical_flow.vel_ne_filtered[1];

	if (fabsf(vel_x) < 0.1f) {
    		vel_x = 0.0f;
	}

	if (fabsf(vel_y) < 0.1f) {
    		vel_y = 0.0f;
	}

	double dx = ((double) vel_x) * dt;
	double dy = ((double) vel_y) * dt;

	// Convert from meters to degrees
	double delta_lat = (dx / CONSTANTS_RADIUS_OF_EARTH) * (180.0 / M_PI);
	double delta_lon = (dy / (CONSTANTS_RADIUS_OF_EARTH * cos(_flow_lat_deg * M_PI / 180.0))) * (180.0 / M_PI);

	_flow_lat_deg += delta_lat;
	_flow_lon_deg += delta_lon;
}

double* GpsSpoofingDetection::getFlowPosition() {
	_flow_pos[0] = _flow_lat_deg;
    	_flow_pos[1] = _flow_lon_deg;
    	return _flow_pos;
}



void GpsSpoofingDetection::analyzeSignal() {
	if((hrt_absolute_time() - _gps.timestamp_sample) < 200000 || (hrt_absolute_time() - _optical_flow.timestamp_sample) < 200000) {
		return;
	}


	GpsSpoofingDetection::checkForOpticalFlowVelUpdate();
	GpsSpoofingDetection::checkForGPSUpdate();
	GpsSpoofingDetection::checkForMissionResultUpdate();


	_update_count++;

	if (_ofv_valid) {
		GpsSpoofingDetection::calculateFlowPosition();
	}

	if (_ofv_valid && _gps_valid) {
		float dx = _optical_flow.vel_ne_filtered[0];
		float dy = _optical_flow.vel_ne_filtered[1];
		float flow_velocity = sqrtf(dx * dx + dy * dy);
		float of_distance = 0.f;


		if (flow_velocity > 0.1f) {
			of_distance = GpsSpoofingDetection::opticalFlowDistance();
			_total_distance_flow += of_distance;
		}

		if (_gps.vel_m_s > 0.1f) {
			double gps_distance = GpsSpoofingDetection::GPSDistance(_prev_gps.longitude_deg, _prev_gps.latitude_deg, _gps.longitude_deg, _gps.latitude_deg);
			_total_distance_gps += gps_distance;

			// CUSUM test
			if ((CUSUM(of_distance, gps_distance))) {
				PX4_ERR("CUSUM GPS SPOOFING DETECTED");
				spoofing_detected = true;
			}

			PX4_INFO("gps_lat: %f, gps_lon: %f", _gps.latitude_deg, _gps.longitude_deg);
			PX4_INFO("diff_lat: %f, diff_lon: %f", _gps.latitude_deg - _flow_lat_deg, _gps.longitude_deg - _flow_lon_deg);
		}


		PX4_INFO("flow_lat: %f, flow_lon: %f", _flow_lat_deg, _flow_lon_deg);
		PX4_INFO("fx_vel: %f, fy_vel: %f, flow_dist: %f", (double) _optical_flow.vel_ne_filtered[0], (double) _optical_flow.vel_ne_filtered[1], (double) _total_distance_flow);
		PX4_INFO("vel: %f, gps_dist: %f\n\n", (double) _gps.vel_m_s, (double) _total_distance_gps);

		_ofv_valid = false;
		_gps_valid = false;

	}
}
