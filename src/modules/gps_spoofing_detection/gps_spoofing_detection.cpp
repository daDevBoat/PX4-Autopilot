
#include "gps_spoofing_detection.hpp"
#include <cmath>
#include <drivers/drv_hrt.h>
#include <cinttypes>

#include <iostream>
#include <fstream>
#include <ctime>
#include <px4_platform_common/log.h>

#include <lib/parameters/param.h>
#include <modules/commander/px4_custom_mode.h>


GpsSpoofingDetection::GpsSpoofingDetection() :
	spoofing_detected(false),
	_sensitivity_threshold(10.0),
	_update_count(0),
	_adapt_thresh_scalar(1.5f),
	_hits_threshold(5)
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

	time_t now = time(nullptr);

    	struct tm utc_time {};
    	gmtime_r(&now, &utc_time);   // PX4 time is usually treated as UTC

    	char buffer[32];
    	strftime(buffer, sizeof(buffer), "%Y_%m_%d-%H_%M_%S", &utc_time);

    	PX4_INFO("Current date/time: %s", buffer);

	std::string log_filename = "/home/dadevboat/PX4_research/PX4-Autopilot/src/modules/gps_spoofing_detection/logs/"
    + std::string(buffer) + ".txt";
	_output_file.open(log_filename.c_str(), std::ios::out | std::ios::app);

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

void GpsSpoofingDetection::calculateGyroDeltaMagnitude() {

	if (_vehicle_attitude_sub.update()) {
		_prev_vehicle_attitude = _vehicle_attitude;
		_vehicle_attitude = _vehicle_attitude_sub.get();
	}

	matrix::Quatf q(_vehicle_attitude.q);
	matrix::Eulerf euler(q);

	float roll = euler.phi();
	float pitch = euler.theta();
	float yaw = euler.psi();

	_prev_gyro_magnitude = _gyro_magnitude;
	_gyro_magnitude = sqrt(roll * roll + pitch * pitch + yaw * yaw);

	if (_gyro_cusum_initalised) {
		CUSUM_GYRO(_gyro_magnitude, _prev_gyro_magnitude);
		//_output_file << _s_mag << "\t" << std::max(_s_pos, _s_neg) << "\n";
		//_output_file.flush();
	} else {
		_gyro_cusum_initalised = true;
	}
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
	double baseline_diff = -0.0041;
	double k = 0.01306;
	double thresh = 2.2;

	PX4_INFO("CUSUM diff: %f", diff);

	_s_pos = std::max(0.0, _s_pos + diff - baseline_diff - k);
	_s_neg = std::max(0.0, _s_neg - diff + baseline_diff - k);

	PX4_INFO("CUSUM s_pos: %f, s_neg: %f", _s_pos, _s_neg);

	if (_s_pos > thresh || _s_neg > thresh) {
		return true; // spoofing detected
	}
	return false; // no spoofing detected
}

void GpsSpoofingDetection::CUSUM_GYRO(float mag, float prev_mag) {
	float diff = abs(mag - prev_mag);
	float baseline_diff = 0.f;
	float k = 0.025;


	_s_mag = std::max(0.f, _s_mag + diff - baseline_diff - k);
	PX4_INFO("CUSUM GYRO    diff: %f    S_mag: %f", (double) diff, (double) _s_mag);
}

bool GpsSpoofingDetection::AdaptiveCUSUM(double of_distance, double gps_distance, double thresh) {
	double diff = of_distance - gps_distance;
	double k = 0.01306; // smaller k = faster detection, more false alarms
	double baseline_diff = -0.0041;
	thresh = std::max(1.0, thresh);

	_adapt_s_pos = std::max(0.0, _adapt_s_pos + diff - baseline_diff - k);
	_adapt_s_neg = std::max(0.0, _adapt_s_neg - diff + baseline_diff - k);

	if (_s_pos > thresh || _s_neg > thresh) {
		return true; // spoofing detected
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




	if (_ofv_valid && _gps_valid) {
		float dx = _optical_flow.vel_ne_filtered[0];
		float dy = _optical_flow.vel_ne_filtered[1];
		float flow_velocity = sqrtf(dx * dx + dy * dy);
		float of_distance = 0.f;
		double gps_distance = 0.0;

		if (!_output_file.is_open()) {
			PX4_ERR("Error opening output file!");
		}

		if (_gps.vel_m_s > 0.1f) {
			gps_distance = GpsSpoofingDetection::GPSDistance(_prev_gps.longitude_deg, _prev_gps.latitude_deg, _gps.longitude_deg, _gps.latitude_deg);
			_total_distance_gps += gps_distance;


			/*
			matrix::Quatf prev_q(_prev_vehicle_attitude.q);
  			matrix::Eulerf prev_euler(prev_q);

  			float prev_roll = prev_euler.phi();
  			float prev_pitch = prev_euler.theta();
  			float prev_yaw = prev_euler.psi();

			  double diff = abs((double) of_distance - gps_distance);

			  float roll_diff = abs(roll - prev_roll);
			  float pitch_diff = abs(pitch - prev_pitch);
			  float yaw_diff = abs(yaw - prev_yaw);

			  _diff_sum += diff;
			  _roll_sum += roll_diff;
			  _pitch_sum += pitch_diff;
			  _yaw_sum += yaw_diff;


			  if (_data_counter == 10) {
				//_output_file << sqrt(roll_diff * roll_diff + pitch_diff * pitch_diff + yaw_diff * yaw_diff) << "\t" << diff << "\n";
				//_output_file << sqrt(_roll_sum * _roll_sum + _pitch_sum * _pitch_sum + _yaw_sum * _yaw_sum) << "\t" << _diff_sum << "\n";
				//_output_file << _roll_sum << "\t" << _diff_sum << "\n";
				_data_counter = 0;
				_roll_sum = 0.f;
				_pitch_sum = 0.f;
				_yaw_sum = 0.f;
				_diff_sum = 0.f;
				}

				_output_file << sqrt(roll * roll + pitch * pitch + yaw * yaw) << "\t" << sqrt(prev_roll * prev_roll + prev_pitch * prev_pitch + prev_yaw * prev_yaw) << "\n";
				_output_file.flush();
				*/

			//PX4_INFO("Errors - ang: %f, vel: %f, pos: %f", (double) _estimator_status.output_tracking_error[0], (double) _estimator_status.output_tracking_error[1], (double) _estimator_status.output_tracking_error[2]);

			//PX4_INFO("IMU   x: %f, y: %f, z: %f", (double) _imu.delta_angle[0] * 1000,  (double) _imu.delta_angle[1] * 1000, (double) _imu.delta_angle[2] * 1000);
			//PX4_INFO("Attitude: roll: %f, pitch: %f, yaw: %f", (double) roll, (double) pitch, (double) yaw);
			//PX4_INFO("gps_lat: %f, gps_lon: %f", _gps.latitude_deg, _gps.longitude_deg);
			//PX4_INFO("diff_lat: %f, diff_lon: %f", _gps.latitude_deg - _flow_lat_deg, _gps.longitude_deg - _flow_lon_deg);
		}


		if (flow_velocity > 0.1f) {
			of_distance = GpsSpoofingDetection::opticalFlowDistance();
			_total_distance_flow += of_distance;

			if ((CUSUM((double) of_distance, gps_distance))) {
				_consecutive_spoofing_hits++;
				if (_consecutive_spoofing_hits == 5) {
					PX4_ERR("CUSUM GPS SPOOFING DETECTED");
					//spoofing_detected = true;
				}
			} else {
				_consecutive_spoofing_hits = 0;
			}

			calculateGyroDeltaMagnitude();

			if (AdaptiveCUSUM((double) of_distance, gps_distance, (double) (_s_mag * _adapt_thresh_scalar))) {
				_adaptive_consecutive_spoofing_hits++;
				if (_adaptive_consecutive_spoofing_hits == 5) {
					PX4_ERR("ADAPTIVE CUSUM GPS SPOOFING DETECTED");
					//spoofing_detected = true;
				}
			} else {
				_adaptive_consecutive_spoofing_hits = 0;
			}

			_output_file << _s_mag << "\t" << (double) _total_distance_flow - _total_distance_gps << "\n";
			_output_file.flush();


		}

		PX4_INFO("gps_dist: %f opt_flow_dist: %f\n\n", (double) _total_distance_gps, (double) _total_distance_flow);

		_ofv_valid = false;
		_gps_valid = false;

		_update_count++;
	}
}
