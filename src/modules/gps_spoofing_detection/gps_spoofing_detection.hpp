#ifndef GPS_SPOOFING_DETECTION_HPP
#define GPS_SPOOFING_DETECTION_HPP

#include <stdint.h>
#include <stdbool.h>

#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/vehicle_optical_flow_vel.h>
#include <uORB/topics/vehicle_optical_flow.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/gps_spoofing_status.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_imu.h>
#include <uORB/topics/estimator_innovations.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/estimator_status.h>
#include <lib/geo/geo.h>
#include <lib/parameters/param.h>

#include <iostream>
#include <fstream>

class GpsSpoofingDetection
{
public:
	GpsSpoofingDetection();
	~GpsSpoofingDetection();

	bool update();

	bool checkForOpticalFlowVelUpdate();
	bool checkForGPSUpdate();
	bool checkForMissionResultUpdate();
	void calculateGyroDeltaMagnitude();

	bool CUSUM(double of_distance, double gps_distance);
	bool AdaptiveCUSUM(double of_distance, double gps_distance, double threshold);
	bool SSDGOF();
	void CUSUM_GYRO(float mag, float prev_mag);

	double* getFlowPosition();

	float opticalFlowDistance();
	double GPSDistance(double lon_a, double lat_a, double lon_b, double lat_b);

	void calculateFlowPosition();

	void set_sensitivity(double threshold);

	bool spoofing_detected;

private:
	double _sensitivity_threshold;
	int _update_count;
	float _adapt_thresh_scalar;
	int _hits_threshold;

	float _total_distance_flow{0.f};
	double _total_distance_gps{0.f};

	sensor_gps_s _initial_gps{};

	double _flow_lon_deg{0.0};
	double _flow_lat_deg{0.0};


	sensor_gps_s _gps{};
	sensor_gps_s _prev_gps{};

	vehicle_optical_flow_vel_s _optical_flow{};
	vehicle_optical_flow_vel_s _prev_optical_flow{};

	mission_result_s _mission_result{};

	vehicle_imu_s _imu{};
	vehicle_imu_s _prev_imu{};

	estimator_innovations_s _estimator_innovations{};
	estimator_status_s _estimator_status{};
	vehicle_attitude_s _vehicle_attitude{};
	vehicle_attitude_s _prev_vehicle_attitude{};

	int _data_counter = 0;
	float _roll_sum = 0.f;
	float _pitch_sum = 0.f;
	float _yaw_sum = 0.f;
	double _diff_sum = 0.f;

	bool _ofv_valid{false};
	bool _gps_valid{false};

	bool _flow_pos_initialised{false};

	double _s_pos{0.0};
	double _s_neg{0.0};
	double _adapt_s_pos{0.0};
	double _adapt_s_neg{0.0};

	double _flow_pos[2];

	int _consecutive_spoofing_hits = 0;
	int _adaptive_consecutive_spoofing_hits = 0;

	float _gyro_magnitude = 0.f;
	float _prev_gyro_magnitude = 0.f;
	float _s_mag = 0.f;
	bool _gyro_cusum_initalised = false;

	std::ofstream _output_file;


	uORB::SubscriptionData<vehicle_optical_flow_vel_s> _vehicle_optical_flow_vel_sub {ORB_ID(estimator_optical_flow_vel)};
	uORB::SubscriptionData<sensor_gps_s> _vehicle_gps_position_sub {ORB_ID(sensor_gps)};
	uORB::SubscriptionData<mission_result_s> _mission_result_sub {ORB_ID(mission_result)};
	uORB::SubscriptionData<vehicle_imu_s> _vehicle_imu_sub {ORB_ID(vehicle_imu)};
	uORB::SubscriptionData<estimator_innovations_s> _estimator_innovations_sub {ORB_ID(estimator_innovations)};
	uORB::SubscriptionData<vehicle_attitude_s> _vehicle_attitude_sub {ORB_ID(vehicle_attitude)};
	uORB::SubscriptionData<estimator_status_s> _estimator_status_sub {ORB_ID(estimator_status)};

	uORB::Publication<gps_spoofing_status_s> _gps_spoofing_status_pub {ORB_ID(gps_spoofing_status)};
	uORB::Publication<vehicle_command_s> _vehicle_command_pub {ORB_ID(vehicle_command)};
	param_t _gps_spoof_plan_param{PARAM_INVALID};
	bool _recovery_command_sent{false};

	int getSpoofPlan();
	void publishRecoveryCommand(int plan);

	void analyzeSignal();
};

#endif // GPS_SPOOFING_DETECTION_HPP
