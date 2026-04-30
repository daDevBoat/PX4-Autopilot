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
#include <lib/geo/geo.h>
#include <lib/parameters/param.h>


class GpsSpoofingDetection
{
public:
	float threshold;

	GpsSpoofingDetection();
	~GpsSpoofingDetection();

	bool update();

	bool checkForOpticalFlowVelUpdate();
	bool checkForGPSUpdate();
	bool checkForMissionResultUpdate();
	bool CUSUM(double of_distance, double gps_distance);

	bool SSDGOF();

	double* getFlowPosition();

	float opticalFlowDistance();
	double GPSDistance(double lon_a, double lat_a, double lon_b, double lat_b);

	void calculateFlowPosition();

	void set_sensitivity(double threshold);

	bool spoofing_detected;

private:
	double _sensitivity_threshold;

	uint32_t _update_count;


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

	bool _ofv_valid{false};
	bool _gps_valid{false};

	bool _flow_pos_initialised{false};

	double s_pos{0.0};
	double s_neg{0.0};

	bool first_run = true;


	double _flow_pos[2];


	uORB::SubscriptionData<vehicle_optical_flow_vel_s> _vehicle_optical_flow_vel_sub {ORB_ID(estimator_optical_flow_vel)};
	uORB::SubscriptionData<sensor_gps_s> _vehicle_gps_position_sub {ORB_ID(sensor_gps)};
	uORB::SubscriptionData<mission_result_s> _mission_result_sub {ORB_ID(mission_result)};

	uORB::Publication<gps_spoofing_status_s> _gps_spoofing_status_pub {ORB_ID(gps_spoofing_status)};
	uORB::Publication<vehicle_command_s> _vehicle_command_pub {ORB_ID(vehicle_command)};
	param_t _gps_spoof_plan_param{PARAM_INVALID};
	bool _recovery_command_sent{false};

	int getSpoofPlan();
	void publishRecoveryCommand(int plan);

	void analyzeSignal();
};

#endif // GPS_SPOOFING_DETECTION_HPP
