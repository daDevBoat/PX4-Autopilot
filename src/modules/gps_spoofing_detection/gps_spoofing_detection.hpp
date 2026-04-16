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
#include <uORB/topics/mission_result.h>
#include <lib/geo/geo.h>


class GpsSpoofingDetection
{
public:
	float threshold;

	GpsSpoofingDetection();
	~GpsSpoofingDetection();

	/**
	 * Update with new GPS data and check for spoofing
	 * @param latitude GPS latitude
	 * @param longitude GPS longitude
	 * @param accuracy GPS accuracy in meters
	 * @return true if spoofing detected
	 */
	bool update();

	/**
	 * Check if spoofing is currently detected
	 * @return true if spoofing detected
	 */
	bool is_spoofing_detected() const;

	/**
	 * Reset the detection state
	 */
	void reset();

	bool checkForOpticalFlowEstimateUpdate();
	bool checkForGPSUpdate();
	bool checkForOpticalFlowUpdate();
	bool checkForMissionResultUpdate();


	float opticalFlowDistance(float ground_distance, float flow_x, float flow_y);
	double GPSDistance(double lon_a, double lat_a, double lon_b, double lat_b);
	float GPSDistance2(float lon_a, float lat_a, float lon_b, float lat_b);


	/**
	 * Set sensitivity threshold for detection
	 * @param threshold sensitivity value (0.0 to 1.0)
	 */
	void set_sensitivity(float threshold);

private:
	bool _spoofing_detected;
	float _sensitivity_threshold;
	double _last_latitude;
	double _last_longitude;
	float _last_accuracy;
	uint32_t _update_count;

	float _flow_x{0.f};
	float _flow_y{0.f};
	float _ground_distance{0.f};
	float _total_distance_flow{0.f};
	double _total_distance_gps{0.f};

	sensor_gps_s _gps{};
	sensor_gps_s _prev_gps{};

	vehicle_optical_flow_s _opt_flow{};

	vehicle_optical_flow_vel_s _optical_flow{};
	vehicle_optical_flow_vel_s _prev_optical_flow{};

	mission_result_s _mission_result{};

	bool _reached_height = false;
	bool _do_once = true;

	bool _of_valid{false};
	bool _ofe_valid{false};
	bool _gps_valid{false};


	uORB::SubscriptionData<vehicle_optical_flow_s> _vehicle_optical_flow_sub {ORB_ID(vehicle_optical_flow)};
	uORB::SubscriptionData<vehicle_optical_flow_vel_s> _vehicle_optical_flow_estimate_sub {ORB_ID(estimator_optical_flow_vel)};
	uORB::SubscriptionData<sensor_gps_s> _vehicle_gps_position_sub {ORB_ID(sensor_gps)};
	uORB::SubscriptionData<mission_result_s> _mission_result_sub {ORB_ID(mission_result)};

	/**
	 * Analyze GPS signal for anomalies
	 * @return true if anomalies detected
	 */
	bool analyzeSignal();
};

#endif // GPS_SPOOFING_DETECTION_HPP
