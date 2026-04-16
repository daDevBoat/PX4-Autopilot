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

	bool checkForOpticalFlowVelUpdate();
	bool checkForGPSUpdate();
	bool checkForMissionResultUpdate();

	bool SSDGOF();


	float opticalFlowDistance();
	double GPSDistance(double lon_a, double lat_a, double lon_b, double lat_b);


	void set_sensitivity(double threshold);

private:
	bool _spoofing_detected;
	double _sensitivity_threshold;

	uint32_t _update_count;


	float _total_distance_flow{0.f};
	double _total_distance_gps{0.f};

	sensor_gps_s _gps{};
	sensor_gps_s _prev_gps{};

	vehicle_optical_flow_vel_s _optical_flow{};
	vehicle_optical_flow_vel_s _prev_optical_flow{};

	mission_result_s _mission_result{};

	bool _ofv_valid{false};
	bool _gps_valid{false};

	uORB::SubscriptionData<vehicle_optical_flow_vel_s> _vehicle_optical_flow_vel_sub {ORB_ID(estimator_optical_flow_vel)};
	uORB::SubscriptionData<sensor_gps_s> _vehicle_gps_position_sub {ORB_ID(sensor_gps)};
	uORB::SubscriptionData<mission_result_s> _mission_result_sub {ORB_ID(mission_result)};

	/**
	 * Analyze GPS signal for anomalies
	 * @return true if anomalies detected
	 */
	bool analyzeSignal();
};

#endif // GPS_SPOOFING_DETECTION_HPP
