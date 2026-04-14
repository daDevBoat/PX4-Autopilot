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

	float opticalFlowDistance(float ground_distance, float flow_x, float flow_y);

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
	float _gps_vel_n{0.f};
	float _gps_vel_e{0.f};



	bool _of_valid{false};
	bool _ofe_valid{false};
	bool _gps_valid{false};


	uORB::SubscriptionData<vehicle_optical_flow_s> _vehicle_optical_flow_sub {ORB_ID(vehicle_optical_flow)};
	uORB::SubscriptionData<vehicle_optical_flow_vel_s> _vehicle_optical_flow_estimate_sub {ORB_ID(estimator_optical_flow_vel)};
	uORB::SubscriptionData<sensor_gps_s> _vehicle_gps_position_sub {ORB_ID(sensor_gps)};


	/**
	 * Analyze GPS signal for anomalies
	 * @return true if anomalies detected
	 */
	bool analyzeSignal();
};

#endif // GPS_SPOOFING_DETECTION_HPP
