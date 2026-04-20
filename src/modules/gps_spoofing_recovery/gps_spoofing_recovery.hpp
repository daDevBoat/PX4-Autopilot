#ifndef GPS_SPOOFING_RECOVERY_HPP
#define GPS_SPOOFING_RECOVERY_HPP

#include <stdint.h>
#include <stdbool.h>

#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/vehicle_optical_flow_vel.h>
#include <uORB/topics/vehicle_optical_flow.h>
#include <uORB/topics/sensor_gps.h>

#include "gps_spoofing_detection.hpp"



class GpsSpoofingRecovery
{
public:

	GpsSpoofingRecovery();
	~GpsSpoofingRecovery();

	bool update();


private:
	bool _spoofing_detected;
	double _sensitivity_threshold;

	uint32_t _update_count;


	sensor_gps_s _inital_gps{};

	vehicle_optical_flow_vel_s _optical_flow{};
	vehicle_optical_flow_vel_s _prev_optical_flow{};

	GpsSpoofingDetection _gps_spoofing_detetction{};

	bool _ofv_valid{false};
	bool _gps_valid{false};



	uORB::SubscriptionData<vehicle_optical_flow_vel_s> _vehicle_optical_flow_vel_sub {ORB_ID(estimator_optical_flow_vel)};
	uORB::SubscriptionData<sensor_gps_s> _vehicle_gps_position_sub {ORB_ID(sensor_gps)};

};

#endif // GPS_SPOOFING_RECOVERY_HPP
