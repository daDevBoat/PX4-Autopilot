#ifndef GPS_SPOOFING_RECOVERY_HPP
#define GPS_SPOOFING_RECOVERY_HPP

#include <stdint.h>
#include <stdbool.h>

#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/aux_global_position.h>

#include <gps_spoofing_detection/gps_spoofing_detection.hpp>


class GpsSpoofingRecovery
{
public:

	GpsSpoofingRecovery(GpsSpoofingDetection &gps_spoofing_detection);
	~GpsSpoofingRecovery();

	bool update();
	void publishAuxGlobalPosition();


private:
	GpsSpoofingDetection &_gps_spoofing_detection;
	bool _gps_disabled{false};

	uORB::PublicationMulti<aux_global_position_s> _aux_global_position_pub{ORB_ID(aux_global_position)};
};

#endif // GPS_SPOOFING_RECOVERY_HPP
