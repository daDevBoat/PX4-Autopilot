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

	GpsSpoofingRecovery(const GpsSpoofingDetection &gps_spoofing_detection);
	~GpsSpoofingRecovery();

	bool update();


private:
	const GpsSpoofingDetection &_gps_spoofing_detection;
	bool _gps_disabled{false};

};

#endif // GPS_SPOOFING_RECOVERY_HPP
