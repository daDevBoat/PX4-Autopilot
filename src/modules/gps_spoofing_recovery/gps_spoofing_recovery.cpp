#include "gps_spoofing_recovery.hpp"
#include <cmath>
#include <drivers/drv_hrt.h>
#include <cinttypes>

GpsSpoofingRecovery::GpsSpoofingRecovery() :
{
	
}

GpsSpoofingRecovery::~GpsSpoofingRecovery() = default;


bool GpsSpoofingRecovery::update() {
	if(hrt_absolute_time() - _optical_flow.timestamp_sample < 200000) {
		return false;
	}

	GpsSpoofingRecovery::checkForOpticalFlowVelUpdate();

}


