#ifndef GPS_SPOOFING_DETECTION_HPP
#define GPS_SPOOFING_DETECTION_HPP

#include <stdint.h>
#include <stdbool.h>

class GpsSpoofingDetection
{
public:
	float threshold;

	GpsSpoofingDetection();
	~GpsSpoofingDetection();

	/**
	 * Initialize the spoofing detection module
	 * @return true if initialization successful
	 */
	bool init();

	/**
	 * Update with new GPS data and check for spoofing
	 * @param latitude GPS latitude
	 * @param longitude GPS longitude
	 * @param accuracy GPS accuracy in meters
	 * @return true if spoofing detected
	 */
	bool update(double latitude, double longitude, float accuracy);

	/**
	 * Check if spoofing is currently detected
	 * @return true if spoofing detected
	 */
	bool is_spoofing_detected() const;

	/**
	 * Reset the detection state
	 */
	void reset();

	/**
	 * Set sensitivity threshold for detection
	 * @param threshold sensitivity value (0.0 to 1.0)
	 */
	void set_sensitivity(float threshold);

private:
	bool _initialized;
	bool _spoofing_detected;
	float _sensitivity_threshold;
	double _last_latitude;
	double _last_longitude;
	float _last_accuracy;
	uint32_t _update_count;

	/**
	 * Analyze GPS signal for anomalies
	 * @return true if anomalies detected
	 */
	bool analyze_signal();
};

#endif // GPS_SPOOFING_DETECTION_HPP
