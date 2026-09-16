#include "ViperX16.h"

#include <Preferences.h>

#include "RobotConfig.h"

namespace {
	constexpr uint32_t kCalibrationMagic = 0x58313643UL;  // "X16C"
	constexpr uint16_t kCalibrationVersion = 1;

	struct CalibrationBlob {
		uint32_t magic;
		uint16_t version;
		uint16_t crc;
		uint16_t minimum[kViperSensorCount];
		uint16_t maximum[kViperSensorCount];
	};

	uint16_t crc16(const uint8_t* data, size_t length) {
		uint16_t crc = 0xFFFF;

		for (size_t i = 0; i < length; ++i) {
			crc ^= static_cast<uint16_t>(data[i]) << 8;
			for (uint8_t bit = 0; bit < 8; ++bit) {
			crc = (crc & 0x8000U) ? static_cast<uint16_t>((crc << 1) ^ 0x1021U)
									: static_cast<uint16_t>(crc << 1);
			}
		}
		
		return crc;
	}
}  // namespace

void ViperX16::begin() {
	for (uint8_t pin : cfg::kMuxSelectPins) {
		pinMode(pin, OUTPUT);
		digitalWrite(pin, LOW);
	}
	pinMode(cfg::kViperAdcPin, INPUT);
	analogReadResolution(12);
	analogSetPinAttenuation(cfg::kViperAdcPin, ADC_11db);
	loadCalibration();
}

uint16_t ViperX16::readChannel(uint8_t channel) const {
	for (uint8_t bit = 0; bit < 4; ++bit) {
		digitalWrite(cfg::kMuxSelectPins[bit], (channel >> bit) & 0x01U);
	}

	// A mux holds charge from the prior input; discard the first conversion so it
	// cannot bias the next sensor, especially when black and white are adjacent.
	delayMicroseconds(cfg::kMuxSettleUs);
	analogRead(cfg::kViperAdcPin);

	uint32_t sum = 0;
	for (uint8_t sample = 0; sample < cfg::kSamplesPerSensor; ++sample) {
		sum += analogRead(cfg::kViperAdcPin);
	}
	return static_cast<uint16_t>(sum / cfg::kSamplesPerSensor);
}

uint16_t ViperX16::normalise(uint8_t channel, uint16_t raw) const {
	const uint16_t low = minimum_[channel];
	const uint16_t high = maximum_[channel];

	if (high <= low + cfg::kMinimumCalibrationSpan) {
		return 0;
	}

	const int32_t scaled = constrain(
		(static_cast<int32_t>(raw) - low) * 1000L / static_cast<int32_t>(high - low),
		0L, 1000L);
	
	return cfg::kLineIsDark ? static_cast<uint16_t>(1000 - scaled)
							: static_cast<uint16_t>(scaled);
}

void ViperX16::read(SensorFrame& frame) {
	int32_t weightedSum = 0;
	uint32_t lineSum = 0;
	uint32_t denominator = 0;
	frame.activeCount = 0;

	for (uint8_t i = 0; i < kViperSensorCount; ++i) {
		frame.raw[i] = readChannel(i);
			const uint16_t normalised = normalise(i, frame.raw[i]);
		
		if (!filterInitialised_) {
			filtered_[i] = normalised;
		} else {
			filtered_[i] += cfg::kSensorFilterAlpha * (normalised - filtered_[i]);
		}

		frame.line[i] = static_cast<uint16_t>(constrain(
			static_cast<int32_t>(filtered_[i] + 0.5F), 0L, 1000L));

		lineSum += frame.line[i];
		denominator += frame.line[i];

		const int16_t weight = static_cast<int16_t>((static_cast<int16_t>(i) * 1000) -
													kPositionLimit);
		
		weightedSum += static_cast<int32_t>(frame.line[i]) * weight;

		if (frame.line[i] >= cfg::kActiveThreshold) {
			++frame.activeCount;
		}
	}

	filterInitialised_ = true;
	frame.strength = static_cast<uint16_t>(lineSum / kViperSensorCount);
	frame.position = denominator == 0
						? 0
						: static_cast<int16_t>(constrain(
								weightedSum / static_cast<int32_t>(denominator),
								-static_cast<int32_t>(kPositionLimit),
								static_cast<int32_t>(kPositionLimit)));
	
	frame.lineVisible = frame.activeCount > 0 &&
						frame.strength >= cfg::kLineVisibleStrength;
	
	frame.leftEdge = frame.line[0] >= cfg::kBranchThreshold;
	frame.rightEdge = frame.line[kViperSensorCount - 1] >= cfg::kBranchThreshold;
	frame.wide = frame.activeCount >= cfg::kJunctionActiveSensors;
}

void ViperX16::startCalibration() {
	calibrationValid_ = false;
	filterInitialised_ = false;
	for (uint8_t i = 0; i < kViperSensorCount; ++i) {
		minimum_[i] = 4095;
		maximum_[i] = 0;
	}
}

void ViperX16::sampleCalibration() {
	for (uint8_t i = 0; i < kViperSensorCount; ++i) {
		const uint16_t raw = readChannel(i);
		minimum_[i] = min(minimum_[i], raw);
		maximum_[i] = max(maximum_[i], raw);
	}
}

bool ViperX16::rangesAreValid() const {
	for (uint8_t i = 0; i < kViperSensorCount; ++i) {
		if (maximum_[i] <= minimum_[i] + cfg::kMinimumCalibrationSpan) {
			return false;
		}
	}
	
	return true;
}

bool ViperX16::finishCalibration() {
	calibrationValid_ = rangesAreValid();
	filterInitialised_ = false;
	return calibrationValid_ && saveCalibration();
}

bool ViperX16::saveCalibration() const {
	if (!calibrationValid_) {
		return false;
	}

	CalibrationBlob blob{};
	blob.magic = kCalibrationMagic;
	blob.version = kCalibrationVersion;

	memcpy(blob.minimum, minimum_, sizeof(minimum_));
	memcpy(blob.maximum, maximum_, sizeof(maximum_));

	blob.crc = 0;
	blob.crc = crc16(reinterpret_cast<const uint8_t*>(&blob), sizeof(blob));

	Preferences preferences;
	if (!preferences.begin("linebot", false)) {
		return false;
	}

	const size_t written = preferences.putBytes("viper-cal", &blob, sizeof(blob));
	preferences.end();

	return written == sizeof(blob);
}

bool ViperX16::loadCalibration() {
	Preferences preferences;
	if (!preferences.begin("linebot", true)) {
		return false;
	}

	if (preferences.getBytesLength("viper-cal") != sizeof(CalibrationBlob)) {
		preferences.end();
		return false;
	}

	CalibrationBlob blob{};
	const size_t read = preferences.getBytes("viper-cal", &blob, sizeof(blob));
	preferences.end();
	const uint16_t expectedCrc = blob.crc;
	blob.crc = 0;

	if (read != sizeof(blob) || blob.magic != kCalibrationMagic ||
		blob.version != kCalibrationVersion ||
		crc16(reinterpret_cast<const uint8_t*>(&blob), sizeof(blob)) != expectedCrc) {
		return false;
	}

	memcpy(minimum_, blob.minimum, sizeof(minimum_));
	memcpy(maximum_, blob.maximum, sizeof(maximum_));

	calibrationValid_ = rangesAreValid();
	filterInitialised_ = false;
	
	return calibrationValid_;
}
