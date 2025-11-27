#pragma once

#include <cstdint>

namespace platform
{
// Triggers a short vibration (haptic feedback) on the device, if supported.
// The |durationMs| parameter specifies the vibration length in milliseconds.
// On platforms where vibration is not available this function is a no-op.
void Vibrate(uint32_t durationMs = 50);

// Triggers multiple vibrations with delays between them.
// |durations| - array of vibration durations in milliseconds
// |delays| - array of delays between vibrations in milliseconds (should be same size as durations)
// |count| - number of vibrations to perform
void VibratePattern(uint32_t const * durations, uint32_t const * delays, size_t count);
}