#ifndef INC_HISTO_FAULT_INJECT_H_
#define INC_HISTO_FAULT_INJECT_H_

#include <stdbool.h>
#include <stdint.h>

#define HISTO_FAULT_CAMERA_COUNT 8u

typedef enum {
	HISTO_FAULT_NONE = 0,
	HISTO_FAULT_FID_SINGLE,
	HISTO_FAULT_FID_MULTI,
	HISTO_FAULT_TIMESTAMP_FREEZE,
	HISTO_FAULT_PACKET_DROP,
	HISTO_FAULT_INVALID_SELECTION,
	HISTO_FAULT_DISABLED_RELEASE,
} histo_fault_mode_t;

typedef struct {
	histo_fault_mode_t mode;
	uint32_t frame_index;
	uint32_t captured_timestamp_ms;
	uint32_t wire_timestamp_ms;
	uint8_t corrupt_camera_mask;
	bool drop_packet;
	bool applied;
} histo_fault_plan_t;

typedef struct {
	histo_fault_mode_t mode;
	uint32_t frame_index;
	uint32_t previous_timestamp_ms;
	uint32_t freeze_timestamp_ms;
	uint8_t freeze_packets_left;
	bool has_previous_timestamp;
	bool injected_once;
} histo_fault_injector_t;

void histo_fault_injector_reset(histo_fault_injector_t *injector,
		uint32_t debug_flags);
histo_fault_plan_t histo_fault_injector_plan(
		histo_fault_injector_t *injector,
		uint8_t enabled_camera_mask,
		const uint8_t raw_frame_ids[HISTO_FAULT_CAMERA_COUNT],
		uint32_t captured_timestamp_ms);
const char *histo_fault_mode_name(histo_fault_mode_t mode);

#endif /* INC_HISTO_FAULT_INJECT_H_ */
