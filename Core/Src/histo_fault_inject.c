#include "histo_fault_inject.h"

#include "common.h"

#include <string.h>

#define HIL_FID_EARLIEST_FRAME 32u
#define HIL_TIMING_TRIGGER_FRAME 80u
#define HIL_TIMESTAMP_FREEZE_PACKETS 3u

static histo_fault_mode_t mode_from_flags(uint32_t debug_flags) {
	switch (debug_flags & DEBUG_FLAG_HIL_FAULT_MASK) {
	case 0u:
		return HISTO_FAULT_NONE;
	case DEBUG_FLAG_FID_CORRUPT:
		return HISTO_FAULT_FID_SINGLE;
	case DEBUG_FLAG_FID_CORRUPT_MULTI:
		return HISTO_FAULT_FID_MULTI;
	case DEBUG_FLAG_TIMESTAMP_FREEZE:
		return HISTO_FAULT_TIMESTAMP_FREEZE;
	case DEBUG_FLAG_HISTO_DROP_ONCE:
		return HISTO_FAULT_PACKET_DROP;
	default:
		return HISTO_FAULT_INVALID_SELECTION;
	}
}

void histo_fault_injector_reset(histo_fault_injector_t *injector,
		uint32_t debug_flags) {
	memset(injector, 0, sizeof(*injector));
	injector->mode = mode_from_flags(debug_flags);
#if !defined(DEBUG)
	if (injector->mode != HISTO_FAULT_NONE) {
		injector->mode = HISTO_FAULT_DISABLED_RELEASE;
	}
#endif
}

#if defined(DEBUG)
static uint8_t camera_count(uint8_t mask) {
	uint8_t count = 0u;
	while (mask != 0u) {
		count += mask & 1u;
		mask >>= 1;
	}
	return count;
}

static bool packet_ids_match(uint8_t enabled_camera_mask,
		const uint8_t raw_frame_ids[HISTO_FAULT_CAMERA_COUNT],
		uint8_t *common_raw_id) {
	bool found = false;
	for (uint8_t camera = 0u; camera < HISTO_FAULT_CAMERA_COUNT; ++camera) {
		if ((enabled_camera_mask & (uint8_t)(1u << camera)) == 0u) {
			continue;
		}
		if (!found) {
			*common_raw_id = raw_frame_ids[camera];
			found = true;
		} else if (raw_frame_ids[camera] != *common_raw_id) {
			return false;
		}
	}
	return found;
}

static uint8_t highest_enabled_cameras(uint8_t enabled_camera_mask,
		uint8_t wanted) {
	uint8_t selected = 0u;
	for (int8_t camera = (int8_t)HISTO_FAULT_CAMERA_COUNT - 1;
			camera >= 0 && wanted > 0u; --camera) {
		uint8_t bit = (uint8_t)(1u << (uint8_t)camera);
		if ((enabled_camera_mask & bit) != 0u) {
			selected |= bit;
			--wanted;
		}
	}
	return selected;
}
#endif

histo_fault_plan_t histo_fault_injector_plan(
		histo_fault_injector_t *injector,
		uint8_t enabled_camera_mask,
		const uint8_t raw_frame_ids[HISTO_FAULT_CAMERA_COUNT],
		uint32_t captured_timestamp_ms) {
	histo_fault_plan_t plan = {
		.mode = injector->mode,
		.frame_index = ++injector->frame_index,
		.captured_timestamp_ms = captured_timestamp_ms,
		.wire_timestamp_ms = captured_timestamp_ms,
		.corrupt_camera_mask = 0u,
		.drop_packet = false,
		.applied = false,
	};

#if defined(DEBUG)
	if ((injector->mode == HISTO_FAULT_FID_SINGLE ||
			injector->mode == HISTO_FAULT_FID_MULTI) &&
			!injector->injected_once &&
			plan.frame_index >= HIL_FID_EARLIEST_FRAME) {
		uint8_t required_cameras =
			(injector->mode == HISTO_FAULT_FID_SINGLE) ? 3u : 4u;
		uint8_t victim_count =
			(injector->mode == HISTO_FAULT_FID_SINGLE) ? 1u : 2u;
		uint8_t common_raw_id = 0u;
		if (camera_count(enabled_camera_mask) >= required_cameras &&
				packet_ids_match(enabled_camera_mask, raw_frame_ids,
					&common_raw_id) &&
				(common_raw_id & 0xC0u) == 0xC0u) {
			plan.corrupt_camera_mask = highest_enabled_cameras(
				enabled_camera_mask, victim_count);
			plan.applied = true;
			injector->injected_once = true;
		}
	} else if (injector->mode == HISTO_FAULT_TIMESTAMP_FREEZE) {
		if (!injector->injected_once && injector->has_previous_timestamp &&
				plan.frame_index >= HIL_TIMING_TRIGGER_FRAME) {
			injector->freeze_timestamp_ms = injector->previous_timestamp_ms;
			injector->freeze_packets_left = HIL_TIMESTAMP_FREEZE_PACKETS;
			injector->injected_once = true;
		}
		if (injector->freeze_packets_left > 0u) {
			plan.wire_timestamp_ms = injector->freeze_timestamp_ms;
			plan.applied = true;
			--injector->freeze_packets_left;
		}
	} else if (injector->mode == HISTO_FAULT_PACKET_DROP &&
			!injector->injected_once &&
			plan.frame_index >= HIL_TIMING_TRIGGER_FRAME) {
		plan.drop_packet = true;
		plan.applied = true;
		injector->injected_once = true;
	}
#else
	(void)enabled_camera_mask;
	(void)raw_frame_ids;
#endif

	injector->previous_timestamp_ms = captured_timestamp_ms;
	injector->has_previous_timestamp = true;
	return plan;
}

const char *histo_fault_mode_name(histo_fault_mode_t mode) {
	switch (mode) {
	case HISTO_FAULT_NONE:
		return "none";
	case HISTO_FAULT_FID_SINGLE:
		return "fid_single";
	case HISTO_FAULT_FID_MULTI:
		return "fid_multi";
	case HISTO_FAULT_TIMESTAMP_FREEZE:
		return "timestamp_freeze";
	case HISTO_FAULT_PACKET_DROP:
		return "packet_drop";
	case HISTO_FAULT_INVALID_SELECTION:
		return "invalid_selection";
	case HISTO_FAULT_DISABLED_RELEASE:
		return "disabled_release";
	default:
		return "unknown";
	}
}
