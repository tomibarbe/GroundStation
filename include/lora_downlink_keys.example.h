// Copy to lora_downlink_keys.h (gitignored). Replace the 32 bytes with the
// unique key provisioned in each collar. All-zero entries are refused.
#pragma once
#define LORA_DOWNLINK_KEYS { \
  { "COW-012", {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} } \
}
