# openmotion-bl Bootloader Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Port the OpenLIFU console bootloader to a new private repo `OpenwaterHealth/openmotion-bl`, building a USB-DFU secure bootloader for STM32H743 with two static targets (sensor, console), plus the minimal app-side changes in `openmotion-sensor-fw` and `openmotion-console-fw` needed to boot through it.

**Architecture:** Single repo, shared `Core/` boot logic + crypto, per-target `Targets/<board>/` for clock/GPIO/USB/linker. The bootloader lives in flash sector 0 (`0x08000000`, 128 KB), metadata in sector 1 (`0x08020000`), application at `0x08040000`. Boot decision is a faithful port of OpenLIFU's flow (vector sanity → metadata v3 → CRC32 → HMAC trust-tag → backup-reg auth cache → ECDSA P-256), adapted from Cortex-M0 (SRAM vector remap) to Cortex-M7 (`SCB->VTOR`). HAL/USB scaffolding is grafted from the existing H743 firmware rather than regenerated from CubeMX.

**Tech Stack:** C11, STM32H7 HAL, ST USB Device DFU class, uECC + bundled SHA-256, arm-none-eabi-gcc + CMake + Ninja, Python host tooling (`cryptography`, `pyusb`), GitHub Actions in `stm32-build-env` Docker.

**Reference checkout:** `C:\Users\ethan\Projects\archive\openlifu-console-bl-ref` (OpenLIFU BL, STM32F072). Referred to below as `$REF`.

**New repo working copy:** `C:\Users\ethan\Projects\openmotion-bl` (referred to as `$BL`).

---

## Conventions for this plan

- Files copied **verbatim** from a reference repo are specified by a copy command, not a code paste (HAL drivers, ST USB middleware, `sha256.c`, `uECC.c`, startup `.s`). Where a copied file needs edits, the exact edit is shown.
- "Build sensor" = `cmake --preset sensor-Release && cmake --build build/sensor-Release`. "Build console" likewise. Both must produce `.elf/.hex/.bin` with no errors.
- Native host unit tests for the portable C logic build with the system `gcc` (not the ARM toolchain) so boot-state and metadata logic can be tested without hardware.
- Commit after every task. Commit prefixes per repo convention: `feat:`/`fix:`/`test:`/`docs:`/`chore:`.
- **HARD GATE:** Task 33 (first hardware flash) must not run until the user explicitly confirms, per the design.

---

## File structure (what gets created)

```
openmotion-bl/
  CMakeLists.txt                 # dual-target, selects Targets/<OMOTION_BL_TARGET>
  CMakePresets.json              # sensor-Debug/Release, console-Debug/Release
  version.h.in
  cmake/gcc-arm-none-eabi.cmake
  Core/Inc/
    memory_map.h                 # H743 addresses + fw_metadata_t
    boot_state.h                 # backup-register crash-loop API
    bl_crypto.h                  # sha256/hmac/ecdsa wrappers
    main.h  common.h
  Core/Src/
    main.c                       # boot decision flow (target-agnostic)
    boot_state.c                 # RTC backup-register logic (host-testable)
    bl_crypto.c                  # hmac + verify wrappers (host-testable)
    sha256.c  uECC.c             # copied verbatim from $REF
  Targets/sensor/
    target_config.h              # APP/META/BITSTREAM addresses, USB ids, product str
    clock.c  gpio.c              # OTG_HS + ULPI bring-up, USB_RESET/USB_MUX
    usbd_conf.c  usbd_desc.c     # OTG_HS, VID/PID, product string
    STM32H743VIHX_BL.ld          # FLASH origin 0x08000000 len 128K
    startup_stm32h743xx.s        # copied from sensor-fw
  Targets/console/
    target_config.h
    clock.c  gpio.c              # OTG_FS internal PHY, HSI48
    usbd_conf.c  usbd_desc.c
    STM32H743VIHX_BL.ld
    startup_stm32h743xx.s        # copied from console-fw
  USB_DEVICE/App/
    usbd_dfu_if.c                # region map; rejects BL sector; bitstream on sensor
    usbd_dfu_if.h
  Drivers/                       # H743 HAL + CMSIS (copied from sensor-fw)
  Middlewares/ST/.../DFU/        # ST USB device core + DFU class
  test/
    generate_keys.py             # ported from $REF (unchanged logic)
    dfu-test.py                  # ported; H743 default addresses + 32B padding
    test_packaging.py            # pytest: pack/verify/CRC parity (NEW)
    keys/                        # gitignored; dev keys only
    README.md
  native-test/                   # host gcc build of boot_state + metadata
    CMakeLists.txt
    test_boot_state.c
    test_metadata.c
  .github/workflows/build-firmware.yml
  README.md  .gitignore  LICENSE
```

---

## Phase 0 — Repo bootstrap

### Task 1: Create the private GitHub repo and local working copy

**Files:** none yet (repo scaffolding).

- [ ] **Step 1: Create the empty private repo**

```bash
gh repo create OpenwaterHealth/openmotion-bl --private \
  --description "STM32H743 secure USB-DFU bootloader for Open-Motion sensor and console" \
  --disable-wiki
```

Expected: `✓ Created repository OpenwaterHealth/openmotion-bl`. If `gh` lacks org rights, stop and report — do not fall back to a personal repo (design locked to org/private).

- [ ] **Step 2: Initialize the local working copy**

```bash
cd C:/Users/ethan/Projects
git clone https://github.com/OpenwaterHealth/openmotion-bl.git
cd openmotion-bl
git checkout -b feature/initial-port
```

Expected: empty repo cloned, on branch `feature/initial-port`.

- [ ] **Step 3: Seed `.gitignore`, `LICENSE`, `README.md` stub**

`.gitignore`:
```
build/
test/keys/
test/*.signed.bin
test/readback_*.bin
__pycache__/
*.pyc
.cache/
```

Copy `LICENSE` from `$REF/`. Create a minimal `README.md` with the project title and a "WIP — see docs" line (full README written in Task 30).

- [ ] **Step 4: Commit**

```bash
git add .gitignore LICENSE README.md
git commit -m "chore: scaffold openmotion-bl repo"
```

---

### Task 2: Graft H743 HAL drivers, CMSIS, and toolchain file

**Files:**
- Create: `$BL/Drivers/**` (from sensor-fw)
- Create: `$BL/cmake/gcc-arm-none-eabi.cmake`
- Create: `$BL/Targets/sensor/startup_stm32h743xx.s`, `$BL/Targets/console/startup_stm32h743xx.s`

- [ ] **Step 1: Copy the HAL + CMSIS drivers from sensor-fw**

```bash
cp -r C:/Users/ethan/Projects/openmotion-sensor-fw/Drivers $BL/Drivers
cp C:/Users/ethan/Projects/openmotion-sensor-fw/cmake/gcc-arm-none-eabi.cmake $BL/cmake/gcc-arm-none-eabi.cmake
cp C:/Users/ethan/Projects/openmotion-sensor-fw/startup_stm32h743xx.s $BL/Targets/sensor/startup_stm32h743xx.s
cp C:/Users/ethan/Projects/openmotion-console-fw/startup_stm32h743xx.s $BL/Targets/console/startup_stm32h743xx.s
```

(The H743 HAL is identical between the two app repos; sensor-fw's copy is the source of truth for the shared `Drivers/`.)

- [ ] **Step 2: Commit**

```bash
git add Drivers cmake Targets
git commit -m "chore: import STM32H7 HAL, CMSIS, toolchain, startup"
```

---

### Task 3: Dual-target CMake skeleton that builds an empty firmware

**Files:**
- Create: `$BL/CMakeLists.txt`, `$BL/CMakePresets.json`, `$BL/version.h.in`
- Create: `$BL/Core/Src/main.c` (temporary stub), `$BL/Core/Inc/main.h`, `$BL/Core/Inc/common.h`
- Create: `$BL/Targets/sensor/STM32H743VIHX_BL.ld`, `$BL/Targets/console/STM32H743VIHX_BL.ld`

The goal of this task is a green build of an empty `while(1)` firmware for both targets, proving the toolchain + linker + startup wire up before any logic lands.

- [ ] **Step 1: Write the bootloader linker script (sensor)** — `$BL/Targets/sensor/STM32H743VIHX_BL.ld`

Copy sensor-fw's `STM32H743VIHX_FLASH.ld` to this path, then change only the FLASH region so the bootloader occupies sector 0:

```
  FLASH (rx)     : ORIGIN = 0x08000000, LENGTH = 128K
```

Leave the RAM regions (DTCMRAM/RAM_D1/RAM_D2/SRAM4/ITCMRAM) unchanged. The `.ram_d2` and `.sram4` sections stay (USB buffers use RAM_D2).

- [ ] **Step 2: Write the bootloader linker script (console)** — `$BL/Targets/console/STM32H743VIHX_BL.ld`

Copy console-fw's `STM32H743VIHX_FLASH.ld`, apply the same `FLASH ... LENGTH = 128K` change.

- [ ] **Step 3: `version.h.in`** — copy from `$REF/version.h.in`:

```c
#ifndef VERSION_H
#define VERSION_H
#define FW_VERSION "@FW_GIT_DESCRIBE@"
#define FW_SHA "@FW_GIT_SHA@"
#define FW_BUILD_TIME "@FW_BUILD_TIME_UTC@"
#endif
```

- [ ] **Step 4: `Core/Inc/common.h`** — copy from `$REF/Core/Inc/common.h` (version-string glue, unchanged).

- [ ] **Step 5: `Core/Inc/main.h`** — minimal:

```c
#ifndef __MAIN_H
#define __MAIN_H
#include "stm32h7xx_hal.h"
void Error_Handler(void);
#endif
```

- [ ] **Step 6: Temporary `Core/Src/main.c` stub**

```c
#include "main.h"
int main(void) { HAL_Init(); for(;;){} }
void Error_Handler(void){ __disable_irq(); for(;;){} }
void SysTick_Handler(void){ HAL_IncTick(); }
```

- [ ] **Step 7: `CMakePresets.json`** — four presets via a `OMOTION_BL_TARGET` cache var:

```json
{
  "version": 3,
  "configurePresets": [
    { "name": "base", "hidden": true, "generator": "Ninja",
      "binaryDir": "${sourceDir}/build/${presetName}",
      "toolchainFile": "${sourceDir}/cmake/gcc-arm-none-eabi.cmake" },
    { "name": "sensor-Debug",   "inherits": "base", "binaryDir": "${sourceDir}/build/sensor-Debug",
      "cacheVariables": { "CMAKE_BUILD_TYPE": "Debug",   "OMOTION_BL_TARGET": "sensor" } },
    { "name": "sensor-Release", "inherits": "base", "binaryDir": "${sourceDir}/build/sensor-Release",
      "cacheVariables": { "CMAKE_BUILD_TYPE": "Release", "OMOTION_BL_TARGET": "sensor" } },
    { "name": "console-Debug",  "inherits": "base", "binaryDir": "${sourceDir}/build/console-Debug",
      "cacheVariables": { "CMAKE_BUILD_TYPE": "Debug",   "OMOTION_BL_TARGET": "console" } },
    { "name": "console-Release","inherits": "base", "binaryDir": "${sourceDir}/build/console-Release",
      "cacheVariables": { "CMAKE_BUILD_TYPE": "Release", "OMOTION_BL_TARGET": "console" } }
  ],
  "buildPresets": [
    { "name": "sensor-Debug",   "configurePreset": "sensor-Debug" },
    { "name": "sensor-Release", "configurePreset": "sensor-Release" },
    { "name": "console-Debug",  "configurePreset": "console-Debug" },
    { "name": "console-Release","configurePreset": "console-Release" }
  ]
}
```

- [ ] **Step 8: `CMakeLists.txt`** — target selection, sources, version generation, hex/bin output

```cmake
cmake_minimum_required(VERSION 3.22)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

if(NOT OMOTION_BL_TARGET)
  message(FATAL_ERROR "Set OMOTION_BL_TARGET=sensor|console (use a preset)")
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Debug")
endif()

# version.h (git describe), identical mechanism to OpenLIFU/app repos
find_package(Git QUIET)
if(GIT_FOUND)
  execute_process(COMMAND ${GIT_EXECUTABLE} describe --tags --dirty --always
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR} OUTPUT_VARIABLE FW_GIT_DESCRIBE
    OUTPUT_STRIP_TRAILING_WHITESPACE RESULT_VARIABLE _d)
  execute_process(COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR} OUTPUT_VARIABLE FW_GIT_SHA
    OUTPUT_STRIP_TRAILING_WHITESPACE RESULT_VARIABLE _s)
  if(NOT _d EQUAL 0) OR (FW_GIT_DESCRIBE STREQUAL "")) set(FW_GIT_DESCRIBE "unknown") endif()
  if(NOT _s EQUAL 0) set(FW_GIT_SHA "unknown") endif()
else()
  set(FW_GIT_DESCRIBE "unknown") set(FW_GIT_SHA "unknown")
endif()
string(TIMESTAMP FW_BUILD_TIME_UTC "%Y-%m-%d %H:%M:%S UTC" UTC)
configure_file(${CMAKE_SOURCE_DIR}/version.h.in ${CMAKE_BINARY_DIR}/generated/version.h @ONLY)

set(CMAKE_PROJECT_NAME openmotion-${OMOTION_BL_TARGET}-bl)
project(${CMAKE_PROJECT_NAME} C ASM)
add_executable(${CMAKE_PROJECT_NAME})

set(TGT ${CMAKE_SOURCE_DIR}/Targets/${OMOTION_BL_TARGET})

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
  Core/Src/main.c
  ${TGT}/startup_stm32h743xx.s
  # HAL sources added in Task 4 (sub-list include)
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
  Core/Inc
  ${TGT}
  ${CMAKE_BINARY_DIR}/generated
  Drivers/STM32H7xx_HAL_Driver/Inc
  Drivers/CMSIS/Device/ST/STM32H7xx/Include
  Drivers/CMSIS/Include
)

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
  USE_HAL_DRIVER STM32H743xx
)

target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
  -mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb
  -Wall -ffunction-sections -fdata-sections
  $<$<CONFIG:Debug>:-Og -g3>
  $<$<CONFIG:Release>:-Os>
)
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
  -mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb
  -T${TGT}/STM32H743VIHX_BL.ld
  --specs=nano.specs --specs=nosys.specs
  -Wl,--gc-sections -Wl,-Map=${CMAKE_PROJECT_NAME}.map
)

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
  COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
  COMMENT "Generating .hex and .bin")
```

> Note: the exact HAL source list and any `target_compile_options` deltas should be cross-checked against sensor-fw's `CMakeLists.txt` during Task 4 so flags match the known-good app build. The version-guard line with the stray paren above is pseudocode — write it as a clean `if(NOT _d EQUAL 0 OR FW_GIT_DESCRIBE STREQUAL "")`.

- [ ] **Step 9: Add the HAL source list** (deferred to Task 4 which imports the exact list). For now, add the minimum HAL sources to compile the stub: `stm32h7xx_hal.c`, `stm32h7xx_hal_cortex.c`, `stm32h7xx_hal_rcc.c`, `stm32h7xx_hal_gpio.c`, `stm32h7xx_hal_pwr.c`, `stm32h7xx_hal_pwr_ex.c`, `stm32h7xx_hal_rcc_ex.c`, plus `system_stm32h7xx.c` from `Drivers/CMSIS/Device/.../Source/Templates/`.

- [ ] **Step 10: Build both targets**

Run:
```bash
cmake --preset sensor-Release  && cmake --build build/sensor-Release
cmake --preset console-Release && cmake --build build/console-Release
```
Expected: both produce `openmotion-sensor-bl.hex` / `openmotion-console-bl.hex` with no errors. Verify the sensor `.map` shows `.isr_vector` at `0x08000000` and total flash usage < 128 KB.

- [ ] **Step 11: Commit**

```bash
git add CMakeLists.txt CMakePresets.json version.h.in Core Targets
git commit -m "feat: dual-target CMake skeleton builds empty BL for sensor and console"
```

---

## Phase 1 — Memory map and crypto (host-testable)

### Task 4: Port `memory_map.h` with H743 addresses

**Files:**
- Create: `$BL/Core/Inc/memory_map.h`

- [ ] **Step 1: Write `memory_map.h`** (ported from `$REF`, H743 values)

```c
#ifndef INC_MEMORY_MAP_H_
#define INC_MEMORY_MAP_H_
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define FLASH_START_ADDRESS     ((uint32_t)0x08000000)
#define FLASH_END_ADDRESS       ((uint32_t)0x08200000) /* 2 MB */
#define BOOTLOADER_ADDRESS      ((uint32_t)0x08000000) /* sector 0, 128 KB */
#define METADATA_ADDRESS        ((uint32_t)0x08020000) /* sector 1 */
#define APPLICATION_ADDRESS     ((uint32_t)0x08040000) /* sector 2 */

/* Per-target application ceiling (set in target_config.h):
   sensor  -> 0x081A0000 (FPGA bitstream)  = 1408 KB
   console -> 0x081C0000 (odometer sector) = 1536 KB */
#include "target_config.h"

#define FW_META_MAGIC                    ((uint32_t)0x314D4657) /* 'WFM1' */
#define FW_META_VERSION                  ((uint16_t)3U)
#define FW_META_FLAG_SIGNATURE_REQUIRED  ((uint16_t)0x0001U)
#define FW_SIGNATURE_SIZE_BYTES          ((uint32_t)64U)
#define FW_TRUST_TAG_SIZE_BYTES          ((uint32_t)32U)

typedef struct {
  uint32_t magic;
  uint16_t version;
  uint16_t flags;
  uint32_t fw_address;
  uint32_t fw_length;
  uint32_t fw_crc32;
  uint32_t key_id;
  uint8_t  signature[FW_SIGNATURE_SIZE_BYTES];
  uint8_t  trust_tag[FW_TRUST_TAG_SIZE_BYTES];
  uint32_t meta_crc32;
} fw_metadata_t;

#define SRAM_BASE_ADDRESS  ((uint32_t)0x24000000) /* RAM_D1 (app .data/.bss) */

#ifdef __cplusplus
}
#endif
#endif
```

> Note: `fw_metadata_t` layout is byte-identical to OpenLIFU so the host tooling's struct packing carries over unchanged (124 bytes). `APPLICATION_MAX_SIZE` is defined per-target in `target_config.h` (Task 6/7).

- [ ] **Step 2: Write the two `target_config.h` files** (sensor and console)

`$BL/Targets/sensor/target_config.h`:
```c
#ifndef TARGET_CONFIG_H_
#define TARGET_CONFIG_H_
#define APPLICATION_MAX_SIZE   ((uint32_t)(0x081A0000U - 0x08040000U)) /* 1408 KB */
#define FPGA_BITSTREAM_ADDRESS ((uint32_t)0x081A0000U)
#define BL_USB_VID             (0x0483U)
#define BL_USB_PID             (0xDF11U)
#define BL_PRODUCT_STRING      "OMOTION SENSOR BL DFU " FW_VERSION_STRING
#define BL_HAS_BITSTREAM_REGION 1
#endif
```

`$BL/Targets/console/target_config.h`:
```c
#ifndef TARGET_CONFIG_H_
#define TARGET_CONFIG_H_
#define APPLICATION_MAX_SIZE   ((uint32_t)(0x081C0000U - 0x08040000U)) /* 1536 KB */
#define BL_USB_VID             (0x0483U)
#define BL_USB_PID             (0xDF11U)
#define BL_PRODUCT_STRING      "OMOTION CONSOLE BL DFU " FW_VERSION_STRING
#define BL_HAS_BITSTREAM_REGION 0
#endif
```

- [ ] **Step 3: Build both targets** to confirm headers compile (`main.c` stub includes `memory_map.h`). Add `#include "memory_map.h"` to the stub temporarily.

Run: `cmake --build build/sensor-Release && cmake --build build/console-Release`
Expected: clean build.

- [ ] **Step 4: Commit**

```bash
git add Core/Inc/memory_map.h Targets/sensor/target_config.h Targets/console/target_config.h
git commit -m "feat: H743 memory map and per-target config"
```

---

### Task 5: Copy crypto primitives verbatim

**Files:**
- Create: `$BL/Core/Src/sha256.c`, `$BL/Core/Inc/sha256.h`
- Create: `$BL/Core/Src/uECC.c`, `$BL/Core/Inc/uECC.h`, `$BL/Core/Inc/uECC_vli.h`
- Create: `$BL/Core/Src/asm_arm.inc`, `curve-specific.inc`, `platform-specific.inc`, `asm_avr.inc`

- [ ] **Step 1: Copy all crypto files from `$REF`**

```bash
cp $REF/Core/Src/sha256.c $BL/Core/Src/
cp $REF/Core/Inc/sha256.h $BL/Core/Inc/
cp $REF/Core/Src/uECC.c $BL/Core/Src/
cp $REF/Core/Inc/uECC.h $BL/Core/Inc/
cp $REF/Core/Inc/uECC_vli.h $BL/Core/Inc/
cp $REF/Core/Src/asm_arm.inc $REF/Core/Src/asm_avr.inc \
   $REF/Core/Src/curve-specific.inc $REF/Core/Src/platform-specific.inc $BL/Core/Src/
```

- [ ] **Step 2: Add sources + uECC defines to `CMakeLists.txt`**

In `target_sources(...)` add `Core/Src/sha256.c` and `Core/Src/uECC.c`. Add the uECC compile defines copied from `$REF/CMakeLists.txt` lines 100-109:
```cmake
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
  uECC_SUPPORTS_secp160r1=0 uECC_SUPPORTS_secp192r1=0 uECC_SUPPORTS_secp224r1=0
  uECC_SUPPORTS_secp256r1=1 uECC_SUPPORTS_secp256k1=0
  uECC_SUPPORT_COMPRESSED_POINT=0 uECC_OPTIMIZATION_LEVEL=0)
```

- [ ] **Step 3: Build both targets** (crypto compiles for Cortex-M7).

Run: `cmake --build build/sensor-Release && cmake --build build/console-Release`
Expected: clean build.

- [ ] **Step 4: Commit**

```bash
git add Core/Src/sha256.c Core/Inc/sha256.h Core/Src/uECC.c Core/Inc/uECC.h Core/Inc/uECC_vli.h Core/Src/*.inc CMakeLists.txt
git commit -m "feat: import SHA-256 and uECC P-256 crypto primitives"
```

---

### Task 6: Crypto wrapper module with host-side known-answer tests

**Files:**
- Create: `$BL/Core/Inc/bl_crypto.h`, `$BL/Core/Src/bl_crypto.c`
- Create: `$BL/native-test/CMakeLists.txt`, `$BL/native-test/test_crypto.c`

`bl_crypto.c` holds the HMAC-SHA256 and the constant-time compare lifted out of OpenLIFU's `main.c` so they are pure and host-testable.

- [ ] **Step 1: Write the failing host test** — `$BL/native-test/test_crypto.c`

```c
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include "bl_crypto.h"

/* RFC 4231 Test Case 2: key="Jefe", data="what do ya want for nothing?" */
static const char *k = "Jefe";
static const char *d = "what do ya want for nothing?";
static const uint8_t expected[32] = {
  0x5b,0xdc,0xc1,0x46,0xbf,0x60,0x75,0x4e,0x6a,0x04,0x24,0x26,0x08,0x95,0x75,0xc7,
  0x5a,0x00,0x3f,0x08,0x9d,0x27,0x39,0x83,0x9d,0xec,0x58,0xb9,0x64,0xec,0x38,0x43
};

int main(void) {
  uint8_t out[32];
  bl_hmac_sha256((const uint8_t*)k, 4, (const uint8_t*)d, strlen(d), out);
  assert(memcmp(out, expected, 32) == 0);

  uint8_t a[4] = {1,2,3,4}, b[4] = {1,2,3,4}, c[4] = {1,2,3,5};
  assert(bl_consttime_equal(a, b, 4) == 1);
  assert(bl_consttime_equal(a, c, 4) == 0);
  printf("crypto OK\n");
  return 0;
}
```

- [ ] **Step 2: Write `bl_crypto.h`**

```c
#ifndef BL_CRYPTO_H_
#define BL_CRYPTO_H_
#include <stdint.h>
void bl_hmac_sha256(const uint8_t *key, uint32_t key_len,
                    const uint8_t *msg, uint32_t msg_len, uint8_t out[32]);
uint8_t bl_consttime_equal(const uint8_t *a, const uint8_t *b, uint32_t len);
#endif
```

- [ ] **Step 3: Write `bl_crypto.c`** — move `bl_hmac_sha256` and `bl_consttime_equal` verbatim from `$REF/Core/Src/main.c` (lines 393-447), `#include "sha256.h"`.

- [ ] **Step 4: Write `native-test/CMakeLists.txt`** (host gcc, not ARM)

```cmake
cmake_minimum_required(VERSION 3.22)
project(omotion_bl_native C)
set(CMAKE_C_STANDARD 11)
add_executable(test_crypto test_crypto.c
  ${CMAKE_SOURCE_DIR}/../Core/Src/bl_crypto.c
  ${CMAKE_SOURCE_DIR}/../Core/Src/sha256.c)
target_include_directories(test_crypto PRIVATE
  ${CMAKE_SOURCE_DIR}/../Core/Inc)
enable_testing()
add_test(NAME crypto COMMAND test_crypto)
```

- [ ] **Step 5: Run the test to verify it fails, then passes**

Run:
```bash
cmake -S native-test -B build/native -G Ninja && cmake --build build/native && ctest --test-dir build/native --output-on-failure
```
Expected first run (before `bl_crypto.c` exists): build error. After Step 3: `crypto OK`, ctest PASS.

- [ ] **Step 6: Build ARM targets** to confirm `bl_crypto.c` also compiles on-target (add it to `target_sources`).

Run: `cmake --build build/sensor-Release`
Expected: clean.

- [ ] **Step 7: Commit**

```bash
git add Core/Inc/bl_crypto.h Core/Src/bl_crypto.c native-test CMakeLists.txt
git commit -m "feat: host-tested HMAC-SHA256 + constant-time compare wrapper"
```

---

## Phase 2 — Boot-state logic (host-testable)

### Task 7: Extract backup-register crash-loop logic into a testable module

**Files:**
- Create: `$BL/Core/Inc/boot_state.h`, `$BL/Core/Src/boot_state.c`
- Create: `$BL/native-test/test_boot_state.c`

The OpenLIFU boot-state logic reads/writes `RTC->BKP0R..BKP4R`. To make it host-testable, `boot_state.c` accesses backup registers through a small accessor that is the real RTC on-target and a plain array in the native test.

- [ ] **Step 1: Write the failing host test** — `$BL/native-test/test_boot_state.c`

```c
#include <assert.h>
#include <stdio.h>
#include "boot_state.h"

/* Native shim: backup registers are a plain array (see boot_state.h). */
extern uint32_t g_bkp[5];

int main(void) {
  /* Cold init writes signature and zeros state. */
  g_bkp[0]=0; g_bkp[1]=0; g_bkp[2]=0; g_bkp[3]=0; g_bkp[4]=0;
  bs_init();
  assert(g_bkp[0] == BS_SIGNATURE);

  /* Auth cache round-trips through the XOR mask. */
  bs_auth_cache_store(0xCAFEF00DU);
  assert(bs_auth_cache_match(0xCAFEF00DU) == 1);
  assert(bs_auth_cache_match(0x00000001U) == 0);
  bs_auth_cache_clear();
  assert(bs_auth_cache_match(0xCAFEF00DU) == 0);

  /* Two IWDG resets while BOOT_IN_PROGRESS latch FORCE_DFU. */
  bs_init();
  bs_mark_boot_in_progress();
  bs_reset_t r = { .por=0, .pin=0, .sft=0, .iwdg=1, .wwdg=0 };
  fw_meta_view m = { .valid=1, .fw_crc32=0x11111111U };
  assert(bs_should_enter_dfu(&r, &m) == 0); /* fail count 1 */
  bs_mark_boot_in_progress();
  uint8_t dfu = bs_should_enter_dfu(&r, &m); /* fail count 2 -> latch */
  assert(dfu == 1);

  /* New firmware CRC clears the FORCE_DFU latch. */
  fw_meta_view m2 = { .valid=1, .fw_crc32=0x22222222U };
  bs_reset_t r2 = { .por=0, .pin=0, .sft=0, .iwdg=0, .wwdg=0 };
  assert(bs_should_enter_dfu(&r2, &m2) == 0);

  printf("boot_state OK\n");
  return 0;
}
```

- [ ] **Step 2: Write `boot_state.h`**

```c
#ifndef BOOT_STATE_H_
#define BOOT_STATE_H_
#include <stdint.h>

#define BS_SIGNATURE 0x4F57424CU /* 'OWBL' */
#define BS_REQ_DFU_MAGIC 0x21554644U /* 'DFU!' */

typedef struct { uint8_t por, pin, sft, iwdg, wwdg; } bs_reset_t;
typedef struct { uint8_t valid; uint32_t fw_crc32; } fw_meta_view;

void    bs_init(void);
void    bs_clear_cold(const bs_reset_t *r);
uint8_t bs_should_enter_dfu(const bs_reset_t *r, const fw_meta_view *m);
void    bs_mark_boot_in_progress(void);
void    bs_clear_boot_in_progress(void);
void    bs_request_dfu(void);              /* set BKP req magic (app calls equiv) */
uint8_t bs_auth_cache_match(uint32_t crc);
void    bs_auth_cache_store(uint32_t crc);
void    bs_auth_cache_clear(void);
#endif
```

- [ ] **Step 3: Write `boot_state.c`** — port the backup-register helpers and `bl_should_enter_dfu` logic from `$REF/Core/Src/main.c` (lines 242-656). Access registers through `BKP(i)`:

```c
#include "boot_state.h"
#if defined(BS_NATIVE_TEST)
uint32_t g_bkp[5];
#define BKP(i) (g_bkp[(i)])
#define BS_RTC_ENABLE() ((void)0)
#else
#include "stm32h7xx_hal.h"
/* STM32H7 backup registers: RTC->BKP0R..BKP31R; we use 0..4. */
#define BKP(i) (RTC->BKP##i##R) /* expanded via small wrapper below */
/* ... on-target accessor: see note */
#endif
/* Port constants BS_STATE_FORCE_DFU/BOOT_IN_PROGRESS/BOOT_OK/FAILCOUNT,
   threshold 2, XOR 0xA5964C3D, and the functions bs_init/bs_clear_cold/
   bs_should_enter_dfu/bs_mark_*/bs_auth_cache_* — logic copied 1:1 from $REF. */
```

> Note: STM32H7 exposes backup registers as `RTC->BKP0R … BKP31R` (token-paste won't index cleanly). On-target, implement `static volatile uint32_t* bkp(int i){ return &RTC->BKP0R + i; }` and use `*bkp(i)`. The native build uses the `g_bkp[]` array. Enabling backup-domain write access (`HAL_PWR_EnableBkUpAccess`, RTC clock) is on-target only; the native build no-ops it. Keep the XOR-masked auth cache and the fail-count crash-loop policy identical to OpenLIFU.

- [ ] **Step 4: Add a `test_boot_state` target to `native-test/CMakeLists.txt`**

```cmake
add_executable(test_boot_state test_boot_state.c ${CMAKE_SOURCE_DIR}/../Core/Src/boot_state.c)
target_include_directories(test_boot_state PRIVATE ${CMAKE_SOURCE_DIR}/../Core/Inc)
target_compile_definitions(test_boot_state PRIVATE BS_NATIVE_TEST)
add_test(NAME boot_state COMMAND test_boot_state)
```

- [ ] **Step 5: Build + run native tests**

Run: `cmake --build build/native && ctest --test-dir build/native --output-on-failure`
Expected: `boot_state OK`, both tests PASS.

- [ ] **Step 6: Commit**

```bash
git add Core/Inc/boot_state.h Core/Src/boot_state.c native-test
git commit -m "feat: host-tested backup-register crash-loop boot state"
```

---

## Phase 3 — Boot decision and app jump

### Task 8: Port the boot decision flow into `main.c` (no USB yet)

**Files:**
- Modify: `$BL/Core/Src/main.c` (replace stub)
- Create: `$BL/Core/Inc/bl_boot.h`, `$BL/Core/Src/bl_boot.c` (validation helpers)

`bl_boot.c` holds the metadata/auth validation (`firmware_metadata_valid`, `firmware_signature_valid`, `firmware_trust_tag_valid`, `firmware_crc32`, vector-sanity checks) ported from OpenLIFU, calling into `bl_crypto` + `boot_state`. `main.c` orchestrates: HAL init → clock → CRC → read reset flags → `bs_init` → validate → decide → jump or DFU.

- [ ] **Step 1: Write `bl_boot.h`** declaring the validation entry points

```c
#ifndef BL_BOOT_H_
#define BL_BOOT_H_
#include <stdint.h>
#include "memory_map.h"
uint8_t bl_app_vectors_sane(void);
uint8_t bl_firmware_metadata_valid(const fw_metadata_t *meta);
void    bl_jump_to_application(uint32_t app_base); /* sets VTOR, MSP, branches */
#endif
```

- [ ] **Step 2: Write `bl_boot.c`** — port `bl_app_stack_pointer_sane`, `bl_app_reset_vector_sane` (combine into `bl_app_vectors_sane`), `firmware_sha256`, `firmware_crc32`, `firmware_trust_tag_valid`, `firmware_signature_valid`, `firmware_metadata_valid` from `$REF/main.c`. Swap the trust HMAC key and pubkey tables (`g_bl_trust_hmac_key`, `g_bl_pubkeys`) in here (copied placeholder values from `$REF` — replaced per-target later). Use the H743 CRC peripheral handle (extern `hcrc`).

- [ ] **Step 3: Write the H743 `bl_jump_to_application`** (replaces the F0 SRAM-remap version)

```c
void bl_jump_to_application(uint32_t app_base) {
  uint32_t msp = *(volatile uint32_t*)app_base;
  uint32_t pc  = *(volatile uint32_t*)(app_base + 4U);
  __disable_irq();
  SysTick->CTRL = 0; SysTick->LOAD = 0; SysTick->VAL = 0;
  HAL_RCC_DeInit();
  for (int i = 0; i < 8; i++) { NVIC->ICER[i] = 0xFFFFFFFFU; NVIC->ICPR[i] = 0xFFFFFFFFU; }
  SCB_DisableICache();
  SCB_DisableDCache();
  SCB->VTOR = app_base;        /* Cortex-M7 has VTOR; no SRAM remap needed */
  __DSB(); __ISB();
  __set_MSP(msp);
  __enable_irq();
  ((void(*)(void))pc)();
  for(;;){}
}
```

> Note: caches are disabled before jump so the app starts from the same cold state the existing app expects (the app re-enables them if it wants). Leave the clock tree returned to HSI via `HAL_RCC_DeInit()`.

- [ ] **Step 4: Rewrite `main.c`** — orchestration ported from `$REF/main.c` main() (lines 664-792), minus USB (added Task 12). Calls `SystemClock_Config()` (target file, Task 9), `MX_CRC_Init()`, reset-flag read, `bs_init`, `bs_clear_cold`, vector + metadata validation, `bs_should_enter_dfu`, and on success `bs_mark_boot_in_progress` → `MX_IWDG_Init` → `bl_jump_to_application(APPLICATION_ADDRESS)`. On failure, for now just `for(;;)` (USB DFU added in Task 12). Provide `MX_CRC_Init` and `MX_IWDG_Init` (port from `$REF`, H743 HAL — IWDG on H7 is `IWDG1`).

- [ ] **Step 5: Build both targets**

Run: `cmake --build build/sensor-Release && cmake --build build/console-Release`
Expected: clean build; `.map` confirms total size < 128 KB.

- [ ] **Step 6: Commit**

```bash
git add Core/Inc/bl_boot.h Core/Src/bl_boot.c Core/Src/main.c
git commit -m "feat: port boot decision flow and H743 VTOR app jump"
```

---

### Task 9: Per-target clock and CRC/GPIO bring-up

**Files:**
- Create: `$BL/Targets/sensor/clock.c`, `$BL/Targets/sensor/gpio.c`
- Create: `$BL/Targets/console/clock.c`, `$BL/Targets/console/gpio.c`

- [ ] **Step 1: Sensor clock** — copy `SystemClock_Config` from sensor-fw's `Core/Src/main.c` (HSE 25 MHz → 480 MHz, USB clock for OTG_HS). Trim to only what the BL needs (SYSCLK + USB + LSI for IWDG). Provide `SystemClock_Config(void)`.

- [ ] **Step 2: Console clock** — copy `SystemClock_Config` from console-fw (HSE 25 MHz, HSI48 for OTG_FS USB). Provide `SystemClock_Config(void)`.

- [ ] **Step 3: Sensor GPIO** — minimal: enable GPIO clocks, configure USB_RESET (PA15) and USB_MUX (PC12) as outputs, plus any LED used as a boot indicator. Port from sensor-fw `MX_GPIO_Init`, keeping only USB-relevant pins. Provide `MX_GPIO_Init(void)`.

- [ ] **Step 4: Console GPIO** — minimal: GPIO clocks + any boot LED. Provide `MX_GPIO_Init(void)`.

- [ ] **Step 5: Add `clock.c`/`gpio.c` to `target_sources`** via the `${TGT}` path so each preset compiles its own.

- [ ] **Step 6: Build both targets**

Run: `cmake --build build/sensor-Release && cmake --build build/console-Release`
Expected: clean.

- [ ] **Step 7: Commit**

```bash
git add Targets/sensor/clock.c Targets/sensor/gpio.c Targets/console/clock.c Targets/console/gpio.c CMakeLists.txt
git commit -m "feat: per-target clock and GPIO bring-up"
```

---

## Phase 4 — USB DFU

### Task 10: Import ST USB Device core + DFU class

**Files:**
- Create: `$BL/Middlewares/ST/STM32_USB_Device_Library/**`
- Create: `$BL/USB_DEVICE/App/usb_device.{c,h}`, `usbd_desc.{c,h}` (per target), `usbd_dfu_if.{c,h}`
- Create: per-target `usbd_conf.{c,h}`

The sensor-fw uses a **composite** USB device; the BL needs only the **DFU** class. Import the ST DFU class from the OpenLIFU BL (already a DFU-only device) and re-target its `usbd_conf.c` to the H743 OTG peripheral using sensor-fw/console-fw's `usbd_conf.c` as the HAL reference.

- [ ] **Step 1: Copy the ST USB Device library + DFU class from `$REF`**

```bash
cp -r $REF/Middlewares $BL/Middlewares
cp $REF/USB_DEVICE/App/usbd_dfu_if.c $BL/USB_DEVICE/App/
cp $REF/USB_DEVICE/App/usbd_dfu_if.h $BL/USB_DEVICE/App/
cp $REF/USB_DEVICE/App/usb_device.c  $BL/USB_DEVICE/App/
cp $REF/USB_DEVICE/App/usb_device.h  $BL/USB_DEVICE/App/
```

- [ ] **Step 2: Confirm the DFU middleware is H7-compatible** — the ST `usbd_dfu.c`/`usbd_core.c`/`usbd_ctlreq.c`/`usbd_ioreq.c` are MCU-agnostic. Only `usbd_conf.c` (the HAL glue) is MCU-specific and is replaced per-target in Step 3.

- [ ] **Step 3: Write per-target `usbd_conf.c`/`usbd_conf.h`** — copy from each app repo's `USB_DEVICE/Target/usbd_conf.c`, then strip composite/CDC/HISTO/IMU specifics, keeping only the OTG init for DFU. Sensor: `USB_OTG_HS` + ULPI (`USE_USB_HS`), USB clock from PLL. Console: `USB_OTG_FS` internal PHY, HSI48. Place at `$BL/Targets/<board>/usbd_conf.c`.

- [ ] **Step 4: Build both targets** with the USB middleware compiled in (add Middlewares includes + sources, and `${TGT}/usbd_conf.c`, to CMake).

Run: `cmake --build build/sensor-Release && cmake --build build/console-Release`
Expected: clean (links DFU class; `main.c` does not call it yet).

- [ ] **Step 5: Commit**

```bash
git add Middlewares USB_DEVICE Targets/*/usbd_conf.* CMakeLists.txt
git commit -m "feat: import ST USB Device DFU class and per-target USB glue"
```

---

### Task 11: Per-target USB descriptors (VID/PID + product string)

**Files:**
- Create: `$BL/Targets/sensor/usbd_desc.c`, `$BL/Targets/console/usbd_desc.c`
- Create: `$BL/USB_DEVICE/App/usbd_desc.h`

- [ ] **Step 1: Write each `usbd_desc.c`** — copy `$REF/USB_DEVICE/App/usbd_desc.c`, change the defines to pull from `target_config.h`:

```c
#define USBD_VID            BL_USB_VID            /* 0x0483 */
#define USBD_PID_FS         BL_USB_PID            /* 0xDF11 */
#define USBD_PRODUCT_STRING_FS  BL_PRODUCT_STRING /* "OMOTION <board> BL DFU <ver>" */
```

Sensor uses the HS descriptor variant if the ST DFU class differentiates; reuse the FS strings for both speeds as OpenLIFU does.

- [ ] **Step 2: Build both targets**, confirm each links its own `usbd_desc.c` from `${TGT}`.

Run: `cmake --build build/sensor-Release && cmake --build build/console-Release`
Expected: clean.

- [ ] **Step 3: Commit**

```bash
git add Targets/sensor/usbd_desc.c Targets/console/usbd_desc.c USB_DEVICE/App/usbd_desc.h CMakeLists.txt
git commit -m "feat: per-target USB descriptors with BL VID/PID and product string"
```

---

### Task 12: Wire DFU entry into `main.c` and adapt the region map

**Files:**
- Modify: `$BL/Core/Src/main.c` (add USB DFU entry on the failure path)
- Modify: `$BL/USB_DEVICE/App/usbd_dfu_if.c` (H743 sectors; reject BL sector; sensor bitstream region)

- [ ] **Step 1: Replace the failure-path `for(;;)` in `main.c`** with the OpenLIFU DFU bring-up: prep USB hardware (sensor: assert USB_RESET/USB_MUX), `MX_USB_DEVICE_Init()`, `MX_IWDG_Init()`, then the IWDG-refresh `while(1)` loop.

- [ ] **Step 2: Rewrite the DFU memory map string in `usbd_dfu_if.c`** for H743 128 KB sectors. The descriptor exposes metadata sector + app region only:

```c
/* 1 metadata sector (128K) + N app sectors (128K each), bootloader sector hidden.
   Sensor app = 1408K = 11 sectors; console app = 1536K = 12 sectors. */
#if BL_HAS_BITSTREAM_REGION
#define FLASH_DESC_STR "@Firmware/0x08020000/01*128Kg,11*128Kg/0x081A0000/02*128Kg"
#else
#define FLASH_DESC_STR "@Firmware/0x08020000/01*128Kg,12*128Kg"
#endif
```

> Note: verify the exact sector multipliers against `APPLICATION_MAX_SIZE` for each target during implementation; the bitstream region length must match the FPGA blob's actual sector span (read from sensor-fw's `merge_hex.py`/`CMakeLists.txt`).

- [ ] **Step 3: Port the write/erase guards** from `$REF/usbd_dfu_if.c` to H743: reject any address `< APPLICATION_ADDRESS` except the metadata sector (so the BL sector at `0x08000000` is never writable), and erase-by-sector / program 32-byte flash words via `HAL_FLASHEx_Erase` (sector model) and `HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, ...)`. Keep the virtual version-read (`0xFFFFFF00`) and reset-trigger (`0xFFFFFF08`) addresses.

- [ ] **Step 4: Build both targets**

Run: `cmake --build build/sensor-Release && cmake --build build/console-Release`
Expected: clean; `.map` total < 128 KB for both.

- [ ] **Step 5: Commit**

```bash
git add Core/Src/main.c USB_DEVICE/App/usbd_dfu_if.c
git commit -m "feat: DFU entry path and H743 sector-based region map"
```

---

## Phase 5 — Host tooling

### Task 13: Port `generate_keys.py` and `dfu-test.py` with H743 defaults

**Files:**
- Create: `$BL/test/generate_keys.py` (verbatim copy), `$BL/test/dfu-test.py` (ported), `$BL/test/README.md`

- [ ] **Step 1: Copy `generate_keys.py` verbatim** from `$REF/test/generate_keys.py` (no addresses inside; works as-is).

- [ ] **Step 2: Copy `dfu-test.py`** from `$REF/test/dfu-test.py`, then change the **default addresses** to H743 values: `--address` default `0x08040000`, `--meta-address` default `0x08020000`. Change the application-max guard to per-target values (accept a `--app-max` arg). Update flash-write padding so signed images are padded to a **32-byte** multiple (H743 flash word) rather than the F0 2-byte alignment — find the pad logic and change the alignment constant.

- [ ] **Step 3: Copy `test/README.md`** from `$REF/test/README.md`, update example addresses to H743.

- [ ] **Step 4: Smoke-run key generation**

Run:
```bash
cd $BL && python test/generate_keys.py --key-id 1
```
Expected: writes `test/keys/fw_signing_key.pem`, `fw_signing_pub.xy.bin`, `boot_trust_hmac_key.bin`; prints the `g_bl_pubkeys[]` C snippet.

- [ ] **Step 5: Commit**

```bash
git add test/generate_keys.py test/dfu-test.py test/README.md
git commit -m "feat: port host signing/DFU tooling with H743 defaults"
```

---

### Task 14: pytest round-trip — sign, verify, and CRC parity

**Files:**
- Create: `$BL/test/test_packaging.py`

This proves the host pack/verify path works offline and that the host CRC32 matches the STM32 CRC peripheral configuration (default polynomial, no in/out inversion, byte input — as in OpenLIFU's `MX_CRC_Init`).

- [ ] **Step 1: Write the failing test** — `$BL/test/test_packaging.py`

```python
import subprocess, sys, os, struct, zlib, pathlib
HERE = pathlib.Path(__file__).parent
KEYS = HERE / "keys"

def run(*args):
    return subprocess.run([sys.executable, str(HERE/"dfu-test.py"), *args],
                          capture_output=True, text=True, check=True)

def test_pack_then_verify(tmp_path):
    # fake app image (one 32-byte flash word + change)
    app = tmp_path / "app.bin"
    app.write_bytes(bytes(range(256)) * 8)  # 2 KB
    signed = tmp_path / "fw.signed.bin"
    run("pack-signed", str(app), str(KEYS/"fw_signing_key.pem"), str(signed),
        "--address","0x08040000","--meta-address","0x08020000","--key-id","1",
        "--trust-keyfile", str(KEYS/"boot_trust_hmac_key.bin"))
    out = run("verify-package", str(signed), str(KEYS/"fw_signing_pub.pem"),
              "--trust-keyfile", str(KEYS/"boot_trust_hmac_key.bin"))
    assert "OK" in out.stdout or out.returncode == 0

def test_crc32_matches_stm32_default():
    # STM32 default CRC: poly 0x04C11DB7, init 0xFFFFFFFF, no inversion, big-endian words.
    # dfu-test.py must expose the same crc; compare against a known vector.
    data = b"\x00\x01\x02\x03" * 8
    out = run("crc32", "--hex", data.hex())
    # value computed once from the device CRC config and pinned here:
    assert out.stdout.strip().lower().endswith("expected_value_pinned_during_impl")
```

> Note: `pack-signed`/`verify-package`/`crc32` subcommands exist in the ported `dfu-test.py`. If `dfu-test.py` lacks a standalone `crc32` subcommand, add one (a thin wrapper over the same CRC routine used in `pack-signed`) so the parity test has a target. Pin the expected CRC value after the first run by reading the device's CRC of the same bytes (or by trusting the documented STM32 default-CRC algorithm, which `zlib.crc32` does **not** match — the device uses the non-reflected MPEG-2-style CRC; implement that in the tool and assert against it).

- [ ] **Step 2: Run pytest, fix until green**

Run: `cd $BL && python -m pytest test/test_packaging.py -v`
Expected: both tests PASS. Generate keys first (`python test/generate_keys.py --key-id 1`) if `keys/` is absent.

- [ ] **Step 3: Commit**

```bash
git add test/test_packaging.py
git commit -m "test: host pack/verify round-trip and CRC32 parity"
```

---

### Task 15: Provision dev keys into the firmware build

**Files:**
- Modify: `$BL/Core/Src/bl_boot.c` (replace placeholder pubkey + trust key)

- [ ] **Step 1: Generate the dev keypair + trust key** (if not already): `python test/generate_keys.py --key-id 1`.

- [ ] **Step 2: Paste the printed `g_bl_pubkeys[]` entry** into `bl_boot.c`, and copy the 32 bytes of `test/keys/boot_trust_hmac_key.bin` into `g_bl_trust_hmac_key`. Add a prominent comment: "DEV KEYS — replace with production provisioning. Trust key bytes here MUST match the keyfile used by the host tool."

- [ ] **Step 3: Build both targets**, confirm clean.

Run: `cmake --build build/sensor-Release && cmake --build build/console-Release`

- [ ] **Step 4: Commit** (keys file itself stays gitignored; only the compiled-in bytes are committed, matching OpenLIFU's model)

```bash
git add Core/Src/bl_boot.c
git commit -m "chore: provision dev signing pubkey and trust HMAC key (placeholder)"
```

---

## Phase 6 — CI

### Task 16: GitHub Actions build for both targets

**Files:**
- Create: `$BL/.github/workflows/build-firmware.yml`

- [ ] **Step 1: Copy the OpenLIFU workflow** from `$REF/.github/workflows/build-firmware.yml` as the base.

- [ ] **Step 2: Edit it** to build all four presets in the `stm32-build-env` container and publish artifacts:

```yaml
      - name: Build inside container
        run: |
          docker run --rm -v ${{ github.workspace }}:/work -w /work \
            ghcr.io/openwaterhealth/stm32-build-env:latest bash -lc '
              for p in sensor-Release console-Release; do
                cmake --preset $p && cmake --build build/$p
              done'
      - name: Collect artifacts
        run: |
          mkdir -p out
          cp build/sensor-Release/openmotion-sensor-bl.{hex,bin} out/
          cp build/console-Release/openmotion-console-bl.{hex,bin} out/
      - uses: actions/upload-artifact@v4
        with: { name: openmotion-bl, path: out/ }
```

Keep the tag-triggered release job (`*.*.*`, `pre-*`) attaching `out/*` to the GitHub Release, mirroring OpenLIFU.

- [ ] **Step 3: Commit and push the branch; confirm CI goes green**

```bash
git add .github/workflows/build-firmware.yml
git commit -m "ci: build sensor and console BL targets, publish artifacts"
git push -u origin feature/initial-port
```
Expected: Actions run succeeds for both targets. Watch with `gh run watch`.

---

## Phase 7 — App-side integration (minimal)

> These tasks happen in the **app repos**, each on a feature branch off `next`. They are required for the apps to actually boot through the BL. Verify the exact app-side boot-state contract against `openlifu-console-fw` on GitHub before writing Step-level code (the BL expects the app to set BOOT_OK and to request DFU via the `'DFU!'` backup-register magic).

### Task 17: sensor-fw — relink app to 0x08040000 and set VTOR

**Files (in `openmotion-sensor-fw`):**
- Modify: `STM32H743VIHX_FLASH.ld`
- Modify: `Core/Src/system_stm32h7xx.c` (or wherever VTOR is set)

- [ ] **Step 1: Branch** `git checkout next && git pull && git checkout -b feature/bootloader-integration`.

- [ ] **Step 2: Edit the linker FLASH region**

```
  FLASH (rx)     : ORIGIN = 0x08040000, LENGTH = 1408K
```
(1408K = `0x081A0000 − 0x08040000`; leaves the FPGA bitstream sector untouched.)

- [ ] **Step 3: Set VTOR** in `SystemInit()` — add `#define USER_VECT_TAB_ADDRESS` with `VECT_TAB_OFFSET 0x00040000`, or explicitly `SCB->VTOR = 0x08040000;`. Confirm the existing `system_stm32h7xx.c` VTOR block and set the offset accordingly.

- [ ] **Step 4: Build sensor-fw**, confirm `.map` shows vectors at `0x08040000` and the image fits in 1408K.

Run: `cmake --preset Release && cmake --build build/Release`

- [ ] **Step 5: Commit**

```bash
git add STM32H743VIHX_FLASH.ld Core/Src/system_stm32h7xx.c
git commit -m "feat: relink application to 0x08040000 for bootloader"
```

---

### Task 18: sensor-fw — replace DFU-entry with backup-register request

**Files (in `openmotion-sensor-fw`):**
- Modify: the `OW_CMD_DFU` handler (`Core/Src/if_commands.c` per exploration) and the main-loop DFU jump

- [ ] **Step 1: Find the current DFU entry** (`_enter_dfu` flag, SRAM4 `0xDEADBEEF`, jump to `0x1FF09800`).

- [ ] **Step 2: Replace the action** so that on `OW_CMD_DFU` the app writes the `'DFU!'` magic (`0x21554644`) to `RTC->BKP1R` (enabling backup-domain access first) and calls `NVIC_SystemReset()`. The BL's `bs_should_enter_dfu` catches it. Keep the ROM-DFU `0x1FF09800` jump available behind a separate, explicit "deep recovery" command as a fallback (optional this task).

- [ ] **Step 3: Add a BOOT_OK signal** — once the app has initialized successfully (e.g. USB enumerated / main loop entered), clear BOOT_IN_PROGRESS / set BOOT_OK in `RTC->BKP2R` per the BL contract so the crash-loop counter resets. (Match the bit layout in `boot_state.c`.)

- [ ] **Step 4: Build sensor-fw**, clean.

- [ ] **Step 5: Commit**

```bash
git commit -am "feat: request bootloader DFU via backup register; signal BOOT_OK"
```

---

### Task 19: sensor-fw — CI signing step

**Files (in `openmotion-sensor-fw`):**
- Modify: `.github/workflows/build-firmware.yml`

- [ ] **Step 1: Add a signing step** after the firmware build that runs the BL repo's `dfu-test.py pack-signed` against the built `.bin`, producing `openmotion-sensor-fw.signed.bin`. Source the signing private key + trust key from GitHub Actions secrets (never committed). Vendor `dfu-test.py` into the app repo or `pip install` it from the BL repo — choose vendoring (copy `dfu-test.py` into `scripts/` of the app) for hermetic CI.

- [ ] **Step 2: Publish the signed artifact** alongside the existing hex/bin in the release job.

- [ ] **Step 3: Commit and push; confirm CI green.**

```bash
git commit -am "ci: produce signed firmware artifact for bootloader updates"
git push -u origin feature/bootloader-integration
```

---

### Task 20: console-fw — mirror the integration

**Files (in `openmotion-console-fw`):** linker, VTOR, DFU-entry handler, CI — same changes as Tasks 17-19 with console addresses (app length 1536K to `0x081C0000`).

- [ ] **Step 1: Branch off `next`**, apply the Task 17-19 changes with console's `APPLICATION_MAX_SIZE` (1536K) and no bitstream region.
- [ ] **Step 2: Build console-fw**, clean; `.map` vectors at `0x08040000`.
- [ ] **Step 3: Commit and push the console branch; confirm CI green.**

---

## Phase 8 — Bench validation (sensor first; flash gated)

### Task 21: Dry-run the two-stage flow in simulation/offline

**Files:** none (validation).

- [ ] **Step 1: Build the BL + a signed sensor app offline** and confirm with `dfu-test.py`:
  - `pack-signed` the relinked sensor-fw `.bin` at `0x08040000`.
  - `verify-package` passes with the dev pubkey + trust key.
- [ ] **Step 2: Sanity-check the combined image addresses** — BL at `0x08000000`, metadata at `0x08020000`, app at `0x08040000`, bitstream still at `0x081A0000`. No overlaps. Confirm via the `.map` files and `dfu-test.py`'s manifest/hex output.

### Task 22: First hardware flash — **STOP, confirm with user**

**Files:** none (hardware).

- [ ] **Step 1: HARD GATE.** Present the bench plan to the user and get explicit go-ahead before flashing sector 0. Recovery if the BL is bad requires BOOT0-pin/ST-Link, so confirm a recovery path is physically available on the bench unit.
- [ ] **Step 2 (after approval): Flash the BL** to `0x08000000` via STM32CubeProgrammer over ROM DFU (`OW_FPGA_OFF` + ROM bootloader entry, per the sensor-fw deploy flow), then flash the signed app metadata + app region.
- [ ] **Step 3: Power-cycle (Shelly)** and observe: BL UART/LED indicates boot, app starts (camera/USB enumerate as normal).
- [ ] **Step 4: Test signed update through BL DFU** — issue the app's new DFU-request command, confirm the device re-enumerates as `OMOTION SENSOR BL DFU`, push a new signed app, verify it boots.
- [ ] **Step 5: Test rejection** — push a tampered/unsigned image, confirm the BL refuses to jump and stays in DFU.
- [ ] **Step 6: Test crash-loop fallback** — flash an app that hangs before BOOT_OK, confirm two IWDG resets latch FORCE_DFU and the BL drops to DFU.
- [ ] **Step 7: Document results** in the BL repo README / a bench-report note; capture UART logs.

---

## Phase 9 — Docs and finish

### Task 23: Write the BL repo README and merge

**Files:**
- Modify: `$BL/README.md`

- [ ] **Step 1: Write the README** modeled on `$REF/README.md`: security model, H743 memory map, metadata v3, boot flow, DFU behavior, key material, build (four presets), and the production-style flashing sequence with H743 addresses.

- [ ] **Step 2: Commit, open PRs.**
  - BL repo: PR `feature/initial-port` → `main` (new repo; `main` is the default).
  - sensor-fw: PR `feature/bootloader-integration` → `next`.
  - console-fw: PR → `next`.

- [ ] **Step 3:** Use the `superpowers:finishing-a-development-branch` skill to choose merge/PR handling per repo.

---

## Self-review notes

- **Spec coverage:** repo+targets (T1-3), memory map opt-1 bitstream region (T4, T12), crypto+HMAC+ECDSA (T5-6, T8), backup-register crash-loop (T7), VTOR jump (T8), per-target USB HS/FS (T9-11), DFU region guards/BL-sector protection (T12), host tooling + CRC parity (T13-14), dev keys (T15), CI (T16), app-side relink/VTOR/DFU-request/signing for both repos (T17-20), bench tests incl. tamper + crash-loop with the flash gate (T21-22), README (T23). Open spec items (app boot-state contract, DfuSe vs dfu-util, ULPI timing) are called out at T7/T12/T17 for implementation-time resolution.
- **CRC parity caveat:** Task 14 flags that the STM32 default CRC is the non-reflected variant (`zlib.crc32` won't match); the tool must implement the device's algorithm and the expected value is pinned during implementation. This is the highest-risk correctness item — verify it before trusting any signed image.
- **Pseudocode flagged:** the CMake git-describe guard in Task 3 Step 8 has a deliberately-flagged stray paren to rewrite cleanly; `boot_state.c`'s `BKP(i)` token-paste is flagged to be implemented as a pointer accessor on-target.
```
