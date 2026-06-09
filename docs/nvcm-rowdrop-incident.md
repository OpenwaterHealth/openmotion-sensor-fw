# NVCM Row-Drop Incident — 2026-06-09

## Summary

Cameras 1–7 on the left sensor module were NVCM-burned with a row-shifted
image and **permanently cannot auto-boot from NVCM** (NVCM is one-time
programmable). They remain fully functional via SRAM programming — the
production path — so the module is not degraded for normal use. Camera 8
(burned the same morning) is correct and auto-boots.

## Root cause (proven on hardware)

`openmotion-sdk/omotion/i2c_parser.py` — the Python port of the Lattice
ispVME I2C engine — never implemented TDO/DTDO readback comparison:

1. **Polling loops didn't poll.** The `LSC_CHECK_BUSY` (0xF0) read between
   NVCM row programs returned its result into a comparison that didn't
   exist, so every polling loop exited on its first iteration. The only
   inter-row pacing was USB round-trip latency.
2. **VERIFY verified nothing.** The verify phase read every row back and
   discarded the data. "PASSED" carried no information about content.

The first NVCM row program is the slowest (charge-pump ramp-up). With no
busy-wait, the row-1 write arrived while the FPGA was still programming
row 0 and was silently ignored; the dropped row gave the device time to
catch up, so rows 2+ landed — each one address early. Result: the
device-check row (`01 2C 00 43 ...`) missing, boot ROM rejects the image.

Camera 8's burn survived on incidental latency, not different code.
**The sensor firmware factory path (`if_factory_prog.c`) was verified
correct and was never at fault** — including the firmware changes on
`feature/nvcm-autodetect`, which do not touch the factory path.

## Proof

Re-burning sacrificial camera 2 with a 5 ms pace after each row-program
transaction produced the predicted OTP-OR signature at the correct
address:

```
before: addr1 = 82 91 15 F8 ...            (shifted: file row 2)
after:  addr1 = 83 BD 15 FB 22 ... 3C 46   (= row2 | row1, all 11,970 rows landed)
```

Artifacts in `docs/captures/nvcm-rowdrop-debug/`: simulated transaction
stream, paced-burn test script, failing-verify log.

## Fix

`openmotion-sdk` branch `fix/i2c-parser-verify`:

- `ispVMRead` performs real masked comparison against expected TDO/DTDO
  bytes (`ERR_VERIFY_FAIL` on mismatch); simulation drivers keep the old
  pass-through.
- `ispVMLoop` returns the last failure when the loop count is exhausted
  instead of success.

Verified on hardware with the fixed parser: the IDCODE comparison passes,
busy polls observably retry, and the algorithm's blank-check correctly
**refuses to program a Done-set device** (it failed fast on camera 3, as
it should). Unit tests: `tests/test_i2c_parser_verify.py`.

## Hardware state after incident

| Camera | NVCM state | Auto-boot | Usable via SRAM |
|---|---|---|---|
| 1–7 (left) | row-shifted + OR'd garbage (cams 2, 3 additionally OR'd by diagnostic re-burns) | **never** | yes |
| 8 (left) | correct image + Done | yes | yes |

## Positive validation (2026-06-09, virgin module in slot 8)

A brand-new camera module was installed in left slot 8 and burned with
the fixed parser:

1. `test_factory_prog.py` PASSED with full readback verification.
2. Probe verdict: **NVCM PROGRAMMED** — 0x40 disappears after CRESETB
   release, i.e. the FPGA auto-boots its user design.
3. The Done fuse took in the same pass — no separate
   `nvcm_burn_done.py` retry needed (the real busy-polling also fixed
   the ISC_PROGRAM_DONE wait that bit the first camera-8 burn).
4. Firmware pin-sampling detect: `C8: NVCM detect clk=0 data=0` →
   "NVCM programmed, skipping SRAM load".

## Outstanding

- Burns are slower with real polling (real busy-waits + full verify).
- Unexplained: one USB drop (errno 32) during an all-8-cameras
  `OW_FPGA_PROG_SRAM` detect loop earlier the same day. Not reproduced or
  investigated yet; tracked separately from this incident.
