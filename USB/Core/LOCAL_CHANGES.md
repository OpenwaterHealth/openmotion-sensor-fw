# Local changes to the STM32 USB Device Library core (SOUP)

`USB/Core` is the CubeMX-generated copy of the STMicroelectronics USB Device Library
core (`usbd_core.c`, `usbd_ctlreq.c`, `usbd_ioreq.c`). Vendor code; every deviation
from upstream is listed here and in the SOUP register. Keep this file next to the code.

| File | Change | Why | Reference |
|---|---|---|---|
| `Src/usbd_ctlreq.c` `USBD_StdEPReq()` | Reject endpoint requests whose endpoint number (`wIndex & 0x7F`) is 16 or more before it indexes `ep_in[]`/`ep_out[]` (16 entries). | A host can send any `wIndex`; `GET_STATUS` in the configured state wrote `pep->status` through the out-of-range index. Upstream fixed this in v2.11.6. | CVA 2026-09-28 O2 / R7; PTR-2026-1-5 |
| `Src/usbd_ctlreq.c` `USBD_GetString()` | Bound the UTF-16 copy loop by the clamped `*len`, not only by the source string. | A string descriptor longer than `USBD_MAX_STR_DESC_SIZ` overran the unicode buffer although the length had been clamped. Upstream fixed this in v2.11.6. | CVA 2026-09-28 O2 / R7 |

Both changes are marked `LOCAL CHANGE (Openwater, ...)` in the source and are the same
as in `openmotion-console-fw` (`Middlewares/ST/STM32_USB_Device_Library/LOCAL_CHANGES.md`).
Re-apply them, or move to upstream v2.11.6 or later, whenever CubeMX regenerates these
files.
