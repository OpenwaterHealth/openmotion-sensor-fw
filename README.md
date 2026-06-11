# Openwater Open-Motion Sensor Module v1.0.0
This repository contains the firmware for the Motion Sensor Module.

# Build & Flash

Requires arm-none-eabi-gcc, CMake 3.22+, and Ninja (or use the `ghcr.io/openwaterhealth/stm32-build-env:latest` Docker image). The configure step downloads the latest FPGA bitstream from GitHub releases; to work offline, pre-place it at `fpga/openmotion-camera-fpga.bin`.

```powershell
cmake --preset Debug
cmake --build build/Debug    # → build/Debug/motion-sensor-fw.{elf,hex,bin}
```

Flash a connected sensor over USB DFU (no ST-LINK needed):

```powershell
python scripts/deploy.py --device left    # or right; see --help for options
# equivalent: cmake --build build/Debug --target flash-left
```

The script enters DFU mode through the `omotion` SDK, flashes the merged firmware + FPGA image, and waits for the sensor to re-enumerate. See `scripts/deploy.py --help` for options.

# Driver Install
This device is a USB composite device with three interfaces and we are required to install the Latest WinUSB Driver for COMMS_HISTO_IMU(HS)

## Driver Utility
For the Host machine to communicate with the sensor module from user-space, each interface must be bound to the Microsoft WinUSB driver provided by Windows 10/11 (latest in-box version).

[Download Zadig](https://zadig.akeo.ie/)

### Associate drivers for COMMS_HISTO_IMU

Connect the Console with Sensor Module(s) attached to your computer.

Run Zadig and select the first interface (two will be listed only need to select one)

Select COMMS_HISTO_IMU(HS) (Interface 0)

![Select Interface 0 in Zadig](docs/if-0-select.png)

Click Install Driver after a little bit you will see success and the driver associated

![Installing](docs/installing.png)
![success](docs/success.png)

After successful install you will see that the interface is now associated with WinUSB

![Select Interface 0 success](docs/if-0-success.png)

Do the same steps for Interface 1 and Interface 2

![Select Interface 1 in Zadig](docs/if-1-select.png)
![Select Interface 2 in Zadig](docs/if-2-select.png)

Once completed close Zadig and you can now run the Open-MOTION software.

TODO:
For Windows to auto-load the latest WinUSB without any INF install, you can add Microsoft OS 2.0 descriptors to your firmware with Compatible ID = WINUSB for each interface. On Win8.1+, it will just bind to the in-box WinUSB automatically.
