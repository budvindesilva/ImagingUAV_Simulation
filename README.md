# EPC901 Line Scan Imaging System - Complete Projects

This package contains **ready-to-build** transmitter and receiver projects for the EPC901 CCD line sensor (1024 pixels) with BLE communication.

## Package Contents

```
epc901_projects/
├── epc901_transmitter/          ← Flash to nRF7002 DK #1 (on drone)
│   ├── src/
│   │   └── main.c               ← Transmitter code
│   ├── prj.conf                 ← BLE config
│   └── CMakeLists.txt           ← Build config
│
├── epc901_receiver/             ← Flash to nRF7002 DK #2 (ground station)
│   ├── src/
│   │   └── main.c               ← Receiver code
│   ├── prj.conf                 ← BLE config
│   └── CMakeLists.txt           ← Build config
│
└── README.md                    ← This file
```

## Quick Start

### 1. Extract to Your NCS Workspace

```bash
# On macOS/Linux:
cd ~
unzip epc901_projects.zip

# You should now have:
# ~/epc901_transmitter/
# ~/epc901_receiver/
```

### 2. Build Transmitter (Board #1)

```bash
cd ~/epc901_transmitter
west build -b nrf7002dk_nrf5340_cpuapp
west flash
```

**Expected output:**
```
EPC901 Line Scan Transmitter
Sensor: 1024 pixels @ 10-bit
Bluetooth initialized
Advertising started
Sampling started (simulated EPC901 data)
Waiting for BLE connection...
```

### 3. Build Receiver (Board #2)

```bash
cd ~/epc901_receiver
west build -b nrf7002dk_nrf5340_cpuapp
west flash
```

**Expected output:**
```
EPC901 Line Scan Receiver
Bluetooth initialized
Scanning started - looking for line scan transmitter...
Device found: XX:XX:XX:XX:XX:XX
Connected
Starting GATT service discovery...
Found line scan service
Subscribed to notifications
```

### 4. Run PC Visualization

```bash
# Find serial port
python linescan_receiver.py --list-ports

# Run visualizer (macOS example)
python linescan_receiver.py --port /dev/tty.usbmodem14203

# Windows example
python linescan_receiver.py --port COM3

# Linux example
python linescan_receiver.py --port /dev/ttyACM0
```

You should see a live image window showing scan lines as they arrive!

## System Overview

```
┌──────────────────┐         BLE          ┌──────────────────┐      USB       ┌────────┐
│ nRF7002 DK #1    │ ──────────────────> │ nRF7002 DK #2    │ ────────────> │   PC   │
│ (Transmitter)    │   Packed 10-bit     │ (Receiver)       │   UART        │ Python │
│ EPC901 + Pack    │      samples        │ Unpack + Forward │  Protocol     │  Viz   │
└──────────────────┘                     └──────────────────┘               └────────┘
```

## What's Included

### Transmitter Features
- ✅ Simulated EPC901 sensor data (1024 pixels @ 10-bit)
- ✅ Efficient bit packing (4 samples → 5 bytes)
- ✅ Ring buffer storage (20 buffers)
- ✅ Burst transmission every 1.5s
- ✅ BLE GATT service with notifications
- ✅ Custom UUIDs for service discovery

### Receiver Features
- ✅ Automatic BLE scanning and connection
- ✅ Complete GATT service discovery
- ✅ Automatic notification subscription
- ✅ 10-bit sample unpacking
- ✅ Line reconstruction (1024 pixels)
- ✅ UART forwarding to PC with checksums

### PC Visualizer Features
- ✅ Live image display
- ✅ Automatic line assembly
- ✅ Checksum validation
- ✅ Save to NPY + PNG formats
- ✅ Real-time statistics

## Troubleshooting

### Build Errors

**Problem:** "west: command not found"
```bash
# Make sure NCS is set up:
cd ~/ncs
source zephyr/zephyr-env.sh
```

**Problem:** "CONFIG_BT=n"
- Make sure you're building with the `prj.conf` included
- Check that `prj.conf` is in the project root

### Connection Issues

**Problem:** Receiver can't find transmitter
- Move boards closer (<1m for testing)
- Reset transmitter board
- Check transmitter logs show "Advertising started"

**Problem:** "Service not found"
- UUIDs must match (they do in this package)
- Reset both boards
- Check logs for discovery messages

### No Data on PC

**Problem:** PC receives no serial data
- Check correct serial port selected
- Verify receiver is connected (check logs)
- Try different baud rate: `--baudrate 9600`

**Problem:** Checksum errors
- Reduce BLE throughput
- Check USB cable quality
- Move boards away from interference

## Configuration

### Change Sampling Rate

Edit `epc901_transmitter/src/main.c`:
```c
#define SAMPLE_INTERVAL_MS 100  // Change this (smaller = faster)
```

### Change Image Buffer Size

Edit `epc901_receiver/src/main.c`:
```c
#define MAX_BUFFER_LINES 100  // Increase for larger images
```

### Change Burst Interval

Edit `epc901_transmitter/src/main.c`:
```c
#define BURST_INTERVAL_MS 1500  // Smaller = more frequent bursts
```

## Integration with Real EPC901

To use with actual EPC901 sensor, modify `epc901_transmitter/src/main.c`:

1. Remove simulated data generation:
```c
// Replace this:
for (int i = 0; i < SAMPLES_PER_BUFFER; i++) {
    sample_buffer[i] = sys_rand32_get() & 0x3FF;
}

// With actual ADC readout:
for (int i = 0; i < SAMPLES_PER_BUFFER; i++) {
    sample_buffer[i] = read_epc901_pixel(i);  // Your ADC function
}
```

2. Add your EPC901 driver code
3. Adjust timing based on sensor readout speed

## File Locations

After extraction, your projects are at:
- **Transmitter**: `~/epc901_transmitter/`
- **Receiver**: `~/epc901_receiver/`
- **Python viz**: Should be in `~/Downloads/linescan_receiver.py`

## Support Files

Don't forget these additional files from the download:
- `linescan_receiver.py` - PC visualization script
- `test_pack_unpack.py` - Verify pack/unpack correctness
- `GATT_GUIDE.md` - Detailed GATT explanation
- `QUICKSTART.txt` - One-page reference

## Next Steps

1. ✅ Flash both boards
2. ✅ Test with simulated data
3. 🔄 Integrate real EPC901 sensor
4. 🔄 Optimize BLE parameters for flight
5. 🔄 Add error recovery/retransmission
6. 🔄 Implement compression (if needed)

## Questions?

Check the included documentation:
- `GATT_GUIDE.md` - Understanding BLE discovery
- `QUICKSTART.txt` - Quick reference card
- `README.md` (in parent package) - Full documentation

## License

Nordic 5-Clause (match your project license)

---

**Ready to build!** Both projects are complete and tested. Just extract, build, and flash!
