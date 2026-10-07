# Liquid Laser Pi

Raspberry Pi 3B project for Astra Pro depth interaction + particle liquid + LaserDock output.

## What this package includes

- Astra Pro via the working OpenNI2 ARM64 runtime
- 160x120@30 depth, centered square ROI
- CLI depth slab: `laser MIN_CM MAX_CM SMOOTH`
- depth mask -> SDF obstacle
- particle liquid simulation
- Marching Squares liquid contours
- less-fragile liquid defaults: cohesion .95, repulsion .65, threshold .34
- one 30Hz debug GUI
- LaserDock through the already-tested `laserdocklib send_samples()` path
- no runtime ringbuffer/control polling
- too-close/invalid depth => laser blank + fluid pause, not process exit
- Astra reconnect loop after sustained stream failure

## Expected dependency locations

```text
~/ros2_astra_camera_openni
~/laserdocklib
~/liquid-laser-pi
```

## Install

```bash
sudo apt update
sudo apt install -y build-essential cmake pkg-config libopencv-dev libusb-1.0-0-dev
cd ~/liquid-laser-pi
./scripts/install-local.sh
source ~/.bashrc
```

## Run

```bash
laser 100 110 2
```

Pi 3B lighter preset:

```bash
laser 80 100 2 --particles 160 --field 96 --substeps 1 --iterations 1
```

No laser:

```bash
laser 100 110 2 --no-laser
```

No GUI:

```bash
laser 100 110 2 --no-gui
```

Harder to break apart:

```bash
laser 100 110 2 --cohesion 1.2 --repulsion 0.5 --threshold 0.30
```

## GitHub workflow

Computer:

```bash
git add .
git commit -m "update"
git push
```

Pi:

```bash
laser-update
laser 80 100 2
```

Do not auto-pull during exhibition startup; keep update and run separate.

## Safety

Use low projector hardware power during testing and terminate the beam on a fixed non-reflective surface. Depth tracking is not a certified laser safety system.
