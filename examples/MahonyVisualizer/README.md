# Mahony Visualizer

Streams Mahony AHRS quaternions over serial and renders a 3D body-frame axis
in Python.

## Build (Teensy 4.1)
This example is wired as the default build for the `teensy41` environment.

## Run
1. Flash the board.
2. Install deps: `pip install -r requirements.txt`
3. Run the visualizer:
   `python examples/MahonyVisualizer/mahony_visualizer.py --port COM5`

The output format is:
`Q,w,x,y,z`

Rotate the board in 3D for a few seconds to collect mag calibration samples.
