# Inverted Pendulum V2

## Objective
The second iteration of the inverted pendulum project aimed to improve upon the first version by using a more powerful microcontroller and experimenting with a different approach to balancing the pendulum.

## Equipment
- **Microcontroller:** STM32F446 Nucleo-64
  - More powerful than the MSP-430 used in V1.
  - Features a faster clock and a Floating Point Unit (FPU), making PID calculations faster and more accurate.
  - Exports telemetry data using UART.
  - Programmed using STM32CubeIDE.
  
- **Servo Motor:** Miuzei MG996R
  - Mounted at the top of the pendulum.
  - Uses a parallel linkage to shift itself left and right. The motor uses itself as a counterweight.
  - Used to control a counterweight instead of a reaction wheel.
  - Mounted at the top of the pendulum, shifting itself left and right via a parallel linkage to act as both the motor and the counterweight.
  - Removes the need for an H-Bridge.
  - According to the datasheet, at 7.4 V and no load it can rotate 60 degrees in .13 seconds.


- **Rotary Encoder:** Taiss KY-040
  - Used to measure the angular position of the pendulum.
  - Only about 30 distinct positions, so if tied directly to the pendulum arm, it only has a resolution of 12 degrees.
  - Used a motion amplification linkage to get more resolution out of the rotary encoder.

- **Pendulum Shaft:**
  - Constructed from Lego Technic bricks, making it lighter but more prone to vibrations.

- **Power Supply:**
  - Started witht he 5V power supply from the Nucleo-64 board, but eventually moved to a 9V battey.

## Design and Implementation

### Servo Motor for Counterweight
- Instead of using a reaction wheel, the servo motor moves a weight horizontally to tip the pendulum left and right.
- Due to the weight of the motor, it was mounted at the top of the pendulum and used a parallel linkage to shift itself left and right.

### Improved UART Protocol
- **Version 1 Protocol:** A 12-byte payload was used, consisting of 4 bytes to transmit `0xDEADBEEF`, 4 bytes for an integer, and 4 bytes for a float.
- **Version 2 Protocol:** The protocol was generalized to transmit any number of floating-point numbers after the initial `0xDEADBEEF` sentinel. The end of the sequence is marked by another `0xDEADBEEF` sentinel.

### Data Collection and Visualization
- **Data Transmission:** The ARM processor reads accelerometer data via an ADC module and transmits system information to a PC using UART.
- **Data Visualization:** A Python program on the PC collects the transmitted data and graphs it using Matplotlib.

## Challenges and Observations

### Pendulum Balancing Issues
- **Rotary Encoder Resolution**
  - The rotary encoder does not have a lot of resolution, even with the motion amplification linkate.

### PID Controller Considerations
- The `D` (derivative) calculation in the PID controller assumes a constant delta-time value between error calculation cycles. This could lead to inaccuracies if the time between cycles varies.
- **Possible Solutions:**
  1. Use interrupts to ensure a consistent time between calculations.
  2. Implement logic to feed in the actual time elapsed since the last calculation.

## Results and Future Improvements
- The pendulum is still unable to self-balance, but the project has provided valuable insights into the limitations of the current setup and areas for improvement.
- **Potential Improvements:**
  - Change out the rotary encoder for one with more resolution.
  - Increase the servo motor’s speed by using a higher voltage power supply.
  - Refine the PID controller logic to handle variable time steps more accurately.
