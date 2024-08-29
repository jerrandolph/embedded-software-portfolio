# UART Data Plotter

This Python script reads data from a microcontroller over a UART connection and plots it in real-time using Matplotlib. It is designed to handle a continuous stream of data from the microcontroller, which is expected to send data packets with a specific sentinel value indicating the start of each packet.

## Features

- **Real-time Data Plotting**: Continuously plots data received from the UART connection.
- **Background Data Processing**: Handles UART data in a background thread to ensure smooth data acquisition and plotting.
- **Sentinel-Based Packet Alignment**: Ensures that the data is properly aligned using a sentinel value (`0xDEADBEEF`) at the start of each data packet.
- **Windowed Data Display**: Displays a fixed number of data points in a scrolling window to visualize recent data trends.

## Requirements

- Python 3.x
- Matplotlib
- PySerial

## Demonstration

<img src="./pendulum_opt_scale" alt="Python Plot" width="700"/>
