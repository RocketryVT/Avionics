# Avionics

<p align="center">
  <img width="200" alt="RocketryLogo_White_MaroonBackground" src="https://github.com/user-attachments/assets/2b8f0c37-6ac3-4bb0-baf4-3e040de47cc5" />
</p>

This repository contains the avionics projects developed by the Rocketry at Virginia Tech team. We build flight computers, tracking systems, and ground support equipment for high-power rockets.

## About

The avionics subteam designs electronic systems for our rockets! Our work ranges from active flight control systems to mobile trackers that monitor rockets throughout their missions.

## Projects

### [Active Drag System](https://github.com/RocketryVT/active-drag-system)

This is our primary flight computer that controls air brakes to hit a precise 10,000ft apogee. The system uses sensor fusion from an IMU, barometer, GPS, and magnetometer to estimate the rocket's state in real-time. A Kalman filter processes this data to predict the flight path, then controls four servo-driven air brakes to add drag as needed. All telemetry is transmitted via LoRa during flight. The hardware is a custom PCB designed in KiCad.

### [Mobile Tracker](https://github.com/RocketryVT/Mobile-Tracker)

A handheld device that creates a LoRa mesh network for rocket tracking. It receives packets from all our LoRa devices since they share the same frequency and settings. The device has its own GPS and screen for standalone operation, but can also connect via Bluetooth to our mobile apps. When connected to a phone, it relays the phone's GPS location across the LoRa network and displays any received telemetry data on the phone's screen.

### [Ground Station](https://github.com/RocketryVT/Ground-Station)

A QT-based application that controls our antenna tracking system. It uses servo motors to automatically point directional antennas at the rocket as it moves through its flight path. The software plots the rocket's position in real-time and maintains communication links by keeping antennas properly oriented.

### [Bareman Tracker](https://github.com/RocketryVT/Bareman-Tracker)

A smaller version of our active drag system without the servo control hardware. It includes the same sensor suite (IMU, magnetometer, barometer, GPS) and LoRa transmission capabilities but in a more compact form factor optimized for tracking rather than active control. The reduced size and power requirements make it suitable for smaller rockets or as a backup tracking system.

### Mobile Applications

We've developed native apps for both [Android](https://github.com/RocketryVT/Android-Rocket-Tracker) and [iOS](https://github.com/RocketryVT/iOS-Rocket-Tracker) that connect to our Mobile Tracker devices over Bluetooth. The apps display real-time flight data, log all received packets to a local SQLite database, and provide GPS data back to the LoRa network through the connected Mobile Tracker.

## How It All Works Together

Our systems form an integrated network for rocket operations. Flight computers collect sensor data and transmit it over LoRa during flight. Mobile Trackers on the ground receive this data and can relay it through the mesh network. The Ground Station automatically tracks the rocket's position to maintain communication links. Mobile apps provide a portable interface to monitor flights and log data for analysis.

## Repository Contents

This repository includes the projects listed above along with flight data from our test launches and IREC competitions in the `flight_data/` directory. Documentation for all systems is maintained in the `docs/` folder using Astro.

Each project has its own repository with detailed build instructions and usage documentation. Check the individual project links above to get started with any specific system.

---

[Rocketry at Virginia Tech](https://rocketryatvirginiatech.org/) | [GitHub Organization](https://github.com/RocketryVT)
