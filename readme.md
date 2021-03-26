RINGULARITY
Smart Ring Operating Environment

License: MIT

Introduction
This is created for the NRF52x series of BLE SOCs to enable the rapid development of smart rings (and other BLE devices that use supported types of sensors)

Dev environment setup and building
This has been developed using nRF5 SDK 17.0.2 and s132_nrf52_7.2.0_softdevice.hex for the SoftDevice.

After downloading the ringularity source code copy the following directories from the nRF5 SDK to the directory containing the ringularity firmware project:
* components
* external
* integration
* modules

Project can be built in both Release and Debug modes

The code currently contains support for a small number of peripheral devices, the remainder of which are currently stubbed out with device simulators.
* SOC
* Maxim MAX30101 (tested with TinyCircuits pulse oximeter AST1041)

One of the concepts in effect here is to make this easily configurable to customize the firmware for different BLE services, BLE settings and different hardware.

Concepts and Terminology
* Service Controller: provides a binding between a system device and a BLE services
* Device: usually refers to a peripheral piece of hardware, such as a sensor, but may also refer to various aspects of the SOC.  Some devices are currently implemented as simulators.
* Device API: an API providing low-level peripheral access, and may also be referred to as a device driver
* System Device: provides an abstraction of the low-level device APIs and provides means for the service controllers to interact with the devices

SDK configuration is done through sdk_config.h
BLE configuration is done through ble_config.h
Hardware configuration is done through hw_config.h

BLE Configuration (ble_config.h)
1) General Access Profile (GAP) settings
2) Device Information Service (DIS) settings
Finalization of SIG value support is left up to the developer
3) Advertising settings
4) Bond management services
5) Various system event handlers to receive dispatched events

Hardware Configuration (hw_config.h)
1) used to include system device headers
2) contains the system device declarations
3) contains the definition of the macros used to instantiate the system devices on startup
4) SYSTEM_MODE: 
  1) simulation mode to provide hardware simulation when a device is not present
  2) physical mode for when hardware will be in use
  3) test mode - not implemented - used to put the system into a mode for test automation
  
Supported system device types:
* battery/power
* inertial measurement unit (IMU)
* temperature sensor
* pulse oximeter
* TWI (I2C)