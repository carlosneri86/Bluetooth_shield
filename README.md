Bluetooth_shield
================
[http://www.seeedstudio.com/wiki/Bluetooth_Shield]: http://www.seeedstudio.com/wiki/Bluetooth_Shield

[Android Bluetooth App]: https://play.google.com/store/apps/details?id=mobi.dzs.android.BLE_SPP_PRO&hl=en

This sample code showcases the Seeed Studio Bluetooth shield with Freescale FRDM-KL25.

[http://www.seeedstudio.com/wiki/Bluetooth_Shield]

The sample code uses the following modules:
	* UART: Used for communication with the Bluetooth module
	* Timer: Used to generate SW timers
	* GPIO: For different modules
	
The bluetooth driver is written in a state machine fashion. Basically initializes the module and uses a status register to let know the application when data is received and when data has been transmitted. The driver handles connections and disconnections, a flag on the status register for connected devices.

The application is a simple loopback, waits for the BT driver to set the connected device flag, receives data and sends it back.

A heart beat LED (green) toggles every 5 seconds to let know the user the application is running.

CodeWarrior 10.6 is required for this sample code.

This [Android Bluetooth App] is used for testing, look for KL25_BT device. When pairing, use 0000 (four zeroes) as pairing code.