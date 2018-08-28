# ESP32-RMT-IR-Library for LTTO Lazertag

This library allows sending and receiving LTTO/LTX/LTAR Lazertag data using the RMT hardware in the ESP32.

There are 8 channels of RMT hardware in the ESP32, so you can create 8 different instances of the class to allow for multiple simultaneous Tx and Rx.
e.g. 	1 x Tx for beacons
	1 x Tx for tags
	1 x Rx focussed on tagger for IFF
	1 x Rx facing fwd
	1 x Rx facing left
	1 x Rx facing back
	1 x Rx facing right
	Depending which one/s of the last 4 Rx devices receive a tag
	will allow the direction of the source to be determined.
