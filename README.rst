
nRF5340 Audio
#############


Overview
********
This demo is based on ncs2.4.0 nrf5340_audio application(https://github.com/nrfconnect/sdk-nrf/tree/main/applications/nrf5340_audio).
Adding nus(Nordic Uart Service) in Gateway when running on BIS mode.

Implement
=================
*Build the application for gateway bis mode.
	west build -b nrf5340_audio_dk_nrf5340_cpuapp -d build_bis_gateway--pristine -- -DCONFIG_AUDIO_DEV=2
*Flash both application core and network core on the gateway board.
*Press PLAY/PAUSE Button to pause audio stream.
*Scan the two advertise name as NRF5340_AUDIO using nrf connect app on mobile phone, one is for transmit audio stream using priodic adv(non-connectable), the other one(connectable) is for ACL connection.
*Connect to the board, transmit ble data as usaul.
*Press PLAY/PAUSE Button again to play audio stream.

Notice
====================
Stop playing audio stream by pressing PLAY/PAUSE Button everytime you want to re-connect the device on the phone.
