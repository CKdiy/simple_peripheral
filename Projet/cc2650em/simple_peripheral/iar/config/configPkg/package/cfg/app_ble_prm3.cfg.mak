# invoke SourceDir generated makefile for app_ble.prm3
app_ble.prm3: .libraries,app_ble.prm3
.libraries,app_ble.prm3: package/cfg/app_ble_prm3.xdl
	$(MAKE) -f E:\BeelinkerCode\simple_peripheral\Projet\cc2650em\simple_peripheral\iar\config/src/makefile.libs

clean::
	$(MAKE) -f E:\BeelinkerCode\simple_peripheral\Projet\cc2650em\simple_peripheral\iar\config/src/makefile.libs clean

