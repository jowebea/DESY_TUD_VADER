import logging
import time
from VaderDeviceDriver import VaderDeviceDriver

print("testing VaderDeviceDriver...")
print("assigning Ports...")
# macOS: ersetze durch deine Ports (siehe ls /dev/cu.*)
MINI1_PORT = "/dev/ttyACM0"
MINI2_PORT = "/dev/ttyACM2"
MAXI_PORT  = "/dev/ttyACM1"

logging.basicConfig(level=logging.DEBUG)
print("##########################")
print("#### init Driver... ######")
print("##########################")
driver = VaderDeviceDriver(MINI1_PORT, MINI2_PORT, MAXI_PORT)
time.sleep(10)
try:
    driver.get_all_status()
    print("##########################")
    print("Setting N2 to 5nl/min...")
    print("##########################")
    driver.use_gas(n2=5, co2=0, butan=0)
    time.sleep(0.2)
     
    print("##########################")
    print("Closing V4 (to let it cool)...")
    print("##########################")
    driver.maxi.activate_v4(False)
    time.sleep(0.2)

    print("Status:", driver.get_all_status(mini1_last_n=2))
    time.sleep(0.2)

finally:
    driver.close()