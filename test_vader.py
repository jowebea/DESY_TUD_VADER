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
print("init Driver...")
driver = VaderDeviceDriver(MINI1_PORT, MINI2_PORT, MAXI_PORT)
time.sleep(10)
print("Setting CO2 to 10 nl/min...")
driver.maxi._send_only(2, 30, 10)
time.sleep(2)
print("Setting N2 to 0 nl/min...")
driver.maxi._send_only(2, 20, 0)
time.sleep(2)
print("Setting Butan to 0 nl/min...")
driver.maxi._send_only(2, 40, 0)
time.sleep(2)
for i in range(100):
    print("applying pressure...")
    driver.mini2._send_raw("2; 3; 500")
    time.sleep(2)
    print(f'Pressure: {driver.mini1.get_last_n(1)}')
    print("applying Vacuum...")
    driver.mini2._send_raw("2; 3; 0")
    time.sleep(8)
    print(f'Pressure: {driver.mini1.get_last_n(1)}')
