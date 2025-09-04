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
time.sleep(20)
try:
    driver.get_all_status()
    print("Setting N2 to 5nl/min...")
    driver.use_gas(n2=5, co2=0, butan=0)
    time.sleep(0.2)
    driver.use_gas(n2=5, co2=0, butan=0)
    time.sleep(0.2)
    driver.use_gas(n2=5, co2=0, butan=0)
    time.sleep(0.2)

    print("Closing V4 (to let it cool)...")
    driver.maxi.activate_v4(False)
    time.sleep(0.2)

    print("Status:", driver.get_all_status(mini1_last_n=2))
    time.sleep(0.2)

    print("Setting Pressure setpoint to 50kPa...")
    driver.setpoint_pressure(50)
    time.sleep(0.2)
    driver.setpoint_pressure(50)
    time.sleep(0.2)
    driver.setpoint_pressure(50)
    time.sleep(0.2)

    print("Status:", driver.get_all_status(mini1_last_n=2))
    time.sleep(0.2)

finally:
    driver.close()