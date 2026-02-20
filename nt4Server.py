from ntcore import NetworkTableInstance
import sys
import ntcore
import time

inst: NetworkTableInstance = NetworkTableInstance.getDefault()

inst.startServer()
print("Version " + ntcore.__version__)

try:
    while True:
        time.sleep(100)
except Exception as e:
    print(e)

inst.stopServer()
