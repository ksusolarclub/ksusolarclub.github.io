import asyncio
import signal
import sys
from bleak import BleakScanner, BleakClient

SERVICE_UUID                = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" 
CHARACTERISTIC_UUID_RX      = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
CHARACTERISTIC_UUID_TX      = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
CHARACTERISTIC_UUID_INVOLT  = "863b8b12-c8fc-11ed-afa1-0242ac120002"
CHARACTERISTIC_UUID_INCURR  = "8c3bf006-c8fc-11ed-afa1-0242ac120002"
CHARACTERISTIC_UUID_BATVOLT =  "90dcd83c-c8fc-11ed-afa1-0242ac120002"
CHARACTERISTIC_UUID_BATCURR =  "978f77d4-c8fc-11ed-afa1-0242ac120002"
CHARACTERISTIC_UUID_LOADCURR = "9cdf32f6-c8fc-11ed-afa1-0242ac120002"
CHARACTERISTIC_UUID_REFVOLT  = "a0e65820-c8fc-11ed-afa1-0242ac120002"
CHARACTERISTIC_UUID_DUTY     = "384614d8-c927-11ed-afa1-0242ac120002"
CHARACTERISTIC_UUID_STATE    = "3e346c8c-c927-11ed-afa1-0242ac120002"

async def handle_notification(sender, data):
    # Print the value of the characteristic that triggered the notification
    print(f"Value of characteristic {sender}: {data}")

async def run():
    # Scan for nearby BLE devices
    print("Scanning for BLE devices...")
    devices = await BleakScanner.discover()

    # Print a list of nearby devices and prompt the user to select one
    print("Available devices:")
    for i, device in enumerate(devices):
        print(f"{i+1}. {device.name} ({device.address})")

    selection = None
    while selection is None:
        try:
            index = int(input("Enter the number of the device you want to connect to: "))
            if index < 1 or index > len(devices):
                raise ValueError()
            selection = devices[index-1]
        except ValueError:
            print("Invalid selection, please try again.")

    # Connect to the selected device and read the values of the characteristics
    async with BleakClient(selection.address) as client:
        # Connect to the device

        #service = await client.get_service(SERVICE_UUID)
        print("Client connected")

        try:
            while True:
                # Read the values of the characteristics
                #message = await client.read_gatt_char(CHARACTERISTIC_UUID_TX)
                #print(f"Input voltage: {message}")

                await client.start_notify(CHARACTERISTIC_UUID_TX, handle_notification)

                # Wait for a second before reading the values of the characteristics again
                #await asyncio.sleep(0.1)

                # Wait for notifications to be received
                await asyncio.Event().wait()
                
        except KeyboardInterrupt:
            # Disconnect from the device when the user stops the script
            print("\nDisconnecting from the device...")
            await client.disconnect()

loop = asyncio.get_event_loop()
loop.run_until_complete(run())
