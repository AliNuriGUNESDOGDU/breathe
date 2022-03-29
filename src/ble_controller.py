import bluepy.btle as btle


class MyDelegate(btle.DefaultDelegate):
    def __init__(self, params):
        btle.DefaultDelegate.__init__(self)
        # ... initialise here

    def handleNotification(self, cHandle, data):
        pass
        # ... perhaps check cHandle
        # ... process 'data'

        # Initialisation  -------


p = btle.Peripheral("CC:4A:71:9E:EE:15")
p.setDelegate(MyDelegate(1))

# Setup to turn notifications on, e.g.
#   svc = p.getServiceByUUID( service_uuid )
#   ch = svc.getCharacteristics( char_uuid )[0]
#   ch.write( setup_data )

# Main loop --------

while True:
    if p.waitForNotifications(1.0):
        # handleNotification() was called
        continue

    print "Waiting..."
    # Perhaps do something else here
