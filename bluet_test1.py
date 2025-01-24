# simple inquiry example
import bluetooth

nearby_devices = bluetooth.discover_devices(duration=5, flush_cache=False, lookup_names=True, lookup_class=False, device_id=-1)
print("Found {} devices.".format(len(nearby_devices)))

for addr, name in nearby_devices:
    print("  {} - {}".format(addr, name))
