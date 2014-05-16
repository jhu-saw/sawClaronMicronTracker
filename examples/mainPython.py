import os
import platform
import sys
import time

if platform.system() == 'Linux':
    sys.setdlopenflags(sys.getdlopenflags() | 8)  # RTLD_GLOBAL = 0x08
import cisstMultiTaskPython as mts

name = 'My Tracker'
period = 0.01
configuration = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'configClaronMicronTracker.xml')

manager = mts.mtsManagerLocal.GetInstance()
manager.CreateAllAndWait(5.0)
manager.StartAllAndWait(5.0)

proxy = mts.mtsComponentWithManagement('{}Proxy'.format(name))
manager.AddComponent(proxy)
proxy.CreateAndWait(5.0)
time.sleep(0.5)

services = proxy.GetManagerComponentServices()
result = services.Load('sawClaronMicronTracker')
assert result, 'Failed to load {} using component services'.format('sawClaronMicronTracker')

args = mts.mtsTaskPeriodicConstructorArg(name, period)
result = services.ComponentCreate('mtsMicronTracker', args)
assert result, 'Failed to create {} of type {}'.format(name, 'mtsMicronTracker')

component = manager.GetComponent(name)
# component.Configure(configuration)

controller = proxy.AddInterfaceRequiredAndConnect((name, 'Controller'))

component.CreateAndWait(5.0)
component.StartAndWait(5.0)

print controller.IsTracking()
