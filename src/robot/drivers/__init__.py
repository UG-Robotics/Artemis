"""One module per device (motor, servo, tof, imu, color, camera).

Vendor libs are imported lazily so this loads off-Pi; the hardware calls raise
NotImplementedError until we implement them.
"""
