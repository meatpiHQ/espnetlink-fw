# Vendored: espressif/icm42670 2.1.0~1 (Apache-2.0, license.txt)

From https://github.com/espressif/esp-bsp `components/sensors/icm42670`
(commit 7198face). Vendored 2026-07-04 instead of consumed as a managed
component because its mandatory `sensor_hub` dependency (0.1.4) does not
compile on IDF v6 (`portTICK_RATE_MS` removed). Local changes, kept
minimal on purpose:

- removed `#include "iot_sensor_hub.h"`
- removed the sensor-hub plugin glue (`icm42670_impl_*`, lines 487-end)

Everything else is byte-identical upstream driver code. If a sensor_hub
release ever supports IDF v6, this can go back to being a managed
dependency unchanged.
