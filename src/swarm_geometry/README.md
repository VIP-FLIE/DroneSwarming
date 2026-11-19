# swarm_geometry

`swarm_geometry` supplies C++ and Python geographic projection functions without PX4 message or topic dependencies. `mission_planner`, `mission_execution`, and `vehicle_control` use these calculations to resolve authored coordinates, check meter-based bounds, and project destinations at local-control boundaries.

The projection matches PX4 v1.17's spherical azimuthal equidistant map with Earth radius 6,371,000 meters. Geographic values use WGS84 latitude/longitude in degrees and AMSL altitude in meters. ENU values use east/north/up meters from an explicit geographic anchor. The vertical coordinate is the difference of AMSL heights. The projection handles longitude wrapping and rejects invalid or antipodal positions.

The C++ library exports `enu_to_global`, `global_to_enu`, coordinate validation, ROS geographic message conversion, and `body_to_enu`. The Python module `swarm_geometry.geography` exports `GeographicOrigin`, `enu_to_geographic`, and `geographic_to_enu`. `mission_planner.geography` retains compatibility imports.

The [coordinate guide](../../docs/architecture/COORDINATES.md) defines application contracts, geographic navigation commands, and the documentation and research basis. Numerical tests run in `vehicle_control/test/test_coordinates.cpp` and `mission_planner/test/test_geography.py`; command-frame tests run in `mission_execution/test/test_geometry.cpp`.
