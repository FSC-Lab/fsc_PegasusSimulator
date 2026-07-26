# Outdoor simulation launchers

These launchers run the standard PX4-owned single-drone Iris and calibrated
X650 scenarios intended for the outdoor GPS/magnetometer parameter profile.

```bash
./scripts/outdoor_sim/start_single_drone_sitl.sh fsc_lab_machine
./scripts/outdoor_sim/start_x650_single_drone.sh fsc_lab_machine
```

For external-vision flight indoors, use the isolated profiles provided by
`scripts/indoor_sim/start_single_drone_iris.sh` and
`scripts/indoor_sim/start_single_drone_x650.sh` instead.

Shared configuration and helper scripts remain in the parent `scripts/`
directory.
