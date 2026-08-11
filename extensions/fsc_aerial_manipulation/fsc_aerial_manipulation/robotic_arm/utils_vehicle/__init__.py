"""
utils_vehicle — Pegasus vehicle classes for the arm-equipped X650.

    x650_vehicle.py     VehicleMod(Vehicle) — thin subclass adding
                        usd_prim_path / configurable body_path so a custom USD
                        asset can be spawned instead of the stock layout
    x650_multirotor.py  MultirotorMod(Multirotor, VehicleMod) — adds
                        configurable rotor prim paths and joint names

Both exist because the stock pegasus.simulator Multirotor hardcodes the
/body + /rotor0..3 + joint0..3 naming, which the AM_realign arm asset does not
follow. The bare X650 (no arm) DOES follow it and therefore uses the stock
classes via rotorcraft/x650_bare_frame_utils.py — not these.

Spawning goes through rotorcraft/x650_rotorcraft_utils.py, which imports
MultirotorMod from here; demos rarely import these modules directly.
"""
