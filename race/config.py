"""Race-track config schema (gate layout) for the OQCRL race course.

A plain stdlib dataclass (no numpy / Isaac) so it loads without booting the sim.
Backported from the isaacrace MWE.
"""

from dataclasses import dataclass


@dataclass
class RaceTrackConfig:
    """A gate racecourse in the OQCRL NED frame (z down; gates at z=-1.5 => 1.5m up)."""

    gate_pos: list[list[float]]  # [N][3] NED gate centres
    gate_yaw: list[float]  # [N] gate headings (units per gate_yaw_unit)
    start_pos: list[float]  # [3] NED start position
    gate_yaw_unit: str = "multiples_pi_2"  # "multiples_pi_2" or "radians"
    gate_size: float = 1.5  # gate window side length (m)

    @classmethod
    def from_dict(cls, d: dict) -> "RaceTrackConfig":
        """Build a RaceTrackConfig from a plain dict."""
        return cls(**d)
