"""Simple vehicle parameter sets.

Both vehicles are modelled the same way: a point that moves at constant
airspeed V and can only be steered sideways with a limited lateral
acceleration a_max. That gives the curvature limit:

    r_min = V^2 / a_max

Fixed-wing : the limit comes from a minimum flying speed and a maximum bank angle.
Quadcopter : the limit comes from the sideways acceleration it can produce at
             the chosen cruise speed (the faster you fly, the wider the smallest turn).

The lateral acceleration follows the command through a first-order lag `tau`
(roll / attitude response). This is a deliberately simple model. It is meant
for comparing guidance laws, not for predicting a specific aircraft.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class VehicleParams:
    name: str
    airspeed: float   # m/s
    a_max: float      # m/s^2, maximum lateral acceleration
    tau: float        # s, response lag of the lateral acceleration

    @property
    def r_min(self) -> float:
        return self.airspeed ** 2 / self.a_max

    @classmethod
    def from_rmin(cls, name: str, airspeed: float, r_min: float, tau: float):
        return cls(name, airspeed, airspeed ** 2 / r_min, tau)


FIXED_WING = VehicleParams.from_rmin("fixed_wing", airspeed=18.0, r_min=50.0, tau=0.5)
QUADCOPTER = VehicleParams.from_rmin("quadcopter", airspeed=5.0, r_min=12.5, tau=0.3)
VEHICLES = {v.name: v for v in (FIXED_WING, QUADCOPTER)}
