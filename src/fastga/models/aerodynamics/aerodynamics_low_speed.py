"""
    FAST - Copyright (c) 2016 ONERA ISAE
"""

#  This file is part of FAST : A framework for rapid Overall Aircraft Design
#  Copyright (C) 2020  ONERA & ISAE-SUPAERO
#  FAST is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <https://www.gnu.org/licenses/>.

from openmdao.core.group import Group

from fastga.models.aerodynamics.components.cd0 import Cd0
from fastga.models.aerodynamics.components.compute_cl_extreme import ComputeExtremeCL
from fastga.models.aerodynamics.components.clalpha_vt import ComputeClAlphaVT
from fastga.models.aerodynamics.components.high_lift_aero import ComputeDeltaHighLift
from fastga.models.aerodynamics.components.airfoil_lift_curve_slope import (
    ComputeAirfoilLiftCurveSlope,
)
from fastga.models.aerodynamics.components.compute_cy_rudder import ComputeCyDeltaRudder
from fastga.models.aerodynamics.external.vlm import ComputeAEROvlm
from fastga.models.aerodynamics.external.openvsp import ComputeAEROopenvsp

# noinspection PyProtectedMember
from fastga.models.aerodynamics.external.openvsp.compute_aero_slipstream import (
    _ComputeSlipstreamOpenvsp,
)

from fastoad.module_management.service_registry import RegisterOpenMDAOSystem
from fastoad.module_management.constants import ModelDomain


@RegisterOpenMDAOSystem("fastga.aerodynamics.lowspeed.legacy", domain=ModelDomain.AERODYNAMICS)
class AerodynamicsLowSpeed(Group):
    """
    Models for low speed aerodynamics
    """

    def initialize(self):
        self.options.declare("propulsion_id", default="", types=str)
        self.options.declare("use_openvsp", default=False, types=bool)
        self.options.declare("compute_slipstream", default=False, types=bool)
        self.options.declare("result_folder_path", default="", types=str)
        self.options.declare("wing_airfoil", default="naca23012.af", types=str, allow_none=True)
        self.options.declare("htp_airfoil", default="naca0012.af", types=str, allow_none=True)
        self.options.declare("vtp_airfoil", default="naca0012.af", types=str, allow_none=True)

    # noinspection PyTypeChecker
    def setup(self):
        if not (self.options["use_openvsp"]):
            self.add_subsystem(
                "aero_vlm",
                ComputeAEROvlm(
                    low_speed_aero=True,
                    compute_mach_interpolation=False,
                    result_folder_path=self.options["result_folder_path"],
                    wing_airfoil_file=self.options["wing_airfoil"],
                    htp_airfoil_file=self.options["htp_airfoil"],
                ),
                promotes=["*"],
            )
        else:
            self.add_subsystem(
                "aero_openvsp",
                ComputeAEROopenvsp(
                    low_speed_aero=True,
                    compute_mach_interpolation=False,
                    result_folder_path=self.options["result_folder_path"],
                    wing_airfoil_file=self.options["wing_airfoil"],
                    htp_airfoil_file=self.options["htp_airfoil"],
                ),
                promotes=["*"],
            )
        self.add_subsystem(
            "Cd0_all",
            Cd0(
                low_speed_aero=True,
                wing_airfoil_file=self.options["wing_airfoil"],
                htp_airfoil_file=self.options["htp_airfoil"],
                propulsion_id=self.options["propulsion_id"],
            ),
            promotes=["*"],
        )
        self.add_subsystem(
            "airfoil_lift_slope",
            ComputeAirfoilLiftCurveSlope(
                wing_airfoil_file=self.options["wing_airfoil"],
                htp_airfoil_file=self.options["htp_airfoil"],
                vtp_airfoil_file=self.options["htp_airfoil"],
            ),
            promotes=["*"],
        )
        self.add_subsystem("high_lift", ComputeDeltaHighLift(), promotes=["*"])
        self.add_subsystem(
            "Cl_extreme",
            ComputeExtremeCL(
                wing_airfoil_file=self.options["wing_airfoil"],
                htp_airfoil_file=self.options["htp_airfoil"],
            ),
            promotes=["*"],
        )
        self.add_subsystem("clAlpha_vt", ComputeClAlphaVT(low_speed_aero=True), promotes=["*"])
        self.add_subsystem("Cy_Delta_rudder", ComputeCyDeltaRudder(), promotes=["*"])
        if self.options["compute_slipstream"]:
            self.add_subsystem(
                "aero_slipstream_openvsp",
                _ComputeSlipstreamOpenvsp(
                    propulsion_id=self.options["propulsion_id"],
                    result_folder_path=self.options["result_folder_path"],
                    wing_airfoil_file=self.options["wing_airfoil"],
                    low_speed_aero=True,
                ),
                promotes=["*"],
            )
