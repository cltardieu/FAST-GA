"""
    Computation of the aircraft polars
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

import numpy as np
from openmdao.core.explicitcomponent import ExplicitComponent
from openmdao.api import Group
from ..lift_equilibrium import AircraftEquilibrium
from fastoad.model_base import Atmosphere


NB_POINTS_ALPHA = 16
NB_POINTS = 10


class ComputePolar(Group):
    def setup(self):
        self.add_subsystem(
            "non_equilibrated_polar",
            _compute_non_equilibrated_polar(low_speed_aero=False),
            promotes=["*"],
        )
        self.add_subsystem(
            "equilibrated_polar", _compute_equilibrated_polar(low_speed_aero=False), promotes=["*"]
        )
        self.add_subsystem(
            "non_equilibrated_polar",
            _compute_non_equilibrated_polar(low_speed_aero=True),
            promotes=["*"],
        )
        self.add_subsystem(
            "equilibrated_polar", _compute_equilibrated_polar(low_speed_aero=True), promotes=["*"]
        )


class _compute_non_equilibrated_polar(ExplicitComponent):
    def initialize(self):
        self.options.declare("low_speed_aero", default=False, types=bool)

    def setup(self):

        if self.options["low_speed_aero"]:
            self.add_input("data:aerodynamics:wing:low_speed:CL0_clean", val=np.nan)
            self.add_input("data:aerodynamics:wing:low_speed:induced_drag_coefficient", val=np.nan)
            self.add_input("data:aerodynamics:aircraft:low_speed:CD0", val=np.nan)
            self.add_input("data:aerodynamics:wing:low_speed:CL_alpha", val=np.nan, units="rad**-1")

            self.add_output(
                "data:aerodynamics:polar:non_equilibrated:low_speed:cd_vector",
                shape=NB_POINTS_ALPHA,
            )
            self.add_output(
                "data:aerodynamics:polar:non_equilibrated:low_speed:cl_vector",
                shape=NB_POINTS_ALPHA,
            )

        else:
            self.add_input("data:aerodynamics:wing:cruise:CL0_clean", val=np.nan)
            self.add_input("data:aerodynamics:wing:cruise:induced_drag_coefficient", val=np.nan)
            self.add_input("data:aerodynamics:aircraft:cruise:CD0", val=np.nan)
            self.add_input("data:aerodynamics:wing:cruise:CL_alpha", val=np.nan, units="rad**-1")

            self.add_output(
                "data:aerodynamics:polar:non_equilibrated:cruise:cd_vector", shape=NB_POINTS_ALPHA
            )
            self.add_output(
                "data:aerodynamics:polar:non_equilibrated:cruise:cl_vector", shape=NB_POINTS_ALPHA
            )

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):

        if self.options["low_speed_aero"]:
            coef_k = inputs["data:aerodynamics:wing:low_speed:induced_drag_coefficient"]
            cd0 = inputs["data:aerodynamics:aircraft:low_speed:CD0"]
            cl0 = inputs["data:aerodynamics:wing:low_speed:CL0_clean"]
            cl_alpha = inputs["data:aerodynamics:wing:low_speed:CL_alpha"]
        else:
            coef_k = inputs["data:aerodynamics:wing:cruise:induced_drag_coefficient"]
            cd0 = inputs["data:aerodynamics:aircraft:cruise:CD0"]
            cl0 = inputs["data:aerodynamics:wing:cruise:CL0_clean"]
            cl_alpha = inputs["data:aerodynamics:wing:cruise:CL_alpha"]

        alpha_array = np.linspace(0, 15, NB_POINTS_ALPHA) * np.pi / 180
        cl_array = cl0 + alpha_array * cl_alpha

        cd_array = cd0 + coef_k * cl_array ** 2

        if self.options["low_speed_aero"]:
            outputs["data:aerodynamics:polar:non_equilibrated:low_speed:cd_vector"] = cd_array
            outputs["data:aerodynamics:polar:non_equilibrated:low_speed:cl_vector"] = cl_array
        else:
            outputs["data:aerodynamics:polar:non_equilibrated:cruise:cd_vector"] = cd_array
            outputs["data:aerodynamics:polar:non_equilibrated:cruise:cl_vector"] = cl_array


class _compute_equilibrated_polar(AircraftEquilibrium):
    def initialize(self):
        self.options.declare("low_speed_aero", default=False, types=bool)
        self.options.declare("x_cg_ratio", default=0.0, types=float)

    def setup(self):
        self.add_input("data:geometry:wing:MAC:leading_edge:x:local", val=np.nan, units="m")
        self.add_input("data:geometry:wing:MAC:length", np.nan, units="m")
        self.add_input("data:geometry:wing:root:virtual_chord", np.nan, units="m")
        self.add_input("data:geometry:fuselage:maximum_width", np.nan, units="m")
        self.add_input("data:geometry:wing:MAC:at25percent:x", np.nan, units="m")
        self.add_input("data:geometry:fuselage:length", np.nan, units="m")
        self.add_input("data:geometry:wing:area", np.nan, units="m**2")
        self.add_input(
            "data:geometry:horizontal_tail:MAC:at25percent:x:from_wingMAC25", np.nan, units="m"
        )
        self.add_input("data:aerodynamics:wing:low_speed:CL_max_clean", val=np.nan)
        self.add_input("data:weight:aircraft:CG:aft:x", np.nan, units="m")
        self.add_input(
            "data:weight:aircraft:in_flight_variation:fixed_mass_comp:equivalent_moment",
            np.nan,
            units="kg*m",
        )
        self.add_input("data:weight:propulsion:tank:CG:x", np.nan, units="m")
        self.add_input(
            "data:weight:aircraft:in_flight_variation:fixed_mass_comp:mass", np.nan, units="kg"
        )
        self.add_input("data:weight:aircraft:MTOW", np.nan, units="kg")

        if self.options["low_speed_aero"]:
            self.add_input("data:aerodynamics:low_speed:mach", val=np.nan)
            self.add_input("data:aerodynamics:wing:low_speed:CL0_clean", val=np.nan)
            self.add_input("data:aerodynamics:wing:low_speed:induced_drag_coefficient", val=np.nan)
            self.add_input("data:aerodynamics:aircraft:low_speed:CD0", val=np.nan)
            self.add_input("data:aerodynamics:wing:low_speed:CL_alpha", val=np.nan, units="rad**-1")
            self.add_input("data:aerodynamics:wing:low_speed:CM0_clean", val=np.nan)
            self.add_input(
                "data:aerodynamics:horizontal_tail:low_speed:induced_drag_coefficient", val=np.nan
            )
            self.add_input(
                "data:aerodynamics:horizontal_tail:low_speed:CL_alpha", val=np.nan, units="rad**-1"
            )
            self.add_input("data:TLAR:v_approach", np.nan, units="m/s")

            self.add_output(
                "data:aerodynamics:polar:equilibrated:low_speed:cd_vector", shape=NB_POINTS
            )
            self.add_output(
                "data:aerodynamics:polar:equilibrated:low_speed:cl_vector", shape=NB_POINTS
            )

        else:
            self.add_input("data:aerodynamics:cruise:mach", val=np.nan)
            self.add_input("data:aerodynamics:wing:cruise:CL0_clean", val=np.nan)
            self.add_input("data:aerodynamics:wing:cruise:induced_drag_coefficient", val=np.nan)
            self.add_input("data:aerodynamics:aircraft:cruise:CD0", val=np.nan)
            self.add_input("data:aerodynamics:wing:cruise:CL_alpha", val=np.nan, units="rad**-1")
            self.add_input("data:aerodynamics:wing:cruise:CM0_clean", val=np.nan)
            self.add_input(
                "data:aerodynamics:horizontal_tail:cruise:induced_drag_coefficient", val=np.nan
            )
            self.add_input(
                "data:aerodynamics:horizontal_tail:cruise:CL_alpha", val=np.nan, units="rad**-1"
            )
            self.add_input("data:mission:sizing:main_route:cruise:altitude", np.nan, units="m")
            self.add_input("data:TLAR:v_cruise", np.nan, units="m/s")

            self.add_output(
                "data:aerodynamics:polar:equilibrated:cruise:cd_vector", shape=NB_POINTS
            )
            self.add_output(
                "data:aerodynamics:polar:equilibrated:cruise:cl_vector", shape=NB_POINTS
            )

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):

        mtow = inputs["data:weight:aircraft:MTOW"]

        if self.options["low_speed_aero"]:
            coef_k_wing = inputs["data:aerodynamics:wing:low_speed:induced_drag_coefficient"]
            coef_k_htp = inputs[
                "data:aerodynamics:horizontal_tail:low_speed:induced_drag_coefficient"
            ]
            cd0 = inputs["data:aerodynamics:aircraft:low_speed:CD0"]
            altitude = 0
            v_tas = inputs["data:TLAR:v_approach"]
        else:
            coef_k_wing = inputs["data:aerodynamics:wing:cruise:induced_drag_coefficient"]
            coef_k_htp = inputs["data:aerodynamics:horizontal_tail:cruise:induced_drag_coefficient"]
            cd0 = inputs["data:aerodynamics:aircraft:cruise:CD0"]
            altitude = inputs["data:mission:sizing:main_route:cruise:altitude"]
            v_tas = inputs["data:TLAR:v_cruise"]

        cl_wing_array = np.array([])
        cl_htp_only_array = np.array([])
        cl_elevator_array = np.array([])
        cl_array = np.array([])

        # TODO improve the way the mass array is created in order to match a specified range of cl
        # lower_boundary = 0.6
        # upper_boundary = 1.4
        #
        # while
        mass_array = np.linspace(0.5 * mtow, 1.5 * mtow, NB_POINTS)
        atm = Atmosphere(altitude, altitude_in_feet=False)

        if self.options["x_cg_ratio"] != 0:
            l0_wing = inputs["data:geometry:wing:MAC:length"]
            mac_position = inputs["data:geometry:wing:MAC:at25percent:x"]
            x_cg = mac_position - 0.25 * l0_wing + self.options["x_cg_ratio"] * l0_wing
        else:
            x_cg = -1.0

        for mass in mass_array:
            cl_wing, cl_htp_only, cl_elevator, _ = self.found_cl_repartition(
                inputs,
                1.0,
                mass,
                (0.5 * atm.density * v_tas ** 2),
                self.options["low_speed_aero"],
                x_cg=x_cg,
            )
            cl_wing_array = np.append(cl_wing_array, cl_wing)
            cl_htp_only_array = np.append(cl_htp_only_array, cl_htp_only)
            cl_elevator_array = np.append(cl_elevator_array, cl_elevator)
            cl_array = np.append(cl_array, cl_wing + cl_htp_only + cl_elevator)

        cd_array = (
            cd0
            + coef_k_wing * cl_wing_array ** 2
            + coef_k_htp * (cl_htp_only_array + cl_elevator_array) ** 2
        )

        if self.options["low_speed_aero"]:
            outputs["data:aerodynamics:polar:equilibrated:low_speed:cd_vector"] = cd_array
            outputs["data:aerodynamics:polar:equilibrated:low_speed:cl_vector"] = cl_array
        else:
            outputs["data:aerodynamics:polar:equilibrated:cruise:cd_vector"] = cd_array
            outputs["data:aerodynamics:polar:equilibrated:cruise:cl_vector"] = cl_array
