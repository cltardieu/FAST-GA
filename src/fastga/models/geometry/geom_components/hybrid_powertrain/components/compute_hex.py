""" Module that computes the heat exchanger subsystem in a hybrid propulsion model (FC-B configuration). """

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

import openmdao.api as om
import numpy as np
from fastoad.model_base import atmosphere


class ComputeHex(om.ExplicitComponent):
    """
    This discipline computes the Heat Exchanger assuming it to be a Compact Heat Exchanger (CHE).
    Based on :
        - work done in FAST-GA-AMPERE
        - https://apps.dtic.mil/sti/pdfs/ADA525161.pdf
    """

    def setup(self):

        self.add_input("data:propulsion:hybrid_powertrain:hex:air_speed", val=np.nan, units='m/s')
        self.add_input("data:propulsion:hybrid_powertrain:fuel_cell:operating_temperature", val=np.nan, units='K')
        self.add_input("data:geometry:hybrid_powertrain:hex:radiator_surface_density", val=np.nan, units='kg/m**2')
        self.add_input("data:propulsion:hybrid_powertrain:fuel_cell:cooling_power", val=np.nan, units='W')
        self.add_input("data:mission:sizing:main_route:cruise:altitude", val=np.nan, units='m')

        self.add_output("data:geometry:hybrid_powertrain:hex:area", units="m**2")
        self.add_output("data:weight:hybrid_powertrain:hex:radiator_mass", units='kg')

        self.declare_partials('*', '*', method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        air_speed = inputs['data:propulsion:hybrid_powertrain:hex:air_speed']
        op_T = inputs['data:propulsion:hybrid_powertrain:fuel_cell:operating_temperature']
        surface_density = inputs['data:propulsion:hybrid_powertrain:hex:surface_density']
        fc_cooling_power = inputs['data:propulsion:hybrid_powertrain:fuel_cell:cooling_power']
        ext_T = atmosphere(altitude=inputs['data:performances:he_mission:cruise_altitude']).temperature  # [K]

        # Determining temperature gap and dissipative power of the CHE
        delta_T = op_T - ext_T
        h = 1269.0 * air_speed + 99.9  # [W/(m**2K)] - Heat Transfer Coefficient used in FAST-GA-AMPERE
        # h = 5000  # Or value of 5000

        # Determining surface of the radiator
        needed_area = fc_cooling_power / (h * delta_T)  # [m**2]
        outputs['data:geometry:hybrid_powertrain:hex:area'] = needed_area

        # Determining mass of the radiator
        M_rad = needed_area * surface_density
        outputs['data:weight:hybrid_powertrain:hex:radiator_mass'] = M_rad

        # Size the intakes here as in FAST-GA-AMPERE ?
