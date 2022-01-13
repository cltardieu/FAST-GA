""" Estimation of hydrogen weight """
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


class ComputeHydrogenWeight(ExplicitComponent):
    """
    Computing hydrogen weight considering a required endurance for the aircraft.
    """
    def setup(self):

        self.add_input("data:mission:sizing:endurance", val=np.nan, units='min')
        self.add_input("data:propulsion:hybrid_powertrain:fuel_cell:design_power", val=np.nan, units='W')
        self.add_input("data:propulsion:hybrid_powertrain:fuel_cell:cell_voltage", val=np.nan, units='V')

        self.add_output("data:weight:hybrid_powertrain:hydrogen:mass", units="kg")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):

        endurance = inputs['data:mission:sizing:endurance']
        fc_power = inputs['data:propulsion:hybrid_powertrain:fuel_cell:design_power']
        V_cell = inputs['data:propulsion:hybrid_powertrain:fuel_cell:cell_voltage']

        F_H = fc_power / (V_cell * 2 * 96500 * 500)  # [kg/s] - Flow rate of hydrogen
        b4_2 = endurance * F_H  # [kg]

        outputs['data:weight:hybrid_powertrain:hydrogen:mass'] = b4_2