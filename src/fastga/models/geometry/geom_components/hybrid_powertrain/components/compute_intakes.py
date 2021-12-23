""" Computes the intakes for the fuel cells stacks and the sized heat exchanger subsystem in a hybrid propulsion
model (FC-B configuration). """

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
import math

import openmdao.api as om
import numpy as np
from .resources.constants import NACA_INTAKE
from stdatm.atmosphere import Atmosphere


class ComputeIntakes(om.ExplicitComponent):

    def setup(self):
        """
        Based on the work done in FAST-GA-AMPERE on heat exchanger sizing.
        Method is based on reference data found here :
            https://www.researchgate.net/publication/303312026_Numerical_Study_of_the_Performance_Improvement_of_Submerged_Air_Intakes_Using_Vortex_Generators
        This discipline computes two types of air intakes :
            - The first one supplies air to the fuel cell stacks ('fc_intake')
            - The second one supplies air to the cooling system of the fuel cell system ('cooling_intake')
        """
        self.add_input("data:geometry:hybrid_powertrain:hex:area", val=np.nan, units="m**2")
        self.add_input("data:propulsion:hybrid_powertrain:fuel_cell:ox_mass_flow", val=np.nan, units='kg/s')
        self.add_input("data:propulsion:hybrid_powertrain:hex:air_speed", val=np.nan, units='m/s')
        self.add_input("data:geometry:hybrid_powertrain:cooling_intake:nb_intakes", val=np.nan, units=None)

        self.add_output("data:geometry:hybrid_powertrain:fc_intake:length", units="mm")
        self.add_output("data:geometry:hybrid_powertrain:fc_intake:width", units="mm")
        self.add_output("data:geometry:hybrid_powertrain:fc_intake:depth", units="mm")
        self.add_output("data:geometry:hybrid_powertrain:cooling_intake:length", units="mm")
        self.add_output("data:geometry:hybrid_powertrain:cooling_intake:width", units="mm")
        self.add_output("data:geometry:hybrid_powertrain:cooling_intake:depth", units="mm")

        self.declare_partials('*', '*', method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        """
        A NACA inlet is used as reference for the sizing of the intakes.
        See https://www.researchgate.net/publication/303312026_Numerical_Study_of_the_Performance_Improvement_of_Submerged_Air_Intakes_Using_Vortex_Generators
        """
        hex_area = inputs['data:geometry:hybrid_powertrain:hex:area']
        air_speed = inputs['data:propulsion:hybrid_powertrain:hex:air_speed']
        nb_cooling_intakes = inputs['data:geometry:hybrid_powertrain:cooling_intake:nb_intakes']
        ox_mass_flow = inputs['data:propulsion:hybrid_powertrain:fuel_cell:ox_mass_flow']

        ref_L = NACA_INTAKE['LENGTH']  # [mm]
        ref_w = NACA_INTAKE['WIDTH']  # [mm]
        ref_d = NACA_INTAKE['DEPTH']  # [mm]
        ref_mass_flow = NACA_INTAKE['MASS_FLOW']  # [kg/s]

        """ Fuel Cell stacks air supply """
        flow_ratio = ox_mass_flow / ref_mass_flow
        dim_par = math.sqrt(flow_ratio)

        L = ref_L * dim_par  # [mm]
        w = ref_w * dim_par  # [mm]
        d = ref_d * dim_par  # [mm]

        outputs['data:geometry:hybrid_powertrain:fc_intake:length'] = L
        outputs['data:geometry:hybrid_powertrain:fc_intake:width'] = w
        outputs['data:geometry:hybrid_powertrain:fc_intake:depth'] = d

        """ Cooling system air supply """
        # Defining constants
        gamma = 1.4
        r = 287
        cp = gamma * r / (gamma - 1)

        T_air = Atmosphere(altitude=0).temperature  # [K]
        p_air = Atmosphere(altitude=0).pressure  # [Pa]

        T_hex = T_air - air_speed ** 2 / (2 * cp)
        p_hex = p_air / (T_air / T_hex) ** (gamma / (gamma - 1))
        rho_hex = p_hex / (r * T_hex)
        hex_mass_flow = hex_area * air_speed * rho_hex / nb_cooling_intakes  # [kg/s] - Mass flow of a single intake

        hex_flow_ratio = hex_mass_flow / ref_mass_flow
        hex_dim_par = math.sqrt(hex_flow_ratio)

        L_hex = ref_L * hex_dim_par  # [mm]
        w_hex = ref_w * hex_dim_par  # [mm]
        d_hex = ref_d * hex_dim_par  # [mm]

        outputs['data:geometry:hybrid_powertrain:cooling_intake:length'] = L_hex
        outputs['data:geometry:hybrid_powertrain:cooling_intake:width'] = w_hex
        outputs['data:geometry:hybrid_powertrain:cooling_intake:depth'] = d_hex


