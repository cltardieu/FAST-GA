""" Estimation of fuel cell stacks center of gravity """

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


class ComputeBatteryCG(ExplicitComponent):
    """
    Center of gravity estimation of the fuel cells considering a maximum of two stacks.
    Assuming that if there are 2 stacks, they are placed next to each other (on an axis perpendicular to the fuselage).
    Therefore CG.x doesn't depend on the number of stacks (still an input in case it's modified).
    """

    def setup(self):
        # self.add_input("data:geometry:hybrid_powertrain:fuel_cell:number_stacks", val=np.nan, units=None)
        self.add_input("data:geometry:fuselage:length", val=np.nan, units="m")
        # self.add_input("data:geometry:fuselage:front_length", val=np.nan, units="m")
        # self.add_input("data:geometry:cabin:length", val=np.nan, units="m")

        self.add_output("data:weight:hybrid_powertrain:battery:CG:x", units="m")

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        # nb_stacks = inputs['data:geometry:hybrid_powertrain:fuel_cell:number_stacks']
        fus_length = inputs["data:geometry:fuselage:length"]
        # fus_front_length = inputs["data:geometry:fuselage:front_length"]
        # cabin_length = inputs["data:geometry:cabin:length"]

        cg_b5 = 0.1 * fus_length

        outputs["data:weight:propulsion:battery:CG:x"] = cg_b5