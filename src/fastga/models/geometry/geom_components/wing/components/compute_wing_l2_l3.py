"""
    Estimation of wing chords (l2 and l3)
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


class ComputeWingL2AndL3(ExplicitComponent):
    # TODO: Document equations. Cite sources
    """ Wing chords (l2 and l3) estimation """

    def setup(self):

        self.add_input("data:geometry:wing:area", val=np.nan, units="m**2")
        self.add_input("data:geometry:wing:root:y", val=np.nan, units="m")
        self.add_input("data:geometry:wing:tip:y", val=np.nan, units="m")
        self.add_input("data:geometry:wing:taper_ratio", val=np.nan)

        self.add_output("data:geometry:wing:root:chord", units="m")
        self.add_output("data:geometry:wing:kink:chord", units="m")

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):

        wing_area = inputs["data:geometry:wing:area"]
        y2_wing = inputs["data:geometry:wing:root:y"]
        y4_wing = inputs["data:geometry:wing:tip:y"]
        taper_ratio = inputs["data:geometry:wing:taper_ratio"]

        l2_wing = wing_area / (2.0 * y2_wing + (y4_wing - y2_wing) * (1.0 + taper_ratio))

        l3_wing = l2_wing

        outputs["data:geometry:wing:root:chord"] = l2_wing
        outputs["data:geometry:wing:kink:chord"] = l3_wing
