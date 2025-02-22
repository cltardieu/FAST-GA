"""
    Estimation of horizontal tail distance
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
import openmdao.api as om


class ComputeHTDistance(om.ExplicitComponent):
    # TODO: Document equations. Cite sources
    """ Horizontal tail distance estimation """

    def setup(self):

        self.add_input("data:geometry:vertical_tail:span", val=np.nan, units="m")
        self.add_input("data:geometry:has_T_tail", val=np.nan)

        self.add_output("data:geometry:horizontal_tail:z:from_wingMAC25", units="m")

        self.declare_partials(
            "data:geometry:horizontal_tail:z:from_wingMAC25",
            ["data:geometry:vertical_tail:span"],
            method="fd",
        )

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        tail_type = inputs["data:geometry:has_T_tail"]
        span = inputs["data:geometry:vertical_tail:span"]

        if tail_type == 0.0:
            height_ht = 0
        else:
            height_ht = 0 + span

        outputs["data:geometry:horizontal_tail:z:from_wingMAC25"] = height_ht
