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

from fastga.models.load_analysis.aerostructural_loads import AerostructuralLoad
from fastga.models.load_analysis.structural_loads import StructuralLoads
from fastga.models.load_analysis.aerodynamic_loads import AerodynamicLoads

from fastoad.module_management.service_registry import RegisterOpenMDAOSystem
from fastoad.module_management.constants import ModelDomain


@RegisterOpenMDAOSystem("fastga.loads.legacy", domain=ModelDomain.OTHER)
class Loads(om.Group):
    def setup(self):
        self.add_subsystem(
            "aerostructural_loads", AerostructuralLoad(), promotes=["*"],
        )
        self.add_subsystem("structural_loads", StructuralLoads(), promotes=["*"])
        self.add_subsystem("aerodynamic_loads", AerodynamicLoads(), promotes=["*"])
