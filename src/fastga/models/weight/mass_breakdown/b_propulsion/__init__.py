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

from .b1_2_oil_weight import ComputeOilWeight
from .b1_ICengine_weight import ComputeEngineWeight
from .b2_fuel_lines_weight import ComputeFuelLinesWeight
from .b3_unusable_fuel_weight import ComputeUnusableFuelWeight
from .b4_Eengine_weight import ComputeEEngineWeight
from .b5_cables_weight import ComputeCablesWeight
from .b6_power_electronics_weight import ComputePowerElecWeight
from .b7_propeller_weight import ComputePropellerWeight
from .b8_battery_weight import ComputeBatteryWeight
from .b9_fuel_cells_weight import ComputeFuelCellWeight
from .b10_bop_weight import ComputeBoPWeight
from .b11_inverter_weight import ComputeInverterWeight
from .b12_h2_storage_weight import ComputeH2StorageWeight
