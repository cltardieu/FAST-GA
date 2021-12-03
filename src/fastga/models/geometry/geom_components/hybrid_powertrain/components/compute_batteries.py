""" Module that sizes the battery in a hybrid propulsion model (FC-B configuration). """

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
from .resources import fuel_cell, battery
from fastoad.model_base import atmosphere
import numpy as np
import math as math
import matplotlib.pyplot as plt


class ComputeBatteries(om.ExplicitComponent):
    """
    Based on the work of Karan Kini K on FAST-GA-Elec - compute_battery.
    Based on Zhao, Tianyuan, "Propulsive Battery Packs Sizing for Aviation Applications" (2018). Dissertations and
    Theses. 393. (https://commons.erau.edu/edt/393)
    """

    def setup(self):
        self.add_input("data:propulsion:hybrid_powertrain:battery:type", val='', units=None, desc=" Facultative specified type of battery")
        self.add_input("data:geometry:hybrid_powertrain:battery:nb_packs", val=np.nan, units=None)
        # self.add_input("data:propulsion:hybrid_powertrain:battery:required_power", val=np.nan, units="J")
        self.add_input("data:propulsion:hybrid_powertrain:battery:required_energy", val=np.nan, units="Wh")
        self.add_input("data:propulsion:hybrid_powertrain:battery:input_current", val=np.nan, units="A")
        self.add_input("data:geometry:hybrid_powertrain:battery:cell_diameter", val=np.nan, units="m")
        self.add_input("data:geometry:hybrid_powertrain:battery:cell_length", val=np.nan, units="m")
        self.add_input("data:geometry:hybrid_powertrain:battery:cell_capacity", val=np.nan, units="Ah")
        self.add_input("data:geometry:hybrid_powertrain:battery:cell_mass", val=np.nan, units="kg")
        # self.add_input("data:propulsion:hybrid_powertrain:battery:cell_voltage", val=np.nan, units="V")
        self.add_input("data:propulsion:hybrid_powertrain:battery:max_C_rate", val=np.nan, units="h**-1")
        self.add_input("data:propulsion:hybrid_powertrain:battery:int_resistance", val=np.nan, units="ohm")
        self.add_input("data:propulsion:hybrid_powertrain:battery:SOC", val=np.nan, units=None)
        # self.add_input("data:propulsion:hybrid_powertrain:battery:current_limit", val=np.nan, units="A")
        # self.add_input("data:propulsion:hybrid_powertrain:battery:cutoff_voltage", val=np.nan, units="V")
        self.add_input("data:propulsion:hybrid_powertrain:battery:sys_nom_voltage", val=np.nan, units="V")
        self.add_input("data:propulsion:hybrid_powertrain:motor:motor_eff", val=np.nan, units=None)
        self.add_input("data:propulsion:hybrid_powertrain:TO_power", val=np.nan, units="W")

        self.add_output("data:geometry:hybrid_powertrain:battery:N_series", units=None)
        self.add_output("data:geometry:hybrid_powertrain:battery:N_parallel", units=None)
        self.add_output("data:geometry:hybrid_powertrain:battery:pack_volume", units='m**3')
        self.add_output("data:geometry:hybrid_powertrain:battery:tot_volume", units='m**3')

        self.declare_partials('*', '*', method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):

        battery_type = inputs['data:propulsion:hybrid_powertrain:battery:type']
        nb_packs = inputs['data:geometry:hybrid_powertrain:battery:nb_packs']
        # required_power = inputs['data:propulsion:hybrid_powertrain:battery:required_power']
        required_energy = inputs['data:propulsion:hybrid_powertrain:battery:required_energy']
        input_current = inputs['data:propulsion:hybrid_powertrain:battery:input_current']
        cell_d = inputs['data:geometry:hybrid_powertrain:battery:cell_diameter']
        cell_l = inputs['data:geometry:hybrid_powertrain:battery:cell_length']
        cell_c = inputs['data:propulsion:hybrid_powertrain:battery:cell_capacity']
        cell_m = inputs['data:propulsion:hybrid_powertrain:battery:cell_mass']
        # cell_V = inputs['data:propulsion:hybrid_powertrain:battery:cell_voltage']
        max_C_rate = inputs['data:propulsion:hybrid_powertrain:battery:max_C_rate']
        int_res = inputs['data:propulsion:hybrid_powertrain:battery:int_resistance']
        SOC = inputs['data:propulsion:hybrid_powertrain:battery:SOC']
        # current_limit = inputs['data:propulsion:hybrid_powertrain:battery:current_limit']
        # cutoff_V = inputs['data:propulsion:hybrid_powertrain:battery:cutoff_voltage']
        nom_V = inputs['data:propulsion:hybrid_powertrain:battery:sys_nom_voltage']
        motor_eff = inputs['data:propulsion:hybrid_powertrain:motor:motor_eff']
        TO_power = inputs['data:propulsion:hybrid_powertrain:TO_power']

        batt = battery.Battery(battery_type=battery_type,
                               nb_packs=nb_packs,
                               # required_power=required_power,
                               required_energy=required_energy,
                               in_current=input_current,
                               cell_diameter=cell_d,
                               cell_length=cell_l,
                               cell_capacity=cell_c,
                               cell_mass=cell_m,
                               # cell_V=cell_V,
                               max_C_rate=max_C_rate,
                               int_resistance=int_res,
                               SOC=SOC,
                               # current_limit=current_limit,
                               # cutoff_voltage=cutoff_V,
                               sys_nom_voltage=nom_V,
                               motor_TO_power=TO_power,
                               motor_eff=motor_eff)

        N_series = batt.compute_nb_cells_ser()
        N_par = batt.compute_nb_cells_par()
        vol = batt.compute_pack_volume()
        tot_vol = batt.compute_tot_volume()

        outputs['data:geometry:hybrid_powertrain:battery:N_series'] = N_series
        outputs['data:geometry:hybrid_powertrain:battery:N_parallel'] = N_par
        outputs['data:geometry:hybrid_powertrain:battery:pack_volume'] = vol
        outputs['data:geometry:hybrid_powertrain:battery:tot_volume'] = tot_vol


class ComputeBatteriesV2(om.ExplicitComponent):
    """
    Another battery sizing discipline based on the model provided here :
        https://electricalnotes.wordpress.com/2015/10/02/calculate-size-of-inverter-battery-bank/
    """

    def setup(self):

        self.add_input("data:propulsion:hybrid_powertrain:battery:required_power", val=np.nan, units='VA')
        self.add_input("data:propulsion:hybrid_powertrain:battery:bank_voltage", val=np.nan, units='V')
        self.add_input("data:propulsion:hybrid_powertrain:battery:backup_time", val=np.nan, units='h')
        self.add_input("data:propulsion:hybrid_powertrain:battery:wire_loss_factor", val=np.nan, units=None)
        self.add_input("data:propulsion:hybrid_powertrain:battery:aging_factor", val=np.nan, units=None)
        self.add_input("data:propulsion:hybrid_powertrain:battery:efficiency", val=np.nan, units=None)
        self.add_input("data:propulsion:hybrid_powertrain:battery:SOC", val=np.nan, units=None)
        self.add_input("data:propulsion:hybrid_powertrain:battery:operating_temperature", val=np.nan, units='K')

        self.add_output("data:propulsion:hybrid_powertrain:battery:design_capacity", units='Ah')
        self.add_output("data:geometry:hybrid_powertrain:battery:pack_volume", units='m**3')
        self.add_output("data:geometry:hybrid_powertrain:battery:tot_volume", units='m**3')

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        required_power = inputs['data:propulsion:hybrid_powertrain:battery:required_power']
        bank_V = inputs['data:propulsion:hybrid_powertrain:battery:bank_voltage']
        backup_t = inputs['data:propulsion:hybrid_powertrain:battery:backup_time']
        WLF = inputs['data:propulsion:hybrid_powertrain:battery:wire_loss_factor']
        Ag = inputs['data:propulsion:hybrid_powertrain:battery:aging_factor']
        eff = inputs['data:propulsion:hybrid_powertrain:battery:efficiency']
        SOC = inputs['data:propulsion:hybrid_powertrain:battery:SOC']
        op_T = inputs['data:propulsion:hybrid_powertrain:battery:operating_temperature']

        """ Computing design capacity """
        total_load = required_power * backup_t / bank_V  # [Ah]
        design_capacity = total_load * (1 + WLF) * (1 + Ag) * self.compute_temperature_corr_f(op_T) / (eff * (1 - SOC))

        outputs['data:propulsion:hybrid_powertrain:battery:design_capacity'] = design_capacity
        ### Compute volume without the number of cells ? use the specific power as an input ?

    def compute_temperature_corr_f(self, op_T: float):
        """
        Computes the temperature correction factor considering operating temperature in [K].
        Data are provided here :
        https://electricalnotes.wordpress.com/2015/10/02/calculate-size-of-inverter-battery-bank/
        Maybe use a linear regression to make it simpler :
            slope = -0.00957142857142857
            intercept = 1.711428571428571
        return slope * op_T + intercept
        """
        if 288 < op_T <= 298:
            return 1.59
        elif 298 < op_T <= 308:
            return 1.40
        elif 308 < op_T <= 318:
            return 1.30
        elif 318 < op_T <= 328:
            return 1.19
        elif 328 < op_T <= 338:
            return 1.11
        elif 338 < op_T <= 348:
            return 1.04
        else:
            return 1.00


