"""Module that contains all methods addressing the sizing of the batteries."""

# -*- coding: utf-8 -*-
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
import numpy as np
from constants import CELL_WEIGHT_FRACTION, BATT_OVERHEAD

CellTypes = {
    'LG-HG2': {
        # Common type of Li-ion battery - references found here : https://www.researchgate.net/publication/319935703_A_Fuel_Cell_System_Sizing_Tool_Based_on_Current_Production_Aircraft
        'DIAMETER': 18,  # [mm]
        'LENGTH': 65,  # [mm]
        'SPECIFIC_ENERGY': 240,  # [Wh/kg]
        'ENERGY_DENSITY': 670,  # [Wh/L]
        'RATED_CAPACITY': 3000,  # [mAh]
        'CELL_MASS': 44.5,  # [g]
        'I_MAX': 20,  # [A]
        'V_CUT_OFF': 2.5,  # [V]
        'V_NOM': 3.6  # [V]
    },
    'LI-S': {
        # At the moment : not developed enough to serve general aviation propulsion purposes
        'SPECIFIC_ENERGY': 550,  # [Wh/kg] - 2023 assessment - 600 in 2030
        'ENERGY_DENSITY': 620,  # [Wh/L] - 2023 assessment - 700 in 2030
    }
}


class Battery(object):
    """
    Batteries are designed for emergency backup if the fuel cell system were to fail, therefore to provide the same
    amount of power as the fuel cell system for around 20~30 min to allow the plane to land safely from any altitude.
    Assuming cylindrical battery cells and hexagonal stacking.
    Li-ion battery cells are considered for now.
    If there are more than one battery pack, we assume that :
        - there is a maximum of 2 batteries
        - the second battery pack serves as an emergency backup in case the first one fails : therefore,
          _init_ parameters define the sizing of a single battery pack
    Based on :
        https://www.researchgate.net/publication/319935703_A_Fuel_Cell_System_Sizing_Tool_Based_on_Current_Production_Aircraft
        https://commons.erau.edu/cgi/viewcontent.cgi?article=1392&context=edt
    """

    def __init__(
            self,
            # specific_energy: float,
            # energy_density: float,
            # power_density: float,
            # specific_power: float,
            # required_power: float,
            required_energy: float,
            in_current: float,
            cell_diameter: float,
            cell_length: float,
            cell_capacity: float,
            cell_nom_volt: float,
            cell_mass: float,
            # cell_V: float,
            max_C_rate: float,
            int_resistance: float,
            SOC: float,
            # current_limit: float,
            # cutoff_voltage: float,
            sys_nom_voltage: float,
            motor_TO_power: float,
            motor_eff: float,
            battery_type: str = '',
            nb_packs: int = 1
    ):

        if battery_type in CellTypes:
            self.data = CellTypes[f'{battery_type}']
            self.cell_d = self.data['DIAMETER']
            self.cell_l = self.data['LENGTH']
            self.cell_c = self.data['RATED_CAPACITY']
            self.cell_m = self.data['CELL_MASS']
            self.cell_nom_V = self.data['V_NOM']
            # self.specific_energy = data['SPECIFIC_ENERGY']
            # self.energy_density = data['ENERGY_DENSITY']
            # self.i_cr = data['I_MAX']
            # self.co_V = data['V_CUT_OFF']
            # self.cell_V = data['V_NOM']
        else:
            self.cell_d = cell_diameter
            self.cell_l = cell_length
            self.cell_c = cell_capacity
            self.cell_m = cell_mass
            self.cell_nom_V = cell_nom_volt
            # self.specific_energy = specific_energy
            # self.energy_density = energy_density
            # self.i_cr = current_limit
            # self.co_V = cutoff_voltage
            # self.cell_V = cell_V

        # self.power_density = power_density
        # self.specific_power = specific_power
        self.nb_packs = nb_packs
        # self.required_power = required_power
        self.energy = required_energy
        self.i_in = in_current
        self.max_C_rate = max_C_rate
        self.int_resistance = int_resistance
        self.SOC = SOC
        self.nom_voltage = sys_nom_voltage
        self.motor_TO_power = motor_TO_power
        self.motor_eff = motor_eff

    def compute_power(self):
        return self.battery_voltage * self.i_in

    def compute_voltage(self):
        """ Computes battery voltage considering cell voltage """
        return self.compute_V_cell() * self.compute_nb_cells_series()

    def compute_V_cell(self):
        """
        Using a linear approximation between 500 mAh and 2750 mAh to compute cell voltage
        (see https://commons.erau.edu/cgi/viewcontent.cgi?article=1392&context=edt)
        """
        V0 = self.cell_nom_V  # [V] - Battery cell voltage when battery at full capacity with a 0 discharging current
        V_soc = 0.94  # [V]
        # R_i = 0.039  # [Ohm] - Internal resistance of the battery

        return V0 - V_soc * self.SOC - self.int_resistance * self.cell_c * self.max_C_rate

    def compute_batt_voltage(self, discharge_current: float, work_time: float):
        """
        Considering Shepherd's empirical model for battery modelling. Equations and reference data can be found here :
        https://repository.tudelft.nl/islandora/object/uuid%3A6e274095-9920-4d20-9e11-d5b76363e709
        https://www.sciencedirect.com/science/article/pii/S0360319914031218
        """
        # Defining constants - Considering nominal battery parameters
        V0 = self.cell_nom_V  # [V] - Nominal voltage
        K = 0.08726  # [1/Ah] - Polarization constant
        Q = self.compute_capacity()  # [Ah] - Maximum battery capacity
        R = self.int_res  # [Ohm] - Ohmic resistance
        A = 2.451  # [V] - Exponential voltage
        C = discharge_current / self.compute_capacity
        p0 = 3.057  # [mA/h]
        p1 = -0.6613  # [mA]
        p2 = 0.1273  # [mAh]
        p3 = -9.331E-5  # [mAh**2]
        B = p3 * C ** 3 + p2 * C ** 2 + p1 * C + p0  # [1/Ah] - Exponential capacity

        V = V0 - K * Q / (Q - self.i * work_time) + A * np.exp(-B * self.i * work_time) - R * self.i
        return V

    def compute_capacity(self):
        """ Computes battery system capacity considering cell capacity """

        return self.motor_TO_power / (self.motor_eff * self.cell_V * self.compute_nb_cells_series())

    def compute_nb_cells_ser(self):
        """ Number of cells in series is computed considering nominal voltage of the battery system """
        # Check conditions : Ns·VCellMin ≥ VMotorMin and Ns·VCellMax ≤ VMotorMax
        return math.ceil(self.nom_voltage / self.compute_V_cell())

    def compute_nb_cells_par(self):
        """ Number of cells in parallel can be sized in power or in endurance """
        nb_power = math.ceil(self.motor_TO_power / (self.compute_V_cell() * self.compute_nb_cells_ser() *
                                                    self.motor_eff * self.cell_c * self.max_C_rate))
        nb_endurance = math.ceil(self.energy / (self.cell_c * self.nom_voltage))
        return max(nb_power, nb_endurance)

    def compute_discharge_current(self):
        """ Computing battery system output current """
        return self.motor_TO_power / (self.motor_eff * self.compute_voltage())

    def compute_pack_volume(self):
        """
        Computes the volume of a single battery pack.
        Assuming :
            - hexagonal packing for our calculation (and using a formula found in sources specified above)
            - identical packs if there are more than 1
        """
        eta = 0.907  # Hexagonal packing density

        V_pack = math.pi * self.cell_d ** 2 * self.cell_l * \
                 self.compute_nb_cells_ser() * self.compute_nb_cells_par() / (4 * eta * BATT_OVERHEAD)
        return V_pack

    def compute_tot_volume(self):
        """
        Computes the total volume of the battery pack(s).
        """
        return self.nb_packs * self.compute_pack_volume()

    def compute_weight(self):
        # Based on https://commons.erau.edu/edt/393
        return self.compute_nb_cells_par() * self.compute_nb_cells_ser() * self.cell_m / CELL_WEIGHT_FRACTION

    def depth_of_discharge(self):
        return 1 - self.SOC

    def compute_batt_voltage(self, discharge_current: float, work_time: float):
        """
        Considering Shepherd's model for battery modelling. Equations and reference data can be found here :
        https://repository.tudelft.nl/islandora/object/uuid%3A6e274095-9920-4d20-9e11-d5b76363e709
        https://www.sciencedirect.com/science/article/pii/S0360319914031218
        """
        # Defining constants - Considering nominal battery parameters
        V0 = self.cell_nom_V  # [V] - Nominal voltage - Battery cell voltage when battery at full capacity with a 0 discharging current
        K = 0.08726  # [1/Ah] - Polarization constant
        Q = self.compute_capacity()  # [Ah] - Maximum battery capacity
        R = self.int_res  # [Ohm] - Ohmic resistance
        A = 2.451  # [V] - Exponential voltage

        C = discharge_current / self.compute_capacity
        p0 = 3.057  # [mA/h]
        p1 = -0.6613  # [mA]
        p2 = 0.1273  # [mAh]
        p3 = -9.331E-5  # [mAh**2]
        B = p3 * C ** 3 + p2 * C ** 2 + p1 * C + p0  # [1/Ah] - Exponential capacity

        V = V0 - K * Q / (Q - self.i * work_time) + A * np.exp(-B * self.i * work_time) - R * self.i
        return V


