from tests.testing_utilities import run_system, get_indep_var_comp, list_inputs
import openmdao.api as om
import numpy as np
import pytest
from .components import (
    ComputeBatteries,
    ComputeBoP,
    ComputeCompressor,
    ComputeElectricMotor,
    ComputeFuelCells,
    ComputeH2Storage,
    ComputeHex,
    ComputeInverter
)
XML_FILE = "hybrid_aircraft.xml"

def test_compute_fuel_cells():
    """ Tests computation of the fuel cell stacks """

    # Research independent input value in .xml file
    ivc = get_indep_var_comp(list_inputs(ComputeFuelCells()), __file__, XML_FILE)

    # Run problem and check obtained value(s) is/(are) correct
    problem = run_system(ComputeFuelCells(), ivc)
    span = problem.get_val("data:geometry:vertical_tail:span", units="m")
    assert span == pytest.approx(1.459, abs=1e-3)