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

from .compute_cl_extreme import ComputeExtremeCL
from .compute_reynolds import ComputeUnitReynolds
from .compute_cnbeta_fuselage import ComputeCnBetaFuselage
from .compute_L_D_max import ComputeLDMax
from .high_lift_aero import ComputeDeltaHighLift
from .hinge_moments_elevator import Compute2DHingeMomentsTail, Compute3DHingeMomentsTail
from .mach_interpolation import ComputeMachInterpolation
from .airfoil_lift_curve_slope import ComputeAirfoilLiftCurveSlope
from .compute_cy_rudder import ComputeCyDeltaRudder
from .clalpha_vt import ComputeClAlphaVT
from .compute_vn import ComputeVNAndVH, ComputeVN
from .compute_cm_alpha_fus import ComputeFuselagePitchingMoment
from .compute_propeller_aero import _ComputePropellerPerformance, ComputePropellerPerformance
