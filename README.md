![Tests](https://github.com/supaero-aircraft-design/FAST-GA/workflows/Tests/badge.svg)
[![Documentation Status](https://readthedocs.org/projects/fast-ga/badge/?version=latest)](https://fast-ga.readthedocs.io/en/latest/?badge=latest)

[![Codacy Badge](https://app.codacy.com/project/badge/Grade/ee153dd5e82d41e7b2f3a964ef5756f5)](https://www.codacy.com/gh/supaero-aircraft-design/FAST-GA/dashboard?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=supaero-aircraft-design/FAST-GA&amp;utm_campaign=Badge_Grade)
[![codecov](https://codecov.io/gh/supaero-aircraft-design/FAST-GA/branch/main/graph/badge.svg?token=VZEDUOFE8V)](https://codecov.io/gh/supaero-aircraft-design/FAST-GA)
[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

FAST-(OAD)-GA: Future Aircraft Sizing Tool - Overall Aircraft Design (General Aviation extension)
===============================================================================================

FAST-(OAD)-GA is derived from FAST-OAD framework performing rapid Overall Aircraft Design.

It proposes multi-disciplinary analysis and optimisation by relying on
the [OpenMDAO framework](https://openmdao.org/).

FAST-(OAD)-GA allows easy switching between models for a same discipline, and
also adding/removing disciplines to match the need of your study.

Currently, FAST-(OAD)-GA is bundled with models for general aviation and conventional
propulsion (ICE propeller based). Other models will come soon, and you may create
your own models and use them instead of bundled ones.

More details can be found in the [official
documentation](https://fast-ga.readthedocs.io/).

Install
-------

**Prerequisite**:FAST-(OAD)-GA needs at least **Python 3.7.0**.

It is recommended (but not required) to install FAST-(OAD)-GA in a virtual
environment ([conda](https://docs.conda.io/en/latest/),
[venv](https://docs.python.org/3.7/library/venv.html), ...).

The FAST-(OAD)-GA is not registered for a direct pip install.
Yet an installation using pip command is possible. To do so, use command:
**pip install git+https://github.com/supaero-aircraft-design/FAST-GA.git@vxxx**, where xxx is the version number (ex: 0.1.2-beta).
