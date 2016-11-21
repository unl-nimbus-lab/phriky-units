|                                                      |
|------------------------------------------------------|
| Phriky Units - Physical Unit Inconsistency Detection |

[![image]]

[![image][1]]

[![Updates]]

Physical unit static analysis tool for C++, especially for ROS, pronounced *‘freaky’* for the Greek work Phriki meaning ‘horror’.

## Demonstration

[![Phriky-Units](.youtube.png)](https://youtu.be/cc-Bubopml4 "Phriky-Units")

\* Free software: MIT license .. \*Lightweight static analysis Lightweight static analysis Documentation: <https://phriky-units.readthedocs.io>

Install (tested on Ubuntu 16.04 and OSX 10.11.6) ——-

`sudo pip install phriky_units`

**Requires Cppcheck &gt; 1.75:**

`sudo apt-get install cppcheck` (Ubuntu 16.04)

`brew install cppcheck` (OSX)

`git clone git://github.com/danmar/cppcheck.git` (Ubuntu 14.04)

Examples:
=========

You can run examples by checking out:

`git clone https://github.com/unl-nimbus-lab/phriky-units.git`

Then from that directory run:

`phriky_units ./examples/addition/src/action.cpp`

`phriky_units ./examples/assignment/src/trajectory_planner_ros.cpp`

`phriky_units ./examples/comparison/src/twist_marker.cpp`

Features
========

-   Detects physical unit inconsistencies, like adding quantities with different units, i.e. `meters` + `seconds`.
-   Lightweight static analysis
-   Path insensitive
-   No annotation burden
-   Low false positive rate (&lt; 15% for \`high-confidence’ inconsistencies)
-   Works with [ROS]

Credits
=======

[NIMBUS] Lab at the University of Nebraska, Lincoln

This work was supported in part by NSF awards \#1638099 and \#1526652, and USDA-NIFA \#2013-67021-20947.

Thank you [Cookiecutter] and the [audreyr/cookiecutter-pypackage] project template.

  [image]: https://img.shields.io/pypi/v/phriky_units.svg
  [![image]]: https://pypi.python.org/pypi/phriky_units
  [1]: https://img.shields.io/travis/jpwco/phriky_units.svg
  [![image][1]]: https://travis-ci.org/jpwco/phriky_units
  [Updates]: https://pyup.io/repos/github/jpwco/phriky_units/shield.svg
  [![Updates]]: https://pyup.io/repos/github/jpwco/phriky_units/
  [ROS]: http://www.ros.org
  [NIMBUS]: http://nimbus.unl.edu
  [Cookiecutter]: https://github.com/audreyr/cookiecutter
