===============================
Phriky Units - Physical Unit Inconsistency Detection 
===============================


.. image:: https://img.shields.io/pypi/v/phriky_units.svg
        :target: https://pypi.python.org/pypi/phriky_units

.. image:: https://img.shields.io/travis/jpwco/phriky_units.svg
        :target: https://travis-ci.org/jpwco/phriky_units

.. image:: https://readthedocs.org/projects/phriky-units/badge/?version=latest
        :target: https://phriky-units.readthedocs.io/en/latest/?badge=latest
        :alt: Documentation Status

.. image:: https://pyup.io/repos/github/jpwco/phriky_units/shield.svg
     :target: https://pyup.io/repos/github/jpwco/phriky_units/
     :alt: Updates


Physical unit static analysis tool for C++, especially for ROS, pronounced *'freaky'* for the Greek work Phriki meaning 'horror'.

* Free software: MIT license
.. *Lightweight static analysis Lightweight static analysis  Documentation: https://phriky-units.readthedocs.io


Install  (tested on Ubuntu 16.04 and OSX 10.11.6)
-------

``sudo pip install phriky_units``

**Requires Cppcheck > 1.75:**

``sudo apt-get install cppcheck`` (Ubuntu 16.04)

``brew install cppcheck`` (OSX)

``git clone git://github.com/danmar/cppcheck.git`` (Ubuntu 14.04)


Examples:
---------
You can run examples by checking out:

``git clone https://github.com/unl-nimbus-lab/phriky-units.git``

Then from that directory run:

``phriky_units ./examples/addition/src/action.cpp``

``phriky_units ./examples/assignment/src/trajectory_planner_ros.cpp``

``phriky_units ./examples/comparison/src/twist_marker.cpp``


Features
--------

* Detects physical unit inconsistencies, like adding quantities with different units, i.e. ``meters`` + ``seconds``.
* Lightweight static analysis 
* Path insensitive
* Zero annotation
* Low false positive rate (< 15% for `high-confidence' inconsistencies)
* Works with ROS_

Credits
---------
 
NIMBUS_ Lab at the University of Nebraska

This work was supported in part by NSF awards #1638099 and #1526652, and USDA-NIFA #2013-67021-20947.

Thank you Cookiecutter_ and the `audreyr/cookiecutter-pypackage`_ project template.

.. _NIMBUS: http://nimbus.unl.edu 
.. _ROS: http://www.ros.org
.. _Cookiecutter: https://github.com/audreyr/cookiecutter
.. _`audreyr/cookiecutter-pypackage`: https://github.com/audreyr/cookiecutter-pypackage

