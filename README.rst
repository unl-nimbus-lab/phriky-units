===============================
Phriky Units  
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
* Documentation: https://phriky-units.readthedocs.io.


Install
-------

``sudo pip install phriky_units``

Requires Cppcheck:

``sudo apt-get install cppcheck`` 
or 
``brew install cppcheck``



Examples:
---------
You can run examples by checking out:

``git clone https://github.com/unl-nimbus-lab/phriky-units.git``

Then run:

``phriky_units ./examples/addition/src/action.cpp``

``phriky_units ./examples/assignment/src/trajectory_planner_ros.cpp``

``phriky_units ./examples/comparison/src/twist_marker.cpp``


Features
--------

* TODO

Credits
---------
 

This work was supported in part by NSF awards #1638099 and #1526652, and USDA-NIFA #2013-67021-20947.


Thank you Cookiecutter_ and the `audreyr/cookiecutter-pypackage`_ project template.
.. _Cookiecutter: https://github.com/audreyr/cookiecutter
.. _`audreyr/cookiecutter-pypackage`: https://github.com/audreyr/cookiecutter-pypackage

