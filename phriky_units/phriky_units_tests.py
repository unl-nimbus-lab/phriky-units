#!/usr/bin/env python
"""  COPYRIGHT (c) 2016 UNIVERSITY OF NEBRASKA NIMBUS LAB - JOHN-PAUL ORE
        PHRIKY-UNITS, THE PHYSICAL UNITS INCONSISTENCY DETECTION TOOL
"""

from cps_units_checker import CPSUnitsChecker
from distutils import spawn
from subprocess import Popen
import click
import os
import pkg_resources
from pkg_resources import resource_string
import re
import sys



@click.command()
@click.argument('target_file') 
@click.option('--include_dir', default='', help='include directory for cppcheck')
def main(target_file, include_dir):
    """Console script for phriky_units"""
    print pkg_resources.resource_listdir('phriky_units.resources', '')
    with open('delete_me.txt', 'w') as fp:
        fp.write(resource_string('phriky_units.resources.cppcheck', 'std.cfg'))
    



if __name__ == "__main__":
    main()
