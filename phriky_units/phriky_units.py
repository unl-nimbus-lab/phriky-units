#!/usr/bin/env python
"""  COPYRIGHT (c) 2016 UNIVERSITY OF NEBRASKA NIMBUS LAB - JOHN-PAUL ORE
        PHRIKY-UNITS, THE PHYSICAL UNITS INCONSISTENCY DETECTION TOOL
"""

from __future__ import print_function
from cps_units_checker import CPSUnitsChecker
from distutils import spawn
from subprocess import Popen
import click
import os
import pkg_resources
import re
import sys
from shutil import copyfile

def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)

@click.command()
@click.argument('target_cpp_file') 
@click.option('--include_dir', default='', help='include directory for cppcheck')
@click.option('--debug_print_ast/--no-debug_print_ast', default=False, help='Very verbose debug print of Abstract Syntax Trees and unit decorations')
@click.option('--show_high_confidence/--no-show_high_confidence', default=True, help='Should show hig-confidence inconsistences. Defaults to True')
@click.option('--show_low_confidence/--no-show_low_confidence', default=False, help='Should show low-confidence inconsistencies. Defaults to False.')
def main(target_cpp_file, include_dir, debug_print_ast, show_high_confidence, show_low_confidence):
    """Console script for phriky_units"""
    # 'Detect physical unit inconsistencies in C++ code, especially ROS code.')

    original_directory = os.getcwd()
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
    # TEST FOR CPPCHECK
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

    if not spawn.find_executable('cppcheck'):
        # CPPCHECK NOT GLOBALLY INSTALLED, CHECK BIN DIRECTORY
        if not os.path.exists('bin/cppcheck'):
            eprint( 'Could not find required program Cppcheck') #todo
            eprint( 'two options: ')
            eprint( '  1.  sudo apt-get install cppcheck')  #todo
            eprint( '  2.  brew install cppcheck')  #todo
        sys.exit(1)

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
    # RUN CPPCHECK
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
    # EXTRACT DIR

    if not os.path.exists(target_cpp_file):
        eprint( 'file does not exist: %s' % target_cpp_file)
        sys.exit(1)

    eprint( 'Attempting to run cppcheck...')
    target_cpp_file_dir = os.path.dirname(target_cpp_file)
    eprint( 'Changing directory to %s' % target_cpp_file_dir)
    os.chdir(target_cpp_file_dir)
    # CREATE LOCAL COPY OF CFG
    if pkg_resources.resource_exists('phriky_units', 'resources'):
        if pkg_resources.resource_exists('phriky_units', 'resources/cppcheck'):
            if pkg_resources.resource_exists('phriky_units', 'resources/cppcheck/std.cfg'):
                if not os.path.exists('cfg'):
                    os.makedirs('cfg')
                with open('std.cfg', 'w') as f:
                    f.write(pkg_resources.resource_string('phriky_units', 'resources/cppcheck/std.cfg'))
                os.rename('std.cfg', 'cfg/std.cfg')
    else:
        eprint("resource phriky_units not found: trying local option")
        if not os.path.exists('cfg'):
            os.makedirs('cfg')
        copyfile(os.path.join(original_directory, 'std.cfg'), os.path.join(os.path.join(target_cpp_file_dir, 'cfg'), 'std.cfg'))
    
    target_cpp_file_base_name = os.path.basename(target_cpp_file)
    dump_filename = os.path.basename(target_cpp_file) + '.dump'

    if not os.path.exists(dump_filename):
        args = ['cppcheck', '--dump', '-I ../include', target_cpp_file_base_name]
        cppcheck_process = Popen(' '.join(args),  shell=True)
        cppcheck_process.communicate()
        if cppcheck_process.returncode != 0:
            eprint( 'cppcheck appears to have failed..exiting with return code %d' % cppcheck_process.returncode)
            sys.exit(1)
        eprint( "Created cppcheck 'dump' file %s'" % dump_filename)

    # REMOVE LOCAL COPY OF CFG
    if os.path.exists('cfg/std.cfg'):
        try:
            os.remove('cfg/std.cfg')
            os.rmdir('cfg')
        except:
            # eprint('problem removing cfg folder')
            pass # todo - fail silently for now
     
    # RETURN TO HOME
    os.chdir(original_directory)

    cps_unit_checker = CPSUnitsChecker()
    dump_file = os.path.join(os.path.dirname(target_cpp_file), dump_filename)
    source_file = dump_file.replace('.dump','')
    cps_unit_checker.debug_print_AST = debug_print_ast
    cps_unit_checker.main_run_check(dump_file, source_file)
    try:
        os.remove(dump_file)
    except: 
        pass


    # for e in cps_unit_checker.errors:
    cps_unit_checker.error_checker.pretty_print(show_high_confidence, show_low_confidence)
        # eprint( "%s" % e.get_error_desc())
        # eprint( "%s" % e.linenr)
        # eprint( "%s" % e.units_at_first_assignment)
        # eprint( "%s" % e.all_units_assigned_to_var)
        # eprint( "%s" % e.units_when_multiple_happened)
    # eprint("Total errors: %i" % len(cps_unit_checker.errors))




if __name__ == "__main__":
    main()
