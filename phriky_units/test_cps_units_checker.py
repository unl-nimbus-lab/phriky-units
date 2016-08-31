#!/usr/local/bin/python
import sys
# sys.path.append('/Users/jore/courses/NIMBUS/RESEARCH/CPS_TYPES/cps_units/')
import unittest
from detect_physical_unit_inconsistencies import CPSUnitsChecker
from unit_error_types import UnitErrorTypes
from unit_error import UnitError
import os

global_debug = False
global_debug_verbose = False
global_debug_AST = False

class TestStringMethods(unittest.TestCase):

        
    def test_function_return_0(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        cps_unit_checker.debug_scope = False
        dump_file  = './dump_files_for_tests/test_it_function_return_0.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        units_for_f1 = []
        # TEST THAT UNITS ARE ASSIGNED TO FUNCTION
        for tw in cps_unit_checker.all_tree_walkers:
            so = tw.symbol_helper.function_dictionary['scopeObject']
            if so.function:
                if so.function.name == 'f1':
                    units_for_f1 = so.function.return_units
        self.assertEquals(units_for_f1, [{'meter': 1}], 'Incorrect units returned for function: f1 .  Expected [{\'meter\':1}], received %s' % units_for_f1)
        # TEST THAT UNITS ARE RECEIVED TO FUNCTION
        actual_units = None
        for s in cps_unit_checker.current_configuration.scopes:
            if s.className == 'main':
                if 'x' in s.var_ordered_dict:
                    actual_units = s.var_ordered_dict['x'][12]['units']
                
        my_oracle = [{'meter': 1}]
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))
        

            
    def test_function_return_1(self):
        ''' x SHOULD END UP M/S, but so far THERE'S NO MECHANISM FOR PASSING UNITS IN TO A FUNCTION
        '''
        cps_unit_checker = CPSUnitsChecker()
        dump_file  = './dump_files_for_tests/test_it_function_return_1.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        for tw in cps_unit_checker.all_tree_walkers:
            so = tw.symbol_helper.function_dictionary['scopeObject']
            if so.function:
                if so.function.name == 'f1':
                    units_for_f1 = so.function.return_units

    def test_comparisons_1(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST= False
        dump_file  = './dump_files_for_tests/test_it_comparisons_1.cpp.dump'
        source_file  = './dump_files_for_tests/test_it_comparisons_1.cpp'
        cps_unit_checker.main_run_check(dump_file, source_file)
        e = cps_unit_checker.errors[0]
        # ORACLES
        token_left_units_oracle = [{'meter': 1}]
        token_right_units_oracle = [{'second': -1, 'meter': 1}]
        # ASSERTIONS
        self.assertEqual(e.token.str, '>')
        self.assertEqual(e.token_left.units, token_left_units_oracle)
        self.assertEqual(e.token_right.units, token_right_units_oracle)

    def test_logical_1(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_logical_1.cpp.dump'
        source_file  = './dump_files_for_tests/test_it_logical_1.cpp'
        cps_unit_checker.main_run_check(dump_file, source_file)
        # TEST 1
        e = cps_unit_checker.errors[0]
        # ORACLES
        token_right_units_oracle = [{'meter': 1}]
        # ASSERTIONS
        self.assertEqual(e.linenr, 13)
        self.assertEqual(e.token.str, '&&')
        self.assertEqual(e.token_right.units, token_right_units_oracle)
        # TEST 2
        e = cps_unit_checker.errors[1]
        # ORACLES
        token_left_units_oracle = [{'meter': 1}]
        # ASSERTIONS
        self.assertEqual(e.linenr, 18)
        self.assertEqual(e.token.str, '||')
        self.assertEqual(e.token_left.units, token_left_units_oracle)

    def test_abs_0(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        #cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_known_function_abs.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)

    
    def test_abs_namespace_std_0(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        #cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_known_function_abs_namespace_std.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)

    def test_abs_1(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        #cps_unit_checker.debug_verbose = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_known_function_abs_1.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        var_name = 't'
        var_linenr = 9
        actual_units = None
        for s in cps_unit_checker.current_configuration.scopes:
            if s.className == 'main':
                if var_name in s.var_ordered_dict and var_linenr in s.var_ordered_dict[var_name]:
                    actual_units = s.var_ordered_dict[var_name][var_linenr]['units']
        my_oracle = [{'second': 1}]
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))

    def test_abs_2(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_known_function_abs_2.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        var_name = 's'
        var_linenr =11 
        my_oracle = [{'meter': 1}]
        actual_units = None
        for s in cps_unit_checker.current_configuration.scopes:
            if s.className == 'main':
                if var_name in s.var_ordered_dict and var_linenr in s.var_ordered_dict[var_name]:
                    actual_units = s.var_ordered_dict[var_name][var_linenr]['units']
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))

    def test_multiplication_assignment_in_multi_configurations_1(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        #cps_unit_checker.debug_verbose = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_multiplication_assignment_in_multi_configurations.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        var_name = 'a_geometry_msgs_Accel.linear.x'
        var_linenr = 19 
        my_oracle = [{'second': -4, 'meter': 2}] 
        actual_units = None
        for s in cps_unit_checker.current_configuration.scopes:
            if s.className == 'main':
                if var_name in s.var_ordered_dict and var_linenr in s.var_ordered_dict[var_name]:
                    actual_units = s.var_ordered_dict[var_name][var_linenr]['units']
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))

    def test_unit_propagation_by_multiplication_1(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        #cps_unit_checker.debug_verbose = False
        #cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_unit_propagation_by_multiplication_1.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        for s in cps_unit_checker.current_configuration.scopes:
            if s.className == 'main':
                if 'f' in s.var_ordered_dict:
                    actual_units = s.var_ordered_dict['f'][14]['units']
        my_oracle = [{'second': -4, 'meter': 2}]
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))


    def test_unit_propagation_by_division_1(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        #cps_unit_checker.debug_verbose = False
        #cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_unit_propagation_by_division_1.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        actual_units = None
        for s in cps_unit_checker.current_configuration.scopes:
            if s.className == 'main':
                if 'f' in s.var_ordered_dict:
                    actual_units = s.var_ordered_dict['f'][14]['units']
        my_oracle = None
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))


    def test_mulitple_units_assigned(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        #cps_unit_checker.debug_verbose = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_multiple_units_assigned_1.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        expected_errors = ["test_it_multiple_units_assigned_1.cpp : 11  MULTIPLE UNITS BY ASSIGNMENT: [{'second': -1, 'meter': 1}, {'second': -2, 'meter': 2}]"]
        # self.assertListEqual([e['error_msg'] for e in cps_unit_checker.errors], expected_errors)
        # TEST QUANTITY OF ERRORS
        self.assertEqual(1, len(cps_unit_checker.errors))
        # TEST TyPE OF ERROR
        self.assertEqual(UnitErrorTypes.VARIABLE_MULTIPLE_UNITS, cps_unit_checker.errors[0].ERROR_TYPE)
        # TEST VALUE OF ERROR
        var_name = 'a_geometry_msgs_Accel.linear.x'
        var_linenr =11
        my_oracle = [{'second': -2, 'meter': 1}, {'second': -4, 'meter': 2}]
        actual_units = None
        for s in cps_unit_checker.current_configuration.scopes:
            if s.className == 'main':
                if var_name in s.var_ordered_dict and var_linenr in s.var_ordered_dict[var_name]:
                    actual_units = s.var_ordered_dict[var_name][var_linenr]['units']
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))

    def test_known_functions_sqrt_1(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        #cps_unit_checker.debug_verbose = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_known_function_sqrt_1.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        for s in cps_unit_checker.current_configuration.scopes:
            if s.className == 'main':
                if 'x' in s.var_ordered_dict:
                    actual_units = s.var_ordered_dict['x'][12]['units']
        my_oracle = [{'meter': 1}]
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))

    def test_known_functions_sqrt_2(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        #cps_unit_checker.debug_verbose = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_known_function_sqrt_2.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        for s in cps_unit_checker.current_configuration.scopes:
            if s.className == 'main':
                if 'x' in s.var_ordered_dict:
                    actual_units = s.var_ordered_dict['x'][12]['units']
        my_oracle = [{'second': -1, 'meter': 1}]
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))


    def test_known_functions_sqrt_3(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        #cps_unit_checker.debug_verbose = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_known_function_sqrt_3.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        actual_units = None
        for s in cps_unit_checker.current_configuration.scopes:
            if s.className == 'main':
                if 'x' in s.var_ordered_dict:
                    actual_units = s.var_ordered_dict['x'][12]['units']
        my_oracle = None
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))

    def test_known_functions_sqrt_4(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        #cps_unit_checker.debug_verbose = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_known_function_sqrt_4.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        for s in cps_unit_checker.current_configuration.scopes:
            if s.className == 'main':
                if 'x' in s.var_ordered_dict:
                    actual_units = s.var_ordered_dict['x'][14]['units']
        my_oracle = [{'meter': 1}]
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))


    def test_known_functions_sqrt_5(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        #cps_unit_checker.debug_verbose = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_known_function_sqrt_5.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        for s in cps_unit_checker.current_configuration.scopes:
            if s.className == 'main':
                if 'x' in s.var_ordered_dict:
                    actual_units = s.var_ordered_dict['x'][14]['units']
        my_oracle = [{'meter': 1}]
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))


    def test_known_functions_sqrt_half_units(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        #cps_unit_checker.debug_verbose = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_known_function_sqrt_half_units.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        for s in cps_unit_checker.current_configuration.scopes:
            if s.className == 'main':
                if 'x' in s.var_ordered_dict:
                    actual_units = s.var_ordered_dict['x'][11]['units']
        my_oracle = [{'meter': 0.5}]
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))


    def test_known_functions_atan2_1(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_verbose = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_known_function_atan2_1.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        for s in cps_unit_checker.current_configuration.scopes:
            if s.className == 'main':
                if 'f' in s.var_ordered_dict:
                    actual_units = s.var_ordered_dict['f'][7]['units']
        my_oracle = [{'radian': 1}]
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))


    def test_known_functions_atan2_2(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        #cps_unit_checker.debug_verbose = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_known_function_atan2_2.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        for s in cps_unit_checker.current_configuration.scopes:
            if s.className == 'main':
                if 'f' in s.var_ordered_dict:
                    actual_units = s.var_ordered_dict['f'][8]['units']
        my_oracle = [{'radian': 1}]
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))

    def test_toSec(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_toSec_0.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        for s in cps_unit_checker.current_configuration.scopes:
            if s.className == 'main':
                if 'duration' in s.var_ordered_dict:
                    actual_units = s.var_ordered_dict['duration'][7]['units']
        my_oracle = [{'second': 1}]
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))
        for s in cps_unit_checker.current_configuration.scopes:
            if s.className == 'main':
                if 'second' in s.var_ordered_dict:
                    actual_units = s.var_ordered_dict['second'][9]['units']
        my_oracle = [{'second': 1}]
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))

    def test_float_1(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_float_1.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        actual_units = None
        for s in cps_unit_checker.current_configuration.scopes:
            if s.className == 'main':
                if 'f' in s.var_ordered_dict and 11 in s.var_ordered_dict['f']:
                    actual_units = s.var_ordered_dict['f'][11]['units']
        my_oracle = [{'second': -1, 'meter': 1}]
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))

    def test_float_2(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_float_2.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        actual_units = None
        for s in cps_unit_checker.current_configuration.scopes:
            if s.className == 'main':
                if 'f' in s.var_ordered_dict and 11 in s.var_ordered_dict['f']:
                    actual_units = s.var_ordered_dict['f'][11]['units']
        my_oracle = [{'second': -1, 'meter': 1}]
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))


    def test_float_3(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_float_3.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        for s in cps_unit_checker.current_configuration.scopes:
            if s.className == 'main':
                if 'f' in s.var_ordered_dict:
                    actual_units = s.var_ordered_dict['f'][12]['units']
        my_oracle = [{'second': -1, 'meter': 1}]
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))


    def test_float_4(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_float_4.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        for s in cps_unit_checker.current_configuration.scopes:
            if s.className == 'main':
                if 'f' in s.var_ordered_dict:
                    actual_units = s.var_ordered_dict['f'][13]['units']
        my_oracle = [{'second': -1, 'meter': 1}]
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))


    def test_float_5(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_float_5.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        for s in cps_unit_checker.current_configuration.scopes:
            if s.className == 'main':
                if 'f' in s.var_ordered_dict:
                    actual_units = s.var_ordered_dict['f'][11]['units']
        my_oracle = [{'second': -1, 'meter': 1}]
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))


    def test_pow_1(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_known_function_pow_1.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        var_name = 'f'
        var_linenr =10 
        my_oracle = [{'meter': 4}]
        actual_units = None
        for s in cps_unit_checker.current_configuration.scopes:
            if s.className == 'main':
                if var_name in s.var_ordered_dict and var_linenr in s.var_ordered_dict[var_name]:
                    actual_units = s.var_ordered_dict[var_name][var_linenr]['units']
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))


    def test_pow_2(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_known_function_pow_2.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        var_name = 'f'
        var_linenr = 11 
        my_oracle = [{'meter': 4}]
        actual_units = None
        for s in cps_unit_checker.current_configuration.scopes:
            if s.className == 'main':
                if var_name in s.var_ordered_dict and var_linenr in s.var_ordered_dict[var_name]:
                    actual_units = s.var_ordered_dict[var_name][var_linenr]['units']
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))

    def test_pow_3(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_known_function_pow_3.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        var_name = 'f'
        var_linenr = 10
        my_oracle = [{'meter': 4}]
        actual_units = None
        for s in cps_unit_checker.current_configuration.scopes:
            if s.className == 'main':
                if var_name in s.var_ordered_dict and var_linenr in s.var_ordered_dict[var_name]:
                    actual_units = s.var_ordered_dict[var_name][var_linenr]['units']
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:%s Expected: %s received %s' %  (var_name, my_oracle, actual_units))

    def test_floor_1(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_known_function_floor_1.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        var_name = 's'
        var_linenr = 8 
        my_oracle = [{'meter': 1, 'second':-1}]
        actual_units = None
        for s in cps_unit_checker.current_configuration.scopes:
            if s.className == 'main':
                if var_name in s.var_ordered_dict and var_linenr in s.var_ordered_dict[var_name]:
                    actual_units = s.var_ordered_dict[var_name][var_linenr]['units']
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))

    def test_ceil_1(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_known_function_ceil_1.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        var_name = 's'
        var_linenr = 8 
        my_oracle = [{'meter': 1, 'second':-1}]
        actual_units = None
        for s in cps_unit_checker.current_configuration.scopes:
            if s.className == 'main':
                if var_name in s.var_ordered_dict and var_linenr in s.var_ordered_dict[var_name]:
                    actual_units = s.var_ordered_dict[var_name][var_linenr]['units']
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))


    def test_acos_1(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_known_function_acos_1.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        var_name = 'f'
        var_linenr = 7 
        my_oracle = [{'radian': 1}]
        actual_units = None
        for s in cps_unit_checker.current_configuration.scopes:
            if s.className == 'main':
                if var_name in s.var_ordered_dict and var_linenr in s.var_ordered_dict[var_name]:
                    actual_units = s.var_ordered_dict[var_name][var_linenr]['units']
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))


    def test_asin_1(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_known_function_asin_1.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        var_name = 'f'
        var_linenr = 7 
        my_oracle = [{'radian': 1}]
        actual_units = None
        for s in cps_unit_checker.current_configuration.scopes:
            if s.className == 'main':
                if var_name in s.var_ordered_dict and var_linenr in s.var_ordered_dict[var_name]:
                    actual_units = s.var_ordered_dict[var_name][var_linenr]['units']
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))


    def test_atan_1(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_known_function_atan_1.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        var_name = 'f'
        var_linenr = 7 
        my_oracle = [{'radian': 1}]
        actual_units = None
        for s in cps_unit_checker.current_configuration.scopes:
            if s.className == 'main':
                if var_name in s.var_ordered_dict and var_linenr in s.var_ordered_dict[var_name]:
                    actual_units = s.var_ordered_dict[var_name][var_linenr]['units']
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))


    def test_ternary_1(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_ternary_1.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        var_name = 'f'
        var_linenr = 9 
        my_oracle = [{'second': -1, 'meter': 1}, {'second': -1}]
        actual_units = None
        for s in cps_unit_checker.current_configuration.scopes:
            if s.className == 'main':
                if var_name in s.var_ordered_dict and var_linenr in s.var_ordered_dict[var_name]:
                    actual_units = s.var_ordered_dict[var_name][var_linenr]['units']
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))

    def test_function_args_1(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_function_args_1.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        # actual_units = None
        f = cps_unit_checker.current_configuration.functions[0].arg_units
        self.assertEqual(f[0][0]['linenr'], 13)
        self.assertEqual(f[0][0]['units'], [{'meter': 1}])


    def test_function_args_2(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_function_args_2.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        for f in cps_unit_checker.current_configuration.functions[0].arg_units:
            self.assertEqual(f[0]['linenr'], 13)
            self.assertEqual(f[0]['units'], [{'meter': 1}])

    def test_function_args_3(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_function_args_3.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        my_oracle_1 = 4
        my_oracle_2 = [{'meter': 1}]
        actual_units = None
        all_units_list = cps_unit_checker.current_configuration.functions[0].arg_units
        self.assertEqual(len(all_units_list), my_oracle_1)
        for u in all_units_list:
            self.assertEqual(u[0]['units'], my_oracle_2)

    def test_function_args_4(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_function_args_4.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        my_oracle = [{'meter': 1}]
        actual_units = None
        for f in cps_unit_checker.current_configuration.functions:
            for arg_u in f.arg_units:
                for arg_use_on_line in arg_u:
                    self.assertEqual(arg_use_on_line['units'], my_oracle)

    def test_function_args_5(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_function_args_5.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        my_oracle_1 = [{'meter': 1}]
        my_oracle_2 = 15
        f = cps_unit_checker.current_configuration.functions[0]
        self.assertEqual(f.arg_units[0][0]['units'], my_oracle_1)
        self.assertEqual(f.arg_units[0][0]['linenr'], my_oracle_2)

    def test_division_1(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_division_1.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        var_name = 'x'
        var_linenr = 9 
        my_oracle = [{'meter': 1}]
        actual_units = None
        for s in cps_unit_checker.current_configuration.scopes:
            if s.className == 'main':
                if var_name in s.var_ordered_dict and var_linenr in s.var_ordered_dict[var_name]:
                    actual_units = s.var_ordered_dict[var_name][var_linenr]['units']
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))

    def test_division_2(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_division_2.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        var_name = 'x'
        var_linenr = 9 
        my_oracle = [{'meter': 1}]
        actual_units = None
        for s in cps_unit_checker.current_configuration.scopes:
            if s.className == 'main':
                if var_name in s.var_ordered_dict and var_linenr in s.var_ordered_dict[var_name]:
                    actual_units = s.var_ordered_dict[var_name][var_linenr]['units']
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))

    def test_division_3(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_division_3.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        var_name = 'x'
        var_linenr = 9 
        my_oracle = [{'second': 2, 'meter': 1}]
        actual_units = None
        for s in cps_unit_checker.current_configuration.scopes:
            if s.className == 'main':
                if var_name in s.var_ordered_dict and var_linenr in s.var_ordered_dict[var_name]:
                    actual_units = s.var_ordered_dict[var_name][var_linenr]['units']
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))


    def test_division_4(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_division_4.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        var_name = 'x'
        var_linenr =10
        my_oracle = [{'second': 2}]
        actual_units = None
        for s in cps_unit_checker.current_configuration.scopes:
            if s.className == 'main':
                if var_name in s.var_ordered_dict and var_linenr in s.var_ordered_dict[var_name]:
                    actual_units = s.var_ordered_dict[var_name][var_linenr]['units']
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))

    def test_logical_2(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_logical_2.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        self.assertEqual(0, len(cps_unit_checker.errors))

    def test_error_type_1(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_error_return_type_1.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.current_file_under_analysis = dump_file
        cps_unit_checker.main_run_check(dump_file, source_file)
        self.assertEqual(1, len(cps_unit_checker.errors))

    def test_laser_scan_range_size_1(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_laser_scan_range_count_1.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        var_name = 'x'
        var_linenr = 7
        my_oracle = None
        actual_units = None
        for s in cps_unit_checker.current_configuration.scopes:
            if s.className == 'main':
                if var_name in s.var_ordered_dict and var_linenr in s.var_ordered_dict[var_name]:
                    actual_units = s.var_ordered_dict[var_name][var_linenr]['units']
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))
        self.assertEqual(0, len(cps_unit_checker.errors))

    def test_laser_scan_range_size_2(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_laser_scan_range_count_2.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        var_name = 'x'
        var_linenr = 7
        my_oracle = [{'meter':1}]
        actual_units = None
        for s in cps_unit_checker.current_configuration.scopes:
            if s.className == 'main':
                if var_name in s.var_ordered_dict and var_linenr in s.var_ordered_dict[var_name]:
                    actual_units = s.var_ordered_dict[var_name][var_linenr]['units']
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))

    def test_ros_duration_isZero_1(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_ros_duration_isZero_1.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        var_name = 't'
        var_linenr = 6
        my_oracle = None
        actual_units = None
        for s in cps_unit_checker.current_configuration.scopes:
            if s.className == 'main':
                if var_name in s.var_ordered_dict and var_linenr in s.var_ordered_dict[var_name]:
                    actual_units = s.var_ordered_dict[var_name][var_linenr]['units']
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))

    def test_ros_duration_isZero_2(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_ros_duration_isZero_2.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        var_name = 't'
        var_linenr = 6
        my_oracle = [{'second':1}]
        actual_units = None
        for s in cps_unit_checker.current_configuration.scopes:
            if s.className == 'main':
                if var_name in s.var_ordered_dict and var_linenr in s.var_ordered_dict[var_name]:
                    actual_units = s.var_ordered_dict[var_name][var_linenr]['units']
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))

    def test_ros_header_include_1(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/src/test_it_header_include_1.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        self.assertEqual(2, len(cps_unit_checker.errors))

    def test_ros_header_include_2(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/src/test_it_header_include_2.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        self.assertEqual(3, len(cps_unit_checker.errors))
        # WEAKER - SOMETHING STOCASTIC IS HAPPENING
        e = cps_unit_checker.errors[0]
        self.assertEqual(7, e.linenr)
        self.assertEqual('./dump_files_for_tests/src/../include/test_it_header_include_2.h', e.get_file_URI_where_error_occured())
        e = cps_unit_checker.errors[1]
        self.assertEqual(5, e.linenr)
        self.assertEqual('./dump_files_for_tests/src/test_it_header_include_2.cpp', e.get_file_URI_where_error_occured())

    #   DON'T ASSIGN UNITS TO ARRAYS WHEN array.empty() IS CALLED 
    def test_laser_range_empty_1(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_range_empty_1.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        self.assertEqual(0, len(cps_unit_checker.errors))

    #   DON'T ASSIGN UNITS TO ARRAYS WHEN time.isZero() IS CALLED 
    def test_ros_isZero_3(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_ros_isZero_3.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        self.assertEqual(0, len(cps_unit_checker.errors))

    #   DON'T ASSIGN UNITS DURING x = y = z = 0
    def test_multiple_initialization_1(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_multiple_initialization.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        self.assertEqual(0, len(cps_unit_checker.errors))

    #   WEAKEN ASSIGNMENT WHEN MULTIPLIED BY A CONSTANT (INT)
    def test_it_multiplication_with_constant_1(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_multiplication_with_constant_1.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        # self.assertEqual(0, len(cps_unit_checker.errors))
        var_name = 'f'
        var_linenr = 9
        my_oracle = [{'second':-1}]
        actual_units = None
        is_unit_propagation_based_on_constants = False
        for s in cps_unit_checker.current_configuration.scopes:
            if s.className == 'main':
                if var_name in s.var_ordered_dict and var_linenr in s.var_ordered_dict[var_name]:
                    actual_units = s.var_ordered_dict[var_name][var_linenr]['units']
                    is_unit_propagation_based_on_constants = s.var_ordered_dict[var_name][var_linenr]['is_unit_propagation_based_on_constants']
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))
        self.assertTrue(is_unit_propagation_based_on_constants, 'Unit inference should be weakened by constant interaction, but is still strong.')

    #   WEAKEN ASSIGNMENT WHEN MULTIPLIED BY A CONSTANT  (FLOAT)
    def test_it_multiplication_with_constant_2(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_multiplication_with_constant_2.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        # self.assertEqual(0, len(cps_unit_checker.errors))
        var_name = 'f'
        var_linenr = 9
        my_oracle = [{'second':-1}]
        actual_units = None
        is_unit_propagation_based_on_constants = False
        for s in cps_unit_checker.current_configuration.scopes:
            if s.className == 'main':
                if var_name in s.var_ordered_dict and var_linenr in s.var_ordered_dict[var_name]:
                    actual_units = s.var_ordered_dict[var_name][var_linenr]['units']
                    is_unit_propagation_based_on_constants = s.var_ordered_dict[var_name][var_linenr]['is_unit_propagation_based_on_constants']
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))
        self.assertTrue(is_unit_propagation_based_on_constants, 'Unit inference should be weakened by constant interaction, but is still strong.')

    #   WEAKEN ASSIGNMENT WHEN MULTIPLIED BY A CONSTANT  (FLOAT)
    def test_it_operator_with_unknown_variable_1(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_operator_with_unknown_variable_1.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        # self.assertEqual(0, len(cps_unit_checker.errors))
        var_name = 'f'
        var_linenr = 10 
        my_oracle = [{'second':-1}]
        actual_units = None
        is_unit_propagation_based_on_unknown_variable = False
        for s in cps_unit_checker.current_configuration.scopes:
            if s.className == 'main':
                if var_name in s.var_ordered_dict and var_linenr in s.var_ordered_dict[var_name]:
                    actual_units = s.var_ordered_dict[var_name][var_linenr]['units']
                    is_unit_propagation_based_on_unknown_variable = s.var_ordered_dict[var_name][var_linenr]['is_unit_propagation_based_on_unknown_variable']
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))
        self.assertTrue(is_unit_propagation_based_on_unknown_variable, 'Unit inference should be weakened by unknown variable interaction, but is still strong.')

    # WEAKEN ERROR WHEN MULTIPLIED BY A CONSTANT 
    def test_it_operator_with_unknown_variable_2(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_operator_with_unknown_variable_2.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        self.assertEqual(2, len(cps_unit_checker.errors))
        for e in cps_unit_checker.errors:
            self.assertTrue(e.is_warning, 'Should be a warning but is not marked as such')

    # WEAKEN ERROR WHEN MULTIPLIED BY A CONSTANT 
    def test_it_operator_with_unknown_variable_3(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_operator_with_unknown_variable_3.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        self.assertEqual(2, len(cps_unit_checker.errors))

    # PROPAGATION ACROSS MIN MAX
    def test_it_min_max_1(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_min_max_1.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        var_name = 'f'
        var_linenr = 7
        my_oracle = [{'second': -1}]
        actual_units = None
        for s in cps_unit_checker.current_configuration.scopes:
            if s.className == 'main':
                if var_name in s.var_ordered_dict and var_linenr in s.var_ordered_dict[var_name]:
                    actual_units = s.var_ordered_dict[var_name][var_linenr]['units']
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))
        self.assertEqual(0, len(cps_unit_checker.errors))

    # PROPAGATION ACROSS MIN MAX
    def test_it_min_max_2(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_min_max_2.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        var_name = 'f'
        var_linenr = 8
        my_oracle = [{'second': -1}]
        actual_units = None
        for s in cps_unit_checker.current_configuration.scopes:
            if s.className == 'main':
                if var_name in s.var_ordered_dict and var_linenr in s.var_ordered_dict[var_name]:
                    actual_units = s.var_ordered_dict[var_name][var_linenr]['units']
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))
        self.assertEqual(0, len(cps_unit_checker.errors))


    # PROPAGATION ACROSS MIN MAX
    def test_it_min_max_3(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_min_max_3.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        var_name = 'f'
        var_linenr = 8
        my_oracle = [{'second': -1}, {'second': -1, 'meter': 1}]
        actual_units = None
        for s in cps_unit_checker.current_configuration.scopes:
            if s.className == 'main':
                if var_name in s.var_ordered_dict and var_linenr in s.var_ordered_dict[var_name]:
                    actual_units = s.var_ordered_dict[var_name][var_linenr]['units']
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))
        self.assertEqual(1, len(cps_unit_checker.errors))
        self.assertTrue(cps_unit_checker.errors[0].was_assigned_mutiple_units)
        self.assertFalse(cps_unit_checker.errors[0].is_unit_propagation_based_on_unknown_variable)
        self.assertFalse(cps_unit_checker.errors[0].is_warning)



    # PROPAGATION ACROSS MIN MAX
    def test_it_min_max_4(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_min_max_4.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        var_name = 'f'
        var_linenr = 9
        my_oracle = [{'second': -1 }, {'second': -1, 'meter': 1}]
        actual_units = None
        is_unit_propagation_based_on_unknown_variable = False
        for s in cps_unit_checker.current_configuration.scopes:
            if s.className == 'main':
                if var_name in s.var_ordered_dict and var_linenr in s.var_ordered_dict[var_name]:
                    actual_units = s.var_ordered_dict[var_name][var_linenr]['units']
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))
        self.assertEqual(1, len(cps_unit_checker.errors))
        self.assertTrue(cps_unit_checker.errors[0].was_assigned_mutiple_units)
        self.assertFalse(cps_unit_checker.errors[0].is_unit_propagation_based_on_unknown_variable)
        self.assertFalse(cps_unit_checker.errors[0].is_warning)



    # PROTECTION AGAINST MULTILINE
    def test_it_multiline_1(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_multiline_1.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        var_name = 'f'
        var_linenr = 25
        my_oracle = [{'second': -1.0, 'meter': 1.0}, {'second': -2.0, 'meter': 2.0}, {'second': -3.0, 'meter': 3.0}, {'second': -2.0, 'meter': 1.0}, {'second': -3.0, 'meter': 2.0}, {'second': -4.0, 'meter': 3.0}]
        actual_units = None
        # is_unit_propagation_based_on_unknown_variable = False
        for s in cps_unit_checker.current_configuration.scopes:
            if s.className == 'main':
                if var_name in s.var_ordered_dict and var_linenr in s.var_ordered_dict[var_name]:
                    actual_units = s.var_ordered_dict[var_name][var_linenr]['units']
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))

    # KNOW FUNCTION quatToRPY
    def test_it_quatToRPY_1(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_quatToRPY_1.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        var_name = 'tw.linear.x'
        var_linenr = 17
        my_oracle = [{'second': -1.0, 'meter': 1.0}, {'radian': 1.0}]
        actual_units = None
        # is_unit_propagation_based_on_unknown_variable = False
        for s in cps_unit_checker.current_configuration.scopes:
            # for v in s.var_ordered_dict:
                # print v
            if s.className == 'main':
                if var_name in s.var_ordered_dict and var_linenr in s.var_ordered_dict[var_name]:
                    actual_units = s.var_ordered_dict[var_name][var_linenr]['units']
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))
        self.assertEqual(1, len(cps_unit_checker.errors))
        self.assertTrue(cps_unit_checker.errors[0].was_assigned_mutiple_units)


    # WEAK INFERENCE WARNING
    def test_it_weak_inference_multiplication_1 (self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_weak_inference_multiplication_1.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        # for e in cps_unit_checker.errors:
            # print '\nweak inference: %s warning:%s ' % (e.var_name, str(e.is_warning))
        var_name = 'tw.linear.x'
        var_linenr = 19
        my_oracle = [{'second': -1.0, 'meter': 1.0}]
        actual_units = None
        # is_unit_propagation_based_on_unknown_variable = False
        for s in cps_unit_checker.current_configuration.scopes:
            # for v in s.var_ordered_dict:
                # print v
            if s.className == 'main':
                if var_name in s.var_ordered_dict and var_linenr in s.var_ordered_dict[var_name]:
                    actual_units = s.var_ordered_dict[var_name][var_linenr]['units']
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))
        self.assertEqual(0, len(cps_unit_checker.errors))

    # WEAK INFERENCE WARNING
    def test_it_weak_inference_multiplication_2 (self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_weak_inference_multiplication_2.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        var_name = 'tw.linear.x'
        var_linenr = 22
        my_oracle = [{'second': -1.0, 'meter': 1.0}]
        actual_units = None
        # is_unit_propagation_based_on_unknown_variable = False
        for s in cps_unit_checker.current_configuration.scopes:
            # for v in s.var_ordered_dict:
                # print v
            if s.className == 'main':
                if var_name in s.var_ordered_dict and var_linenr in s.var_ordered_dict[var_name]:
                    actual_units = s.var_ordered_dict[var_name][var_linenr]['units']
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))
        self.assertEqual(0, len(cps_unit_checker.errors))

    # STRONG INFERENCE BECAUSE ADDITION
    def test_it_weak_inference_addition_1 (self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_weak_inference_addition_1.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        # for e in cps_unit_checker.errors:
            # print '\nweak inference addition : %s warning:%s ' % (e.var_name, str(e.is_warning))
        var_name = 'tw.linear.x'
        var_linenr = 22
        my_oracle = [{'second': -1.0, 'meter': 1.0}, {'radian':1}]
        actual_units = None
        # is_unit_propagation_based_on_unknown_variable = False
        for s in cps_unit_checker.current_configuration.scopes:
            # for v in s.var_ordered_dict:
                # print v
            if s.className == 'main':
                if var_name in s.var_ordered_dict and var_linenr in s.var_ordered_dict[var_name]:
                    actual_units = s.var_ordered_dict[var_name][var_linenr]['units']
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))
        self.assertEqual(2, len(cps_unit_checker.errors))
        self.assertTrue(cps_unit_checker.errors[0].was_assigned_mutiple_units)
        self.assertFalse(cps_unit_checker.errors[0].is_unit_propagation_based_on_unknown_variable)
        self.assertFalse(cps_unit_checker.errors[0].is_warning)
        var_name = 'tw.linear.y'
        var_linenr = 23
        my_oracle = [{'second': -1.0, 'meter': 1.0}, {'radian':1}]
        actual_units = None
        # is_unit_propagation_based_on_unknown_variable = False
        for s in cps_unit_checker.current_configuration.scopes:
            # for v in s.var_ordered_dict:
                # print v
            if s.className == 'main':
                if var_name in s.var_ordered_dict and var_linenr in s.var_ordered_dict[var_name]:
                    actual_units = s.var_ordered_dict[var_name][var_linenr]['units']
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))
        self.assertTrue(cps_unit_checker.errors[1].was_assigned_mutiple_units)
        self.assertFalse(cps_unit_checker.errors[1].is_unit_propagation_based_on_unknown_variable)
        self.assertFalse(cps_unit_checker.errors[1].is_warning)

    # STRONG INFERENCE BECAUSE ADDITION - SWAPPED OPERAND ORDER
    def test_it_weak_inference_addition_2 (self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_weak_inference_addition_2.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        # for e in cps_unit_checker.errors:
            # print '\nweak inference addition : %s warning:%s ' % (e.var_name, str(e.is_warning))
        var_name = 'tw.linear.x'
        var_linenr = 22
        my_oracle = [{'second': -1.0, 'meter': 1.0}, {'radian':1}]
        actual_units = None
        # is_unit_propagation_based_on_unknown_variable = False
        for s in cps_unit_checker.current_configuration.scopes:
            # for v in s.var_ordered_dict:
                # print v
            if s.className == 'main':
                if var_name in s.var_ordered_dict and var_linenr in s.var_ordered_dict[var_name]:
                    actual_units = s.var_ordered_dict[var_name][var_linenr]['units']
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))
        self.assertEqual(2, len(cps_unit_checker.errors))
        self.assertTrue(cps_unit_checker.errors[0].was_assigned_mutiple_units)
        self.assertFalse(cps_unit_checker.errors[0].is_unit_propagation_based_on_unknown_variable)
        self.assertFalse(cps_unit_checker.errors[0].is_warning)
        var_name = 'tw.linear.y'
        var_linenr = 23
        my_oracle = [{'second': -1.0, 'meter': 1.0}, {'radian':1}]
        actual_units = None
        # is_unit_propagation_based_on_unknown_variable = False
        for s in cps_unit_checker.current_configuration.scopes:
            # for v in s.var_ordered_dict:
                # print v
            if s.className == 'main':
                if var_name in s.var_ordered_dict and var_linenr in s.var_ordered_dict[var_name]:
                    actual_units = s.var_ordered_dict[var_name][var_linenr]['units']
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))
        self.assertTrue(cps_unit_checker.errors[1].was_assigned_mutiple_units)
        self.assertFalse(cps_unit_checker.errors[1].is_unit_propagation_based_on_unknown_variable)
        self.assertFalse(cps_unit_checker.errors[1].is_warning)

    # STRONG INFERENCE BECAUSE ADDITION - SWAPPED OPERAND ORDER
    def test_it_weak_inference_addition_3 (self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_weak_inference_addition_3.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        # for e in cps_unit_checker.errors:
            # print '\nweak inference addition : %s warning:%s ' % (e.var_name, str(e.is_warning))
        var_name = 'tw.linear.x'
        var_linenr = 22
        my_oracle = [{'second': -1.0, 'meter': 1.0}, {'radian':1.0}, {'second':1.}]
        actual_units = None
        # is_unit_propagation_based_on_unknown_variable = False
        for s in cps_unit_checker.current_configuration.scopes:
            # for v in s.var_ordered_dict:
                # print v
            if s.className == 'main':
                if var_name in s.var_ordered_dict and var_linenr in s.var_ordered_dict[var_name]:
                    actual_units = s.var_ordered_dict[var_name][var_linenr]['units']
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))
        self.assertEqual(2, len(cps_unit_checker.errors))
        self.assertTrue(cps_unit_checker.errors[0].was_assigned_mutiple_units)
        self.assertTrue(cps_unit_checker.errors[0].is_unit_propagation_based_on_unknown_variable)
        self.assertTrue(cps_unit_checker.errors[0].is_warning)


    # ADDITION STAND ALONE ERROR FOR ADDITION OF INCOMPATIBLE UNITS - STRONG
    def test_it_addition_without_assignment_1(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_addition_without_assignment_1.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        # for e in cps_unit_checker.errors:
            # print '\nweak inference addition : %s warning:%s ' % (e.var_name, str(e.is_warning))
        self.assertEqual(1, len(cps_unit_checker.errors))
        self.assertEqual(UnitErrorTypes.ADDITION_OF_INCOMPATIBLE_UNITS, cps_unit_checker.errors[0].ERROR_TYPE)
        self.assertFalse(cps_unit_checker.errors[0].is_warning)

    # ADDITION STAND ALONE ERROR FOR ADDITION OF INCOMPATIBLE UNITS - WEAK UNKNOWN VARIABLE
    def test_it_addition_without_assignment_2(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_addition_without_assignment_2.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        # for e in cps_unit_checker.errors:
            # print '\nweak inference addition : %s warning:%s ' % (e.var_name, str(e.is_warning))
        self.assertEqual(1, len(cps_unit_checker.errors))
        self.assertEqual(UnitErrorTypes.ADDITION_OF_INCOMPATIBLE_UNITS, cps_unit_checker.errors[0].ERROR_TYPE)
        self.assertTrue(cps_unit_checker.errors[0].is_warning)

    # ADDITION STAND ALONE ERROR FOR ADDITION OF INCOMPATIBLE UNITS - WEAK CONSTANT
    def test_it_addition_without_assignment_3(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_addition_without_assignment_3.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        self.assertEqual(1, len(cps_unit_checker.errors))
        self.assertEqual(UnitErrorTypes.ADDITION_OF_INCOMPATIBLE_UNITS, cps_unit_checker.errors[0].ERROR_TYPE)
        self.assertTrue(cps_unit_checker.errors[0].is_warning)

    # ADDITION STAND ALONE ERROR FOR SUBTRACTION OF INCOMPATIBLE UNITS - STRONG CONSTANT
    def test_it_addition_without_assignment_4(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_addition_without_assignment_4.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        self.assertEqual(1, len(cps_unit_checker.errors))
        self.assertEqual(UnitErrorTypes.ADDITION_OF_INCOMPATIBLE_UNITS, cps_unit_checker.errors[0].ERROR_TYPE)
        self.assertFalse(cps_unit_checker.errors[0].is_warning)


    # ADDITION OF RADIANS
    def test_it_radian_addition_1(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_radian_addition_1.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        self.assertEqual(2, len(cps_unit_checker.errors))
        self.assertEqual(UnitErrorTypes.VARIABLE_MULTIPLE_UNITS, cps_unit_checker.errors[0].ERROR_TYPE)
        self.assertFalse(cps_unit_checker.errors[0].is_warning)
        self.assertEqual(UnitErrorTypes.ADDITION_OF_INCOMPATIBLE_UNITS, cps_unit_checker.errors[1].ERROR_TYPE)
        self.assertFalse(cps_unit_checker.errors[1].is_warning)


    # ADDITION OF RADIANS
    def test_it_radian_addition_2(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_radian_addition_2.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        self.assertEqual(1, len(cps_unit_checker.errors))
        self.assertEqual(UnitErrorTypes.VARIABLE_MULTIPLE_UNITS, cps_unit_checker.errors[0].ERROR_TYPE)
        self.assertFalse(cps_unit_checker.errors[0].is_warning)

    # MULTIPLICATION OF RADIANS
    def test_it_radian_multiplication_1(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_radian_multiplication_1.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        self.assertEqual(0, len(cps_unit_checker.errors))

    # MULTIPLICATION OF RADIANS 2
    def test_it_radian_multiplication_2(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_radian_multiplication_2.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        self.assertEqual(1, len(cps_unit_checker.errors))
        self.assertEqual(UnitErrorTypes.VARIABLE_MULTIPLE_UNITS, cps_unit_checker.errors[0].ERROR_TYPE)
        self.assertFalse(cps_unit_checker.errors[0].is_warning)

    # MULTIPLICATION OF RADIANS
    def test_it_radian_multiplication_3(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_radian_multiplication_3.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        self.assertEqual(0, len(cps_unit_checker.errors))

    # MULTIPLICATION OF RADIANS 2
    def test_it_radian_multiplication_4(self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_radian_multiplication_4.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        self.assertEqual(1, len(cps_unit_checker.errors))
        self.assertEqual(UnitErrorTypes.VARIABLE_MULTIPLE_UNITS, cps_unit_checker.errors[0].ERROR_TYPE)
        self.assertFalse(cps_unit_checker.errors[0].is_warning)


    # getXYZ 
    def test_it_getXYZ_1 (self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_getXYZ_1.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        # for e in cps_unit_checker.errors:
            # print '\nweak inference addition : %s warning:%s ' % (e.var_name, str(e.is_warning))
        self.assertEqual(0, len(cps_unit_checker.errors))

    # getXYZ 
    def test_it_getXYZ_2 (self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_getXYZ_2.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        # for e in cps_unit_checker.errors:
            # print '\nweak inference addition : %s warning:%s ' % (e.var_name, str(e.is_warning))
        var_name = 'tw.linear.x'
        var_linenr = 10
        my_oracle = [{'second': -1.0, 'meter': 1.0}, {'meter':1}]
        actual_units = None
        # is_unit_propagation_based_on_unknown_variable = False
        for s in cps_unit_checker.current_configuration.scopes:
            # for v in s.var_ordered_dict:
                # print v
            if s.className == 'main':
                if var_name in s.var_ordered_dict and var_linenr in s.var_ordered_dict[var_name]:
                    actual_units = s.var_ordered_dict[var_name][var_linenr]['units']
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))
        self.assertEqual(1, len(cps_unit_checker.errors))
        self.assertTrue(cps_unit_checker.errors[0].was_assigned_mutiple_units)


    # getXYZ 
    def test_it_getXYZ_3 (self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_getXYZ_3.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        var_name = 'tw.linear.x'
        var_linenr = 10
        my_oracle = [{'second': -1.0, 'meter': 1.0}, {'quaternion':1}]
        actual_units = None
        # is_unit_propagation_based_on_unknown_variable = False
        for s in cps_unit_checker.current_configuration.scopes:
            # for v in s.var_ordered_dict:
                # print v
            if s.className == 'main':
                if var_name in s.var_ordered_dict and var_linenr in s.var_ordered_dict[var_name]:
                    actual_units = s.var_ordered_dict[var_name][var_linenr]['units']
        self.assertEquals(actual_units, my_oracle, 'Incorrect units assigned to symbol:x Expected: %s received %s' %  (my_oracle, actual_units))
        self.assertEqual(1, len(cps_unit_checker.errors))
        self.assertTrue(cps_unit_checker.errors[0].was_assigned_mutiple_units)

    # getXYZ 
    def test_it_getXYZ_4 (self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_getXYZ_4.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        # for e in cps_unit_checker.errors:
            # print '\nweak inference addition : %s warning:%s ' % (e.var_name, str(e.is_warning))
        self.assertEqual(1, len(cps_unit_checker.errors))
        self.assertEqual(UnitErrorTypes.VARIABLE_MULTIPLE_UNITS, cps_unit_checker.errors[0].ERROR_TYPE)
        self.assertFalse(cps_unit_checker.errors[0].is_warning)

    # getXYZ 
    def test_it_getXYZ_5 (self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_getXYZ_5.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        self.assertEqual(1, len(cps_unit_checker.errors))
        self.assertEqual(UnitErrorTypes.VARIABLE_MULTIPLE_UNITS, cps_unit_checker.errors[0].ERROR_TYPE)
        self.assertFalse(cps_unit_checker.errors[0].is_warning)

    # QUATERNION ADDITION 1
    def test_it_quaternion_addition_1 (self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_quaternion_addition_1.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        self.assertEqual(2, len(cps_unit_checker.errors))
        self.assertEqual(UnitErrorTypes.VARIABLE_MULTIPLE_UNITS, cps_unit_checker.errors[0].ERROR_TYPE)
        self.assertFalse(cps_unit_checker.errors[0].is_warning)
        self.assertEqual(UnitErrorTypes.ADDITION_OF_INCOMPATIBLE_UNITS, cps_unit_checker.errors[1].ERROR_TYPE)
        self.assertFalse(cps_unit_checker.errors[1].is_warning)


    # QUATERNION ADDITION 2
    def test_it_quaternion_addition_2 (self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_quaternion_addition_2.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        self.assertEqual(1, len(cps_unit_checker.errors))
        self.assertEqual(UnitErrorTypes.VARIABLE_MULTIPLE_UNITS, cps_unit_checker.errors[0].ERROR_TYPE)
        self.assertFalse(cps_unit_checker.errors[0].is_warning)


    # QUATERNION ADDITION 3
    def test_it_quaternion_addition_3 (self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_quaternion_addition_3.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        self.assertEqual(0, len(cps_unit_checker.errors))

    # QUATERNION ADDITION 4
    def test_it_quaternion_addition_4 (self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_quaternion_addition_4.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        self.assertEqual(0, len(cps_unit_checker.errors))

    # QUATERNION MULTIPLICATION 1
    def test_it_quaternion_multiplication_1 (self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_quaternion_multiplication_1.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        self.assertEqual(0, len(cps_unit_checker.errors))

    # QUATERNION MULTIPLICATION 2
    def test_it_quaternion_multiplication_2 (self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_quaternion_multiplication_2.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        self.assertEqual(0, len(cps_unit_checker.errors))

    # QUATERNION MULTIPLICATION 3
    def test_it_quaternion_multiplication_3 (self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_quaternion_multiplication_3.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        self.assertEqual(0, len(cps_unit_checker.errors))

    # QUATERNION MULTIPLICATION 4
    def test_it_quaternion_multiplication_4 (self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_quaternion_multiplication_4.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        self.assertEqual(0, len(cps_unit_checker.errors))

    # QUATERNION MULTIPLICATION CLOSURE
    def test_it_quaternion_closed_under_multiplication_1 (self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_quaternion_closed_under_multiplication_1.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        self.assertEqual(0, len(cps_unit_checker.errors))


    # QUATERNION MULTIPLICATION CLOSURE
    def test_it_quaternion_closed_under_multiplication_2 (self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_quaternion_closed_under_multiplication_2.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        self.assertEqual(1, len(cps_unit_checker.errors))
        self.assertEqual(UnitErrorTypes.VARIABLE_MULTIPLE_UNITS, cps_unit_checker.errors[0].ERROR_TYPE)
        self.assertFalse(cps_unit_checker.errors[0].is_warning)

    # RADIAN MULTIPLICATION CLOSURE
    def test_it_radian_closed_under_multiplication_1 (self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_radian_closed_under_multiplication_1.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        self.assertEqual(0, len(cps_unit_checker.errors))

    # RADIAN MULTIPLICATION CLOSURE
    def test_it_radian_closed_under_multiplication_2 (self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_radian_closed_under_multiplication_2.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        self.assertEqual(1, len(cps_unit_checker.errors))
        self.assertEqual(UnitErrorTypes.VARIABLE_MULTIPLE_UNITS, cps_unit_checker.errors[0].ERROR_TYPE)
        self.assertFalse(cps_unit_checker.errors[0].is_warning)


    # dt Heuristic
    def test_it_dt_heuristic (self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_dt_heuristic_1.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        self.assertEqual(0, len(cps_unit_checker.errors))

    # dt Heuristic
    def test_it_plus_equals_1 (self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_plus_equals_1.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        self.assertEqual(0, len(cps_unit_checker.errors))

    # dt Heuristic
    def test_it_range_1 (self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_range_1.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        self.assertEqual(0, len(cps_unit_checker.errors))


    # same named argument in interface scope bug
    def test_it_scope_bug_1 (self):
        cps_unit_checker = CPSUnitsChecker()
        cps_unit_checker.debug = False
        cps_unit_checker.debug_print_AST = False
        dump_file  = './dump_files_for_tests/test_it_cppcheck_scope_bug_at_argument_1.cpp.dump'
        source_file  = dump_file.replace('.dump','')
        cps_unit_checker.main_run_check(dump_file, source_file)
        self.assertEqual(0, len(cps_unit_checker.errors))

if __name__ == '__main__':
    unittest.main()

