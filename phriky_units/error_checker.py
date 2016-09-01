#!/usr/bin/python

from unit_error import UnitError
from unit_error_types import UnitErrorTypes
from tree_walker import TreeWalker
from symbol_helper import SymbolHelper
from collections import OrderedDict
import os.path

class ErrorChecker:
    ''' IMPLEMENTATION OF MAIN ERROR CHECKING
    '''
    #warnings = [] 

    def __init__(self, debug, source_file, cps_units_checker=None):
        self.unit_errors = []
	self.debug	 = debug  # CONTROLS STD OUT PRINTING
	self.debug_verbose= False  
	self.file_under_analysis_name	= ''
	self.file_under_analyais_URI 	= source_file
        self.all_errors                 = []
        # self.all_warnings               = []
        self.git_version_of_FUA         = ''   # FUA is FILE UNDER ANALYSIS
        self.symbol_helper              = SymbolHelper(None)
        self.source_file_exists = False
        self.source_file_lines = []
        self.prepare_source_file_for_reading()
        self.cps_units_checker           = cps_units_checker
        self.have_found_addition_error_on_this_line = False


    def prepare_source_file_for_reading(self):
        # OPEN SOURCE FILE IF ERRORS FOUND
        self.source_file_exists = os.path.isfile(self.file_under_analyais_URI)
        self.source_file_lines = []
        if self.source_file_exists:
            # OPENS THE ORIGINAL FILE 
            with open(self.file_under_analyais_URI, 'r') as f:
                self.source_file_lines = f.readlines()
        else:
            if self.debug:
                print 'No source file found at: %s' % self.file_under_analyais_URI


    def error_check_function_args_consistent(self, cppcheck_configuration_unit):
        ''' VERIFIES UNIT CONSISTENCY OF FUNCTIONS AT EVERY CALL POINT
            input: cppcheck configuration unit from dump
            returns: none
            side_effects:  might add UnitError objects to self.all_errors list
            '''
        # FOR EACH FUNCTION
        for f in cppcheck_configuration_unit.functions:
            # FOR EACH ARG IN A FUNCTION
            for arg_list_of_call_points in f.arg_units:
                # FOR EACH TIME THE FUNCTION WAS CALLED
                error_found = False
                new_error = UnitError(self.cps_units_checker)
                first_call_point_with_units = None
                for call_point in arg_list_of_call_points:
                    # FIND SOME CALL POINT WITH UNITS
                    if not first_call_point_with_units:
                        if call_point['units']:
                            if self.debug:
                                print 'CALL POINT UNITS:' + str(call_point['units'])
                            first_call_point_with_units = call_point
                        continue
                    # CHECK UNITS OF FIRST CALL POINT AGAINST ALL OTHERS
                         #HAS UNITS  and UNITS ARE DIFFERENT THAN SOME OTHER CALL POINT
                    if call_point['units'] and (call_point['units'] != first_call_point_with_units['units']):
                        error_found = True
                        # FOUND DIFFERENT UNITS AT TWO DIFFERENT CALL POINTS
                        new_error.var_name = f.name
                        new_error.ERROR_TYPE = UnitErrorTypes.FUNCTION_CALLED_WITH_DIFFERENT_UNIT_ARGUMENTS
                        new_error.token = first_call_point_with_units['token']
                        new_error.f = first_call_point_with_units['token']
                        # FIRST ASSIGNMENT
                        new_error.set_primary_line_number(first_call_point_with_units['linenr'])
                        new_error.linenr_at_first_unit_assignment = first_call_point_with_units['linenr']
                        new_error.units_at_first_assignment = first_call_point_with_units['units']
                        # SECOND (DIFFERENT) ASSIGNMENT
                        new_error.linenr_of_multiple_unit_assignment = call_point['linenr']
                        new_error.units_when_multiple_happened = call_point['units']
                        break # FOR LOOP
                if error_found:
                    # GET LINE FROM ORIGINAL FILE IF IT EXISTS
                    if self.source_file_exists:
                        # todo: resolve relative link to different file and load source
                        if new_error.linenr_at_first_unit_assignment <= len(self.source_file_lines):
                            if new_error.linenr_of_multiple_unit_assignment <= len(self.source_file_lines):
                                new_error.source_code_at_first_assignment = self.source_file_lines[new_error.linenr_at_first_unit_assignment - 1].strip()
                                new_error.source_code_when_multiple_units_happened = self.source_file_lines[new_error.linenr_of_multiple_unit_assignment - 1].strip()
                    # COLLECT ERROR
                    self.all_errors.append(new_error)

    def error_check_addition_of_incompatible_units(self, sorted_analysis_unit_dict):
        ''' ERROR CHECK ADDITIONAL OF INCOMPATIBLE UNITS
            input: cppcheck configuration unit from dump
            returns: none
            side_effects:  might add UnitError objects to self.all_errors list
            '''            
        if self.debug:
            print 'call to error_check_addition_of_incompatible_units'
        for function_dict in sorted_analysis_unit_dict.values():
            tw = TreeWalker(function_dict)  # ARGUMENT 'FUNCTION DICTIONARY' IS USED BY SYMBOL TABLE
            self.have_found_addition_error_on_this_line = False
            for root_token in function_dict['root_tokens']:
                # tw.is_assignment_statement = False
                # tw.generic_recurse_and_apply_function(root_token, tw.find_assignment_tokens_recursive_target)
                # if not tw.is_assignment_statement:
                tw.generic_recurse_and_apply_function(root_token, self.error_check_addition_of_incompatible_units_recursive_target)


    def error_check_addition_of_incompatible_units_recursive_target(self, token, left_token, right_token):
        ''' ERROR CHECK ADDITIONAL OF INCOMPATIBLE UNITS - RESURSIVE TARGET
            input:  token   cppcheck token object
                    left_token, right_token
            returns: nothing, with possible side effect of adding errors
            '''            
        if token.str in ['+', '-', '+=', '-=']:
        # if token.isArithmeticalOp and token.str in ['+', '-', '+=', '-=']:
            if self.debug:
                print 'found + or - or += or -='
            # THIS IS ADDITION OR SUBTRACTION
            # assert(token.astOperand1 and token.astOperand2)
            if token.astOperand1 and token.astOperand2:  # WEIRD IF THIS WERE FALSE
                if token.astOperand1.units and token.astOperand2.units:        
                    # BOTH CHILDREN HAVE UNITS
                    if token.astOperand1.units != token.astOperand2.units:
                        if not self.have_found_addition_error_on_this_line:
                            # UNIT MISMATCH ON ADDITION : REPORT ERROR
                            new_error = UnitError(self.cps_units_checker)
                            new_error.var_name = '' # NO VAR NAME SINCE ITS A COMPARISON OPERATORS
                            new_error.ERROR_TYPE = UnitErrorTypes.ADDITION_OF_INCOMPATIBLE_UNITS
                            new_error.is_unit_propagation_based_on_constants = token.is_unit_propagation_based_on_constants
                            new_error.is_unit_propagation_based_on_unknown_variable = token.is_unit_propagation_based_on_unknown_variable
                            if token.is_unit_propagation_based_on_constants or token.is_unit_propagation_based_on_unknown_variable:
                                new_error.is_warning = True
                            # LINENR 
                            new_error.linenr = token.linenr
                            new_error.token_left = left_token
                            new_error.token_right = right_token
                            new_error.token = token
                            # GET LINE FROM ORIGINAL FILE IF IT EXISTS
                            if self.source_file_exists:
                                new_error.source_code_at_first_assignment = self.source_file_lines[new_error.linenr_at_first_unit_assignment - 1].strip()
                                new_error.source_code_when_multiple_units_happened = self.source_file_lines[new_error.linenr_of_multiple_unit_assignment - 1].strip()
                            # COLLECT ERROR
                            self.all_errors.append(new_error)
                            self.have_found_addition_error_on_this_line = True


    def error_check_comparisons(self, sorted_analysis_unit_dict):
        ''' ERR CHECK COMPARISION OF UNITS OVER LOGICAL OPERATORS  
            input: cppcheck configuration unit from dump
            returns: none
            side_effects:  might add UnitError objects to self.all_errors list
            '''            
        for function_dict in sorted_analysis_unit_dict.values():
            tw = TreeWalker(function_dict)  # ARGUMENT 'FUNCTION DICTIONARY' IS USED BY SYMBOL TABLE
            for root_token in function_dict['root_tokens']:
                tw.generic_recurse_and_apply_function(root_token, self.error_check_comparison_recursive_target)

    def error_check_comparison_recursive_target(self, token, left_token, right_token):
        ''' COMPARISON OPERATORS - MUST BE THE SAME ON BOTH SIDES
            input:  token   cppcheck token object
                    left_token, right_token
            returns: nothing, with possible side effect of adding errors
            '''
        if self.debug_verbose and self.cps_units_checker:
            self.cps_units_checker.handle_debug_verbose(inspect.stack()[0][3])
        if token.isComparisonOp:
            if left_token and left_token.units and right_token and right_token.units:  # BOTH HAVE UNITS
                if left_token.units != right_token.units:
                    # CREATE VARIABLE ERROR OBJECT
                    new_error = UnitError(self.cps_units_checker)
                    new_error.var_name = '' # NO VAR NAME SINCE ITS A COMPARISON OPERATORS
                    new_error.ERROR_TYPE = UnitErrorTypes.COMPARISON_INCOMPATIBLE_UNITS
                    # LINENR 
                    new_error.linenr = token.linenr
                    new_error.token_left = left_token
                    new_error.token_right = right_token
                    new_error.token = token
                    # new_error.is_unit_propagation_based_on_constants = inner_dict['is_unit_propagation_based_on_constants']
                    # new_error.is_unit_propagation_based_on_unknown_variable = inner_dict['is_unit_propagation_based_on_unknown_variable']
                    # if inner_dict['is_unit_propagation_based_on_constants'] or inner_dict['is_unit_propagation_based_on_unknown_variable']:
                        # new_error.is_warning = True
                    # GET LINE FROM ORIGINAL FILE IF IT EXISTS
                    if self.source_file_exists:
                        new_error.source_code_at_first_assignment = self.source_file_lines[new_error.linenr_at_first_unit_assignment - 1].strip()
                        new_error.source_code_when_multiple_units_happened = self.source_file_lines[new_error.linenr_of_multiple_unit_assignment - 1].strip()
                    # COLLECT ERROR
                    self.all_errors.append(new_error)



    def error_check_logical_operators(self, sorted_analysis_unit_dict):
        ''' ERR CHECK UNITS DURING LOGICAL OPERATIONS
            input: cppcheck configuration unit from dump
            returns: none
            side_effects:  might add UnitError objects to self.all_errors list
            '''
        for function_dict in sorted_analysis_unit_dict.values():
            tw = TreeWalker(function_dict)  # ARGUMENT 'FUNCTION DICTIONARY' IS USED BY SYMBOL TABLE
            for root_token in function_dict['root_tokens']:
                tw.generic_recurse_and_apply_function(root_token, self.error_check_logical_recursive_target)
    
    def error_check_logical_recursive_target(self, token, left_token, right_token):
        if self.debug_verbose:
            self.cps_units_checker.handle_debug_verbose(inspect.stack()[0][3])
        if token.isOp and token.str in ['&&', '||', '!']: # NOT << and >> since theses are used in PRINT OUTS
            if (left_token and left_token.units) or (right_token and right_token.units):
                # CREATE VARIABLE ERROR OBJECT
                new_error = UnitError(self.cps_units_checker)
                new_error.var_name = '' # NO VAR NAME SINCE ITS A COMPARISON OPERATORS
                new_error.ERROR_TYPE = UnitErrorTypes.LOGICAL_OPERATOR_USED_ON_UNITS
                # LINENR 
                new_error.set_primary_line_number(token.linenr)
                new_error.token_left = left_token
                new_error.token_right = right_token
                new_error.token = token
                # GET LINE FROM ORIGINAL FILE IF IT EXISTS
                if self.source_file_exists:
                    new_error.source_code_at_first_assignment = self.source_file_lines[new_error.linenr_at_first_unit_assignment - 1].strip()
                    new_error.source_code_when_multiple_units_happened = self.source_file_lines[new_error.linenr_of_multiple_unit_assignment - 1].strip()
                # COLLECT ERROR
                self.all_errors.append(new_error)


    def error_check_unit_smell(self, cppcheck_configuration_unit):
        ''' UNIT_SMELL ERROR CHECKING IMPLEMENTATION 
            input: cppcheck configuration unit from dump
            returns: none
            side_effects:  might add UnitError objects to self.all_errors list
            '''            
        # SANITY DEFENSE
        if not cppcheck_configuration_unit:
            return
	# SEARCH SCOPE OBJECTS FOR ERRORS
	for s in cppcheck_configuration_unit.scopes:
	    if s.var_ordered_dict:
		# FOR EACH VARIABLE
		for var_name, dict_of_assignments in s.var_ordered_dict.iteritems():
		    # FOR EACH LIST OF UNITS ASSIGNED TO A VARIABLE
		    for linenr, inner_dict in dict_of_assignments.iteritems():
                        units_list = inner_dict['units']
			# TEST FOR MULTIPLE UNITS ASSIGNED
                        for u in units_list:
                            # WRITES ALL UNITS OUT TO A FILE FOR EMPIRICALLY LOOKING FOR UNUSUAL UNITS
                            # with open('2016_07_20_1026_list_of_all_units.txt', 'a') as f:
                                # f.write(str(u) + "\n")
                            if u not in self.symbol_helper.list_of_vetted_units:
                                # CREATE VARIABLE ERROR/WARNING OBJECT
                                new_error = UnitError(self.cps_units_checker)
                                new_error.var_name = var_name
                                new_error.ERROR_TYPE = UnitErrorTypes.UNIT_SMELL
                                # LINENR OF FIRST ASSIGNMENT
                                new_error.linenr_at_first_unit_assignment = s.var_ordered_dict[var_name].keys()[0] # WORKS BECAUSE ORDERED DICT 
                                new_error.set_primary_line_number(s.var_ordered_dict[var_name].keys()[0]) # WORKS BECAUSE ORDERED DICT 
                                # UNITS AT FIRST ASSIGNMENT
                                new_error.units_at_first_assignment = s.var_ordered_dict[var_name].values()[0]['units'] # WORKS BECAUSE ORDERED DICT 
                                new_error.all_units_assigned_to_var_as_dict = s.var_ordered_dict[var_name]
                                new_error.token = s.var_ordered_dict[var_name].values()[0]['token'] # WORKS BECAUSE ORDERED DICT 
                                new_error.is_unit_propagation_based_on_constants = inner_dict['is_unit_propagation_based_on_constants']
                                new_error.is_unit_propagation_based_on_unknown_variable = inner_dict['is_unit_propagation_based_on_unknown_variable']
                                if inner_dict['is_unit_propagation_based_on_constants'] or inner_dict['is_unit_propagation_based_on_unknown_variable']:
                                    new_error.is_warning = True
                                # GET LINE FROM ORIGINAL FILE IF IT EXISTS
                                if self.source_file_exists:
                                    if new_error.linenr_at_first_unit_assignment <= len(self.source_file_lines):
                                        if new_error.linenr_of_multiple_unit_assignment <= len(self.source_file_lines):
                                            new_error.source_code_at_first_assignment = self.source_file_lines[new_error.linenr_at_first_unit_assignment - 1].strip()
                                            new_error.source_code_when_multiple_units_happened = self.source_file_lines[new_error.linenr_of_multiple_unit_assignment - 1].strip()
                                # COLLECT ERROR / WARNING
                                self.all_errors.append(new_error)
                                break

    def error_check_multiple_units(self, cppcheck_configuration_unit):
        ''' MULTIPLE_UNIT_TYPE ASSIGNMENT ERROR CHECKING IMPLEMENTATION 
            input: cppcheck configuration unit from dump
            returns: none
            side_effects:  might add UnitError objects to self.all_errors list
            '''            
        # SANITY DEFENSE
        if not cppcheck_configuration_unit:
            return
        vars_with_multi_units_dict = OrderedDict() # {scope_id: [list_of_vars_with_multi_units]}
	# SEARCH SCOPE OBJECTS FOR CHECK FOR ERRORS
	for s in cppcheck_configuration_unit.scopes:
	    #if self.debug:
		#print '- '*42
	    if s.var_ordered_dict:
		# FOR EACH VARIABLE
		for var_name, dict_of_assignments in s.var_ordered_dict.iteritems():
                    # have_found_multi_units_for_this_var = False
		    # FOR EACH LIST OF UNITS ASSIGNED TO A VARIABLE
		    for linenr, inner_dict in dict_of_assignments.iteritems():
                        units_list = inner_dict['units']
			# TEST FOR MULTIPLE UNITS ASSIGNED
			if len(units_list) > 1:
                            # CREATE VARIABLE ERROR/WARNING OBJECT
                            new_error = UnitError(self.cps_units_checker)
                            new_error.var_name = var_name
                            new_error.ERROR_TYPE = UnitErrorTypes.VARIABLE_MULTIPLE_UNITS
                            # LINENR OF FIRST ASSIGNMENT
                            new_error.linenr_at_first_unit_assignment = s.var_ordered_dict[var_name].keys()[0] # WORKS BECAUSE ORDERED DICT 
                            # UNITS AT FIRST ASSIGNMENT
                            new_error.units_at_first_assignment = s.var_ordered_dict[var_name].values()[0]['units'] # WORKS BECAUSE ORDERED DICT 
                            new_error.all_units_assigned_to_var_as_dict = s.var_ordered_dict[var_name]
                            new_error.token = s.var_ordered_dict[var_name].values()[0]['token'] # WORKS BECAUSE ORDERED DICT 
                            new_error.is_unit_propagation_based_on_constants = inner_dict['is_unit_propagation_based_on_constants']
                            new_error.is_unit_propagation_based_on_unknown_variable = inner_dict['is_unit_propagation_based_on_unknown_variable']
                            if inner_dict['is_unit_propagation_based_on_constants'] or inner_dict['is_unit_propagation_based_on_unknown_variable']:
                                new_error.is_warning = True
                            for linenr, units_list in s.var_ordered_dict[var_name].iteritems():
                                if 'units' in units_list:
                                    if len(units_list['units']) > 1:
                                        # LINENR AT ERROR ASSIGNMENT
                                        new_error.set_primary_line_number(linenr)
                                        # UNITS AT ERROR ASSIGNMENT
                                        new_error.units_when_multiple_happened = units_list
                                        break # REPORT THE FIRST
                            # GET LINE FROM ORIGINAL FILE IF IT EXISTS
                            if self.source_file_exists:
                                if new_error.linenr_at_first_unit_assignment <= len(self.source_file_lines):
                                    if new_error.linenr_of_multiple_unit_assignment <= len(self.source_file_lines):
                                        new_error.source_code_at_first_assignment = self.source_file_lines[new_error.linenr_at_first_unit_assignment - 1].strip()
                                        new_error.source_code_when_multiple_units_happened = self.source_file_lines[new_error.linenr_of_multiple_unit_assignment - 1].strip()
                            # WAS VARIABLE ASSIGNED MULTIPLE UNITS
                            if new_error.linenr_at_first_unit_assignment != new_error.linenr_of_multiple_unit_assignment:
                                new_error.was_assigned_mutiple_units = True
                            # COLLECT ERROR / WARNING
                            self.all_errors.append(new_error)
			    # MULTIPLE UNITS DETECTED
                            # have_found_multi_units_for_this_var = True
                            break
                    # if have_found_multi_units_for_this_var:
                        # break


    def pretty_print(self, show_high_confidence=True, show_low_confidence=False):
        ''' PRINTS ERRORS TO STD OUT, ATTEMPTS TO BE HUMAN READABLE
        '''
        for e in self.all_errors:
            is_high_confidence = not e.is_warning
            is_low_confidence = e.is_warning

            if is_high_confidence and not show_high_confidence:
                continue
            if is_low_confidence and not show_low_confidence:
                continue

            print '- '*42

            if e.is_warning:
                confidence = 'low'
            else:
                confidence = 'high'

            if e.ERROR_TYPE == UnitErrorTypes.VARIABLE_MULTIPLE_UNITS:
                # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
                # MUTIPLE UNITS ON VARIABLE
                # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
                print UnitErrorTypes().get_err_short_discription(e.ERROR_TYPE)
                # print 'File:  %s' % self.file_under_analyais_URI
                error_file = e.get_file_URI_where_error_occured()
                # if error_file != self.file_under_analyais_URI:
                    # print 'File:  %s' % self.error_file
                # print 'File:  %s' % self.file_under_analyais_URI
                linenr_of_multiple_assignment = 0
                # if e.was_assigned_mutiple_units:

                # print "Assignment of multiple units on line %s with %s-confidence. units: %s" % (e.linenr, confidence, e.units_when_multiple_happened)
                linenr = e.linenr
                units = []
                if 'units' in e.units_when_multiple_happened:
                    units = e.units_when_multiple_happened['units']

                print "Assignment of multiple units on line %s with %s-confidence. units: %s" % (linenr, confidence, units)
                # print e.all_units_assigned_to_var_as_dict
                # print 'UNITS FIRST ASSIGNED LINE %s : %s' % (e.linenr_at_first_unit_assignment, e.units_at_first_assignment)
                # print 'SOURCE : %s' % e.source_code_at_first_assignment
                # print 'UNITS BECAME MULTIPLE AT LINE %s : %s' % (e.linenr_of_multiple_unit_assignment, e.units_when_multiple_happened)
                # print 'SOURCE : %s' % e.source_code_when_multiple_units_happened
                linenr_of_multiple_assignment = e.linenr_of_multiple_unit_assignment
                # else:
                    # print 'MULTIPLE UNITS LINE %s : %s' % (e.linenr_at_first_unit_assignment, e.units_at_first_assignment)
                    # print 'SOURCE : %s' % e.source_code_at_first_assignment
                    # linenr_of_multiple_assignment = e.linenr_at_first_unit_assignment
                # print
                # print 'Cause: A variable %s was assigned multiple units on line %s' % (e.var_name, e.linenr_at_first_unit_assignment)
                # print '- '*42
            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
            # FUNCTION CALLED WITH DIFFERENT UNIT ARGUMENTS
            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
            if e.ERROR_TYPE == UnitErrorTypes.FUNCTION_CALLED_WITH_DIFFERENT_UNIT_ARGUMENTS:
                linenr_first = e.linenr_at_first_unit_assignment
                linenr_second  = e.linenr_of_multiple_unit_assignment
                # print 'File:  %s' % self.file_under_analyais_URI
                print
                print 'Cause: Function %s invoked with different units:' % e.var_name
                print 'LINE             UNITS'
                print '-----            -----------'
                for u in e.units_at_first_assignment:
                    print '{:5d}           '.format(linenr_first),
                    for k,v in u.iteritems():
                        print '{:10s} {:3d}  '.format(k, v), 
                    print
                for u in e.units_when_multiple_happened:
                    print '{:5d}           '.format(linenr_second),
                    for k,v in u.iteritems():
                        print '{:10s} {:3d}  '.format(k, v), 
                    print
                print
                print '{:5d} {:80s}'.format(linenr_first, e.source_code_at_first_assignment)
                print '{:5d} {:80s}'.format(linenr_second, e.source_code_when_multiple_units_happened)
                # print '- '*42

            if e.ERROR_TYPE == UnitErrorTypes.ADDITION_OF_INCOMPATIBLE_UNITS:
                print UnitErrorTypes().get_err_short_discription(e.ERROR_TYPE)
                units_left = []
                if e.token_left and e.token_left.units:
                    units_left = e.token_left.units
                units_right = []
                if e.token_right and e.token_right.units:
                    units_right = e.token_right.units
                print "Addition of inconsistent units on line %s with %s-confidence. Attempting to add %s to %s." % (e.linenr, confidence, units_left, units_right)
            if e.ERROR_TYPE == UnitErrorTypes.COMPARISON_INCOMPATIBLE_UNITS:
                print UnitErrorTypes().get_err_short_discription(e.ERROR_TYPE)
                units_left = []
                if e.token_left and e.token_left.units:
                    units_left = e.token_left.units
                units_right = []
                if e.token_right and e.token_right.units:
                    units_right = e.token_right.units
                print "Comparison of inconsistent units on line %s with %s-confidence. Attempting to compare %s to %s." % (e.linenr, confidence, units_left, units_right)






