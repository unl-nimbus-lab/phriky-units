#!/usr/bin/env python
#Copyright (c) 2016, University of Nebraska NIMBUS LAB  John-Paul Ore jore@cse.unl.edu
#All rights reserved.

#Redistribution and use in source and binary forms, with or without
#modification, are permitted provided that the following conditions are met:

#1. Redistributions of source code must retain the above copyright notice, this
   #list of conditions and the following disclaimer.
   #2. Redistributions in binary form must reproduce the above copyright notice,
      #this list of conditions and the following disclaimer in the documentation
         #and/or other materials provided with the distribution.

         #THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
         #ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
         #WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
         #DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
         #ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
         #(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
         #LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
         #ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
         #(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
         #SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

         #The views and conclusions contained in the software and documentation are those
         #of the authors and should not be interpreted as representing official policies,
         #either expressed or implied, of the FreeBSD Project.

#           _          _ _                        _ _
#     _ __ | |__  _ __(_) | ___   _   _   _ _ __ (_) |_ ___
#    | '_ \| '_ \| '__| | |/ / | | | | | | | '_ \| | __/ __|
#    | |_) | | | | |  | |   <| |_| | | |_| | | | | | |_\__ \
#    | .__/|_| |_|_|  |_|_|\_\\__, |  \__,_|_| |_|_|\__|___/
#    |_|                      |___/


import sys
import os
import cppcheckdata                 # http://cppcheck.sourceforge.net/
from tree_walker import TreeWalker
import fileinput
# egg_path='./networkx/networkx-1.11-py2.7.egg'
# sys.path.append(egg_path)
import networkx as nx
from collections import OrderedDict
import inspect
from operator import attrgetter
from error_checker import ErrorChecker
from unit_error_types import UnitErrorTypes


#messages = []


class CPSUnitsChecker:
    ''' PHYSICAL UNITS CHECKING   
    '''

    def __init__(self):
        self.VERSION                    = 0.21
        self.debug                      = False
        self.debug_scope                = False
        self.debug_print_AST            = False
        self.debug_print_function_topo_sort = False
        self.debug_LHS                  = False
        self.debug_verbose              = False
        self.debug_configuration_count  = 0
        self.debug_function_count       = 0
        self.only_scan_these_functions  = []
        self.symbol_dict                = {}
        self.source_file                = ''
        self.errors                     = []  
        self.warnings                   = []
        self.error_checker              = None
        self.current_file_under_analysis= ''
        self.tw                         = None  # TREE WALKER
        self.source_file_lines          = []
        self.function_graph = nx.DiGraph()
        self.all_function_graphs        = []
        self.all_sorted_analysis_unit_dicts = []
        self.should_sort_by_function_graph = True
        self.count_of_files_containing_ros_units = 0
        self.found_ros_units_in_this_file = False
        self.all_tree_walkers           = []
        self.current_configuration      = None
        self.should_abandon_early       = True
        self.SHOULD_ONLY_FIND_FILES_WITH_UNITS = False
        self.SHOULD_WRITE_RESULTS_TO_DATABASE = False
        self.SHOULD_FIND_ALL_UNITS      = False
        self.CWD                        = '/Users/jore/courses/NIMBUS/RESEARCH/CPS_TYPES/cps_units'
        self.db_analysis_audit_row_id         = 0   # USED DURING INSERT
        self.current_repository         = ''   # USED DURING INSERT
        # http://stackoverflow.com/questions/14989858/get-the-current-git-hash-in-a-python-script
        self.git_version_command        = 'git describe --tags --always'  # MIGHT NEED THIS
        # INITIALIZE DATA STRUCTURE TO COUNT ERRORS BY TYPE
        self.list_of_error_counts_by_type = [0 for x in UnitErrorTypes().ERR_TYPE_NAMES]
        if self.debug_verbose:
            print inspect.stack()[0][3]
        self.all_classes_and_units_for_this_file_as_dict = {}



    def add_class_and_units_to_all_list(self, class_name, units):
        ''' WHEN TRACKING ALL UNIT TYPES, ADD THEM TO DATA STRUCTURE HERE
            input:  class_name str the name of the ROS class or str (ex: 'atan2') that merits units
                    units : the dictionary or units
            output: None, side effect updates local data structures.
            '''
        if not class_name in self.all_classes_and_units_for_this_file_as_dict:
            self.all_classes_and_units_for_this_file_as_dict[class_name] = units
        else:
            pass  # WE DON'T CASE BECAUSE WE'RE ONLY TRACKING UNIQUE UNITS, NOT COUNTS




    def handle_debug_verbose(self, message_string):
        print ("%d %d %s" % (self.debug_configuration_count,  self.debug_function_count, message_string))

    def handle_command_line_input(self): 
        for line in fileinput.input():
            print line

    def find_functions(self, a_cppcheck_configuration):
        ''' LINEAR SCAN THROUGH TOKENS TO FIND 'function' TOKENS THAT HAVE GLOBAL SCOPE.
            COLLECT AND RETURN DICT CONTAINING FUNCTION START AND END TOKEN POINTERS 

            returns: dict containing function start and end tokens
        '''
        function_dicts = {}

        if self.debug_verbose:
            print inspect.stack()[0][3]
        # FIND FUNCTIONS IN 'SCOPES' REGION OF DUMP FILE, START AND END TOKENs
        for s in a_cppcheck_configuration.scopes:
            if s.type=='Function': 
                # SCAN ALL FUNCTIONS UNLESS LIST OF FUNCTIONS SPECIFIED
                if self.only_scan_these_functions:
                    if  s.className not in self.only_scan_these_functions:
                        print 'cont...'
                        continue
                function_dicts[s.Id] = {'name': s.className,
                                        'linern': s.classStart.linenr,
                                        'tokenStart': s.classStart, 
                                        'tokenEnd': s.classEnd, 
                                        'scopeObject':s,
                                        'symbol_table':{},
                                        'function_graph_edges':[],
                                        'function':s.function}
                # CONSTRUCT LIST OF ROOT TOKENS
                function_dicts[s.Id]['root_tokens'] = self.find_root_tokens(s.classStart, s.classEnd)
                    
        if self.debug: 
            print "Found %d functions..." % len(function_dicts)
        return function_dicts


    def find_root_tokens(self, tokenStart, tokenEnd):
        ''' FOR A FUNCTION DEFIND AS ALL TOKENS FROM tokenStart TO tokenEnd, FIND THE ROOTS
            input: tokenStart  a CPPCheckData Token, first token in a function
            input: tokenEnd    a CPPCheckData Token, last token in a function

            output: a list of root_tokens, in flow order
            '''

        if self.debug_verbose:
            print inspect.stack()[0][3]
        root_tokens_set = set()
        current_token = tokenStart
        while(current_token != tokenEnd):  #todo: reverse token set exploration to top-down instead of bottom-up
            # HAS A PARENT
            if current_token.astParent: 
                a_parent = current_token.astParent
                has_parent = True
                while has_parent:
                    # HAS NO PARENT, THEREFORE IS ROOT
                    if not a_parent.astParent:     
                        root_tokens_set.add(a_parent)
                        a_parent.isRoot = True  # THIS PROPERTY IS A CUSTOM NEW PROPERTY
                        has_parent = False
                    else:
                        a_parent = a_parent.astParent 
            current_token = current_token.next

        
        root_tokens = list(root_tokens_set)
        # SORT NUMERICALLY BY LINE NUMBER
        root_tokens = sorted(root_tokens, key=lambda x : int(x.linenr))
        #for t in root_tokens:
            #print t.linenr
        return root_tokens


    def analyze_function(self, function_dict):
        
        if self.debug_verbose:
            print inspect.stack()[0][3]
        tw = TreeWalker(function_dict)  # ARGUMENT 'FUNCTION DICTIONARY' IS USED BY SYMBOL TABLE
        tw.cps_unit_checker = self
        tw.my_symbol_helper.cps_unit_checker = self
        self.tw = tw   # LINK CURRENT Tree Walker to self
        tw.debug = self.debug
        tw.debug_verbose = self.debug_verbose
        tw.current_file = self.current_file_under_analysis
        tw.source_file_lines = self.source_file_lines
        tw.source_file = self.source_file
        tw.should_check_unitless_during_multiplication = False # UNITLESS
        tw.my_symbol_helper.should_match_on_heuristic_variable_names = False
        tw.current_function_dict = function_dict

        found_units_in_this_function = False

        # ASSUME THE TOKENS COME BACK AS A SORTED LIST
        break_point = 1000
        i=0

        for root_token in function_dict['root_tokens']:
            # INITIALIZAE TREE WALKER FOR THIS ROOT TOKEN
            tw.was_some_unit_changed = True
            tw.is_unit_propagation_based_on_constants = False
            tw.is_unit_propagation_based_on_unknown_variable = False
            if self.SHOULD_ONLY_FIND_FILES_WITH_UNITS and self.found_ros_units_in_this_file and not self.SHOULD_FIND_ALL_UNITS:
                return
            # RESET THE TREE WALKER'S LINE NUMBERS
            tw.reset_min_max_line_numbers()
            # FIND THE MIN AND MAX LINE NUMBERS IN THIS AST : USED TO PROTECT LOOP FROM MULTI-LINE STATEMENTS
            tw.generic_recurse_and_apply_function(root_token, tw.find_min_max_line_numbers)
            
            # APPLY ROS UNITS TO VARIABLES
            tw.generic_recurse_and_apply_function(root_token, tw.apply_ROS_units)
            # APPLY UNITS FROM PREVIOUS ASSIGNMENTS IN SCOPE
            tw.generic_recurse_and_apply_function(root_token, tw.apply_units_from_scope)
            # APPLY UNITS FROM KNOWN FUNCTIONS - atan2
            tw.generic_recurse_and_apply_function(root_token, tw.apply_units_inverse_trig)
            # APPLY UNITS FROM KNOWN SYMBOLS - M_PI
            tw.generic_recurse_and_apply_function(root_token, tw.apply_units_known_symbols)
            # APPLY UNITS FROM KNOWN FUNCTION - toSec
            tw.generic_recurse_and_apply_function(root_token, tw.apply_units_toSec)
            # APPLY UNITS FROM KNOWN FUNCTION - getX, getY, getZ  
            tw.generic_recurse_and_apply_function(root_token, tw.apply_units_getXYZ)
            # APPLY UNITS FROM KNOWN FUNCTION - quatToRPY
            tw.generic_recurse_and_apply_function(root_token, tw.apply_units_quatToRPY)

            # CONTINUE TO ATTEMPT CHANGES UNTIL CHANGES CEASE
            while tw.was_some_unit_changed:  
                if i>break_point:
                    if self.debug_verbose:
                        s = "BREAKING QUIESSENCE (WHILE) LOOP AT %d" % break_point
                        raise ValueError(s)
                        return
                    break
                i+=1
                tw.was_some_unit_changed = False
                # LOOK FOR EARLY ABANDONMENT OF THIS AST
                if not tw.found_units_in_this_tree and self.should_abandon_early:
                    break
                # SPECIAL MODE - JUST FIND ROS FILES
                if self.SHOULD_ONLY_FIND_FILES_WITH_UNITS:
                    break
                ### PROPAGATE ACROSS '.'
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_across_dot_connectors)
                ### PROPAGATE ACROSS + - * /  += -= *= /=
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_across_operators)
                ### PROPAGATE ACROSS (
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_across_parenthesis)
                ### PROPAGATE ACROSS [
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_across_square_brackets)
                ### PROPAGATE INTO f(meters, arg2)  into function  - FUTURE: CONSTRAINT MODE
                #tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_into_function_args)
                ### PROPAGATE ACROSS '='
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_across_assignment)
                ### PROPAGATE ACROSS 'std::abs' 'std::fabs' 'abs' 'fabs'
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_math_abs_fabs_floor_ceil)
                ### PROPAGATE ACROSS 'std::min' 'std::max' 'min' 'max'
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_math_min_max)
                ### PROPAGATE ACROSS 'sqrt'
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_sqrt)
                ### PROPAGATE ACROSS 'getXYZ
                # tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_getXYZ)
                ### PROPAGATE ACROSS TERNARY
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_ternary)
                ### PROPAGATE ACROSS 'pow'
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_pow)
                ### PROPAGATE ACROSS 'atan2', 'acos', 'asin'
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_inverse_trig)
                ### PROPAGATE ACROSS 'return'
                tw.generic_recurse_and_apply_function(root_token, tw.propagate_units_across_return)
                ### COLLECT UNITS FROM FUNCTION CALL POINTS
                tw.generic_recurse_and_apply_function(root_token, tw.collect_function_parameters_units_and_decorate_function_objects)
                 #FUNCTIONS todo:
            # END -- WHILE LOOP
            #ASSIGNMENT STATEMENTS WITH UNITS - RIGHT TO LEFT PROPAGATION
            if root_token.isAssignmentOp and root_token.units:   
                # COLLECT LEFT SIDE OF ASSIGNMENT
                lhs_compound_name = tw.recurse_and_collect_string(root_token.astOperand1)
                # ADD UNITS TO VARIABLE IN SCOPE
                tw.return_value_list = []
                tw.generic_recurse_and_apply_function(root_token.astOperand1, tw.find_first_variable)
                if len(tw.return_value_list) != 1:
                    if len(tw.return_value_list) == 0:
                        if self.debug and self.debug_LHS:
                            print "%s %s LHS no variable found not found" % (root_token.file, root_token.linenr)
                    else:
                        if self.debug and self.debug_LHS:
                            for value in tw.return_value_list:
                                print "%s %s LHS Variable found: %s" % (root_token.file, root_token.linenr, value)
                    continue # todo track count of this error
                lhs_var= tw.return_value_list[0]
                lhs_scope = lhs_var.nameToken.scope
                if lhs_var.isArgument:
                    lhs_scope = function_dict['scopeObject']
                        
                line_num = int(root_token.linenr)
                if lhs_compound_name in lhs_scope.var_ordered_dict:
                    # ALREADY HAS ASSIGNMENT, ADD TO IT 
                    lhs_scope.var_ordered_dict[lhs_compound_name][line_num] =  {
                                    'units':root_token.units,  # THIS IS A LIST 
                                    'token':root_token,
                                    'is_unit_propagation_based_on_constants':tw.is_unit_propagation_based_on_constants,
                                    'is_unit_propagation_based_on_unknown_variable':tw.is_unit_propagation_based_on_unknown_variable}
                else:
                    lhs_scope.var_ordered_dict[lhs_compound_name] = {
                            line_num : {'units':root_token.units, 
                                        'token':root_token,
                                        'is_unit_propagation_based_on_constants':tw.is_unit_propagation_based_on_constants,
                                        'is_unit_propagation_based_on_unknown_variable':tw.is_unit_propagation_based_on_unknown_variable}}
                                        
                if self.debug and self.debug_scope:
                    print "ADDED to SCOPE %s : %s has units %s at %s" % (lhs_scope.Id, lhs_compound_name, root_token.units, line_num)
                    print 'constants-based? %s  unknown-based? %s' % (tw.is_unit_propagation_based_on_constants,  tw.is_unit_propagation_based_on_unknown_variable)
            # EXTRACT UNITS FROM RETURN STATEMENT AND APPLY TO FUNCTION
            if root_token.str == 'return' and root_token.units:
                if function_dict['scopeObject'].function:
                    function_return_units = function_dict['scopeObject'].function.return_units
                    for u in root_token.units:
                        if u not in function_return_units:
                            function_return_units.append(u)
                            function_dict['scopeObject'].function.is_unit_propagation_based_on_constants = tw.is_unit_propagation_based_on_constants
                            function_dict['scopeObject'].function.is_unit_propagation_based_on_unknown_variable = tw.is_unit_propagation_based_on_unknown_variable
            found_units_in_this_function = found_units_in_this_function or tw.found_units_in_this_tree

        # LOOK FOR EARLY ABANDONMENT
        if not found_units_in_this_function and not self.debug_print_AST and self.should_abandon_early:
            return

        # DEBUG PRINTING
        for root_token in function_dict['root_tokens']:
            if self.debug_print_AST:
                tw.debug_walk(root_token)
 
        self.found_ros_units_in_this_file = self.found_ros_units_in_this_file or found_units_in_this_function
            

    def init_cppcheck_config_data_structures(self, cppcheck_configuration):  # initialize
        c = cppcheck_configuration
        for t in c.tokenlist:
            t.units = []
            t.is_unit_propagation_based_on_constants = False
            t.is_unit_propagation_based_on_unknown_variable = False
            t.isRoot = False
        for s in c.scopes:
            s.var_ordered_dict = OrderedDict()
        for f in c.functions:
            f.return_units = []
            f.arg_units = []
            f.is_unit_propagation_based_on_constants = False
            f.is_unit_propagation_based_on_unknown_variable = False
            for arg_number in f.argument.keys():
                f.arg_units.append([])
            f.arg_units_dict = {}
            for k,v in f.argumentId.iteritems():
                f.arg_units_dict[k] = []
        return c


    def init_cppcheckdata_structures(self, dump_file_as_cppcheckdata):
        '''  AUGMENT SCOPE DATA STRUCTURE WITH ADDITIONAL FEATURES TO SUPPORT THIS ANALYSIS
            each token gets an empty list to hold unit assignments
            each scope gets an empty list to hold variable unit assignments in that scope
            '''
        if self.debug_verbose:
            print inspect.stack()[0][3]
        for c in dump_file_as_cppcheckdata.configurations:
            c = self.init_cppcheck_config_data_structures(c)
                    
        return dump_file_as_cppcheckdata


    def main_run_check(self, dump_file, source_file=''): 
        ''' PEFORM UNITS CHECKING
            todo: pep8
            input: a cppcheck 'dump' file containing an Abstract Syntax Tree (AST), symbol table, and token list.
            returns: None
            side-effects: updates datbase with information about this unit analysis
        '''
        if self.debug_verbose:
            print inspect.stack()[0][3]
        self.source_file = source_file
        self.current_file_under_analysis = dump_file
        # PARSE INPUT
        data = cppcheckdata.parsedump(dump_file)
        analysis_unit_dict = {}
        # INITIALIZE ERROR CHECKING OBJECT
        self.error_checker = ErrorChecker(self.debug, source_file, self)
        # GIVE TREE WALKER ACCESS TO SOURCE FILE FOR DEBUG PRINT
        if self.source_file and self.debug and os.path.exists(self.source_file):
            with open(self.source_file) as f:
                self.source_file_lines = f.readlines()
                print "yes"
        else:
            if self.debug:
                print "no %s %s" % (self.source_file, self.debug)
        # for c in data.configurations:  #todo: what is a data configuration?  -- Check for multiple
        for c in data.configurations[:1]:   # MODIFIED TO ONLY TEST THE FIRST CONFIGURATION
            self.current_configuration = c
        # ADD AST DECORATION PLACEHOLDERS
            c = self.init_cppcheck_config_data_structures(c)
            # REFRESH VARIABLES
            self.function_graph = nx.DiGraph()
            # GET DICT OF ALL GLOBALLY SCOPED FUNCTIONS
            analysis_unit_dict = self.find_functions(c)
            sorted_analysis_unit_dict = analysis_unit_dict;  # WILL BECOME AN ORDERED DICT IF self.should_sort_by_function_graph
            # FIND ORDER FOR FUNCTION GRAPH EXPLORATION ( topo sort, if possible, otherwise todo ??)
            if self.should_sort_by_function_graph:
                self.build_function_graph(analysis_unit_dict)  # WILL USE DAG SUBGRAPH
                sorted_analysis_unit_dict = self.make_sorted_analysis_unit_dict_from_function_graph(analysis_unit_dict) # RETURNS ORDERED DICT
                self.all_sorted_analysis_unit_dicts.append(sorted_analysis_unit_dict)
                # SPECIAL MODE FOR COUNTING / IDENTIFYING FILES WITH ROS UNITS
                if self.SHOULD_ONLY_FIND_FILES_WITH_UNITS and self.found_ros_units_in_this_file:
                    return
            ##print "G nodes:%d edges:%d" % (G.order(), G.size())
            if self.debug_print_function_topo_sort:
                self.debug_print_function_graph(sorted_analysis_unit_dict)
            # COLLECT ALL TOKEN PARSE TREES FOR EACH FUNCTION
            for function_dict in sorted_analysis_unit_dict.values():
                self.analyze_function(function_dict)
                # SAVE OFF A POINTER TO THE TREE WALKERS USED IN THIS ANALYSIS FOR TESTING
                self.all_tree_walkers.append(self.tw)
                self.debug_function_count += 1
            # if self.debug:
                # for e in self.errors:
                    # print e['error_msg']
            # self.error_checker.error_check_function_args_consistent(c)
            # for e in self.error_checker.all_errors:
                # self.errors.append(e)
            # ADD ERROR OBJECTS FOUND BY ERROR CHECKER TO MY ERRORS
            if self.debug :
                self.error_checker.pretty_print()
            # DEBUG COUNTERS
            self.debug_configuration_count += 1
            self.debug_function_count = 0
        ## - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
        ## OPTIONAL FIND UNITS ONLY AND WRITE TO DATABASE
        ## - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
        if self.SHOULD_ONLY_FIND_FILES_WITH_UNITS and self.SHOULD_FIND_ALL_UNITS:
            self.insert_file_unit_class_records()
            return
        ## - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
        ## MAIN ERROR CHECKING
        ## - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
        # for c in data.configurations: 
        for c in data.configurations[:1]:  # FIRST ONLY
            # CHECK THIS CONFIGURATION FOR ERRORS
            self.error_checker.error_check_multiple_units(c)
            self.error_checker.error_check_function_args_consistent(c)
            # self.error_checker.error_check_unit_smell(c)
            # todo: when the same error occurs in multiple data configs, where is a good place to catch that
            # todo: add checking of addition on RH
        for sorted_analysis_unit_dict in self.all_sorted_analysis_unit_dicts:
            self.error_checker.error_check_comparisons(sorted_analysis_unit_dict)
            self.error_checker.error_check_logical_operators(sorted_analysis_unit_dict)
            self.error_checker.error_check_addition_of_incompatible_units(sorted_analysis_unit_dict)
        # GET ERRORS FROM ERROR CHECKER OBJECT  (add to list of pointers)
        for e in self.error_checker.all_errors:
            self.errors.append(e)
            # COUNTS ERRORS BY TYPE
            # self.list_of_error_counts_by_type[e.ERROR_TYPE] += 1
        if self.SHOULD_WRITE_RESULTS_TO_DATABASE:
            if self.errors:
                self.update_database(self.errors)


    def build_function_graph(self, analysis_unit_dict):
        ''' BUILDS DIRECTED FUNCTION GRAPH 
            input:  a dictionary of functions from this dump file
            output: none.  Side effect creates a graph linked to this object
            '''
        if self.debug_verbose:
            print inspect.stack()[0][3]
        # BUILD CALL GRAPH
        self.function_graph = nx.DiGraph()
        G = self.function_graph
        for k, function_dict in analysis_unit_dict.iteritems():
            if function_dict['function']:  # MIGHT BE NONE WHEN FUNCTION IS CLASS CONSTRUCTOR (?)
                node = function_dict['function'].Id             # Id of the Function
                #if not G.has_node(node):
                G.add_node(node) #, function_dict_key=k})  # FUNCTION CPP OBJECT IS NODE
                #else:
                all_attr = nx.get_node_attributes(G, 'function_id')
                all_attr[node] = k
                nx.set_node_attributes(G, 'function_id', all_attr)
                #function_dict['is_visited'] = False
                self.add_edges_to_function_graph(function_dict, G, node)
        self.function_graph = G


    def make_sorted_analysis_unit_dict_from_function_graph(self, analysis_unit_dict):
        ''' BUILDS A TOPO SORTED FUNCTION GRAPH.  THIS ALLOWS THE ANALYSIS TO START ON FUNCTION LEAFS, 
            SO WE CAN HOPEFULLY DISCOVER UNITS ON THE RETURN TYPE AND PROPAGE THEM UP.  THE 
            FUNCTION GRAPH MAY HAVE CYCLES (recursion, for example), THEREFORE WE REMOVE THESE EDGES FROM THE GRAPH
            AND ANALYZE THEM LAST (<-- not sure this is best)
            input:  a dictionary of functions from this dump file
            output: OrderedDict of functions
            postcondition:   returned dict must be the same length as the input dict, and contain all the same elements
            '''
        if self.debug_verbose:
            print inspect.stack()[0][3]
        return_dict = OrderedDict()
        G = self.function_graph 
        # TRY FINDING A DAG.  IF NOT, REMOVE EDGES AND TRY AGAIN. 
        super_break = 0
        while nx.number_of_nodes(G) > 0 and super_break < 1000:
            super_break +=1 
            if not nx.is_directed_acyclic_graph(G):
                try:
                    # SEARCH FOR CYCLE AND REMOVE EDGES
                    edges = nx.find_cycle(G)
                    G.remove_edges_from(edges)
                    if self.debug and self.debug_print_function_graph:
                        print 'Function graph has cycle %s' % edges,
                except:
                    if self.debug and self.debug_print_function_graph:
                        print 'Function graph is not a DAG and does not have a cycle!'
                        # GIVE UP AND RETURN UNSORTED 
                    return analysis_unit_dict
            else:
                # WE HAVE A DIGRAPH, CAN PROCEED ( and topo sort )
                break
        
        if nx.number_of_nodes(G) == 0:
            # RETURN UNCHANGED
            return analysis_unit_dict
        # WE HAVE A DIRECTED GRAPH WITH NODES, CAN SORT AND ADD NODES TO ORDERED LIST
        function_graph_topo_sort = nx.topological_sort(G)
        function_graph_topo_sort_reversed = function_graph_topo_sort[::-1]  # REVERSED
        
        # OPTIONAL DEBUG PRINT
        if self.debug_print_function_topo_sort:
            print function_graph_topo_sort_reversed


        # CREATE RETURN DICT FROM TOPO SORT
        for node in function_graph_topo_sort_reversed:
            function_id_attr_dict = nx.get_node_attributes(G, 'function_id')
            #print function_id_attr_dict
            if node in function_id_attr_dict:  # FIRST OPTION SHOULD SHORTCUT WHEN ATTR DICT DOES NOT EXITS
                # ADD FUNCTION TO NEW DICTIONARY - THIS IS THE EXPLORE ORDER
                return_dict[function_id_attr_dict[node]] = analysis_unit_dict[function_id_attr_dict[node]]
            else:
                if self.debug and self.debug_print_function_graph:
                    print "Graph node does not have function_dict_key"
        
        # ADD ANY REMAINING FUNCTIONS NOT IN THE TOPO SORT TO THE ORDERED DICT
        for k in analysis_unit_dict.keys():
            if k not in return_dict:
                return_dict[k] = analysis_unit_dict[k]

        assert (len(return_dict) == len(analysis_unit_dict))

        return return_dict


    def debug_print_function_graph(self, analysis_unit_dict):
        if self.debug_verbose:
            print inspect.stack()[0][3]
        if not analysis_unit_dict:
            return
        for function_dict in analysis_unit_dict.values():
            print "%s :" % function_dict['name']
            for edge in function_dict['function_graph_edges']:
                print ' --> %s' % edge.name
            

    def add_edges_to_function_graph(self, function_dict, G, current_node):
        if self.debug_verbose:
            print inspect.stack()[0][3]
            #self.print 'cps_unit_checker: add_edges_to_function_graph'
        
        # GET THE FIRST TOKEN AFTER THE FUNCTION DEFINITION
        current_token = function_dict['tokenStart']
        end_token = function_dict['tokenEnd']
        while current_token is not end_token: # TERMINATION GUARENTEED IF DUMP FILE IS WELL FORMED
            current_token = current_token.next  # ON FIRST LOOP, SKIPS SELF-REFERENCE 
            if current_token.function:
                if not G.has_edge(current_node, current_token.function.Id):
                    G.add_edge(current_node, current_token.function.Id)
      


if __name__ == "__main__":
    # FOR COMMAND LINE INPUT
    cps_unit_checker = CPSUnitsChecker()
    cps_unit_checker.handle_command_line_input()




