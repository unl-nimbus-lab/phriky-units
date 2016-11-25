from symbol_helper import SymbolHelper
import copy
import inspect


class TreeWalker:

    name = None

    def __init__(self, function_dictionary):
        self.my_symbol_helper = SymbolHelper(function_dictionary)
        self.symbol_helper = self.my_symbol_helper
        self.debug = False
        self.debug_scope = False
        self.debug_verbose = False
        self.source_file = ''
        self.source_file_lines = []
        self.errors = []
        self.return_value = None
        self.return_value_list = []
        self.current_file = ''
        self.current_function_dict = None
        self.should_check_unitless_during_multiplication = False
        self.found_units_in_this_tree = False
        self.was_some_unit_changed = False
        self.cps_unit_checker = None   # DEPENDENCY INJECTION.  SEE MARTIN FOWLER
        self.is_unit_propagation_based_on_unknown_variable = False
        self.is_unit_propagation_based_on_constants = False
        self.is_assignment_statement = False
        self.current_AST_start_line_number = None  # PROTECTS FROM MULTI-LINE STATEMENTS
        self.current_AST_end_line_number = None  # PROTECTS FROM MULTI-LINE STATEMENTS
        pass

    def generic_recurse_and_apply_function(self, token, function_to_apply):
        ''' GENERIC RECURSION PATTERN - LEFT RIGHT TOKEN
            input:  token  CPPCHECK token to recurse upon
            function_to_apply:  the function to apply to the token 
                after recursing.
            returns: None   Side effect determined by function_to_apply
            '''
        if not token:
            return
        # INITIALIZE
        left_token = right_token = None

        # LEFT
        if token.astOperand1:
            left_token = token.astOperand1
            self.generic_recurse_and_apply_function(left_token,
                                                    function_to_apply)
        # RIGHT
        if token.astOperand2:
            right_token = token.astOperand2
            self.generic_recurse_and_apply_function(right_token,
                                                    function_to_apply)

        function_to_apply(token, left_token, right_token)

    def find_assignment_tokens_recursive_target(self,
                                                token,
                                                left_token,
                                                right_token):
        ''' FIND IF THIS AST IS AN ASSIGNMENT STATEMENT
            input:  token AN AST TOKEN
                    left_token  (ignored)
                    right_token  (ignored)
            returns: None  (side effect: modifies class is_assignment_statement
            '''
        if self.debug_verbose:
            self.cps_unit_checker.handle_debug_verbose(inspect.stack()[0][3])
        if token.isAssignmentOp:
            self.is_assignment_statement = True

    def find_min_max_line_numbers(self, token, left_token, right_token):
        ''' FIND THE MIN AND MAX LINE NUMBERS FOR THIS AST,
                PROTECT FROM MULTI-LINE STATEMENTS
            input:  token AN AST TOKEN
                    left_token  (ignored)
                    right_token  (ignored)
            returns: None  (side effect: modifies class min and max
                line number range
            '''
        if self.debug_verbose:
            self.cps_unit_checker.handle_debug_verbose(inspect.stack()[0][3])
        if not self.current_AST_start_line_number or \
                token.linenr < self.current_AST_start_line_number:
            self.current_AST_start_line_number = token.linenr
        if not self.current_AST_end_line_number or \
                token.linenr > self.current_AST_end_line_number:
            self.current_AST_end_line_number = token.linenr

    def reset_min_max_line_numbers(self):
        ''' INITIALIZES THE AST LINE NUMBERS BACK TO NONE.
                SHOULD BE CALLED BEFORE EVERY AST IS EVALUATED
            input: None
            output: None.  side effect is setting class variables
                for min max to None
            '''
        self.current_AST_start_line_number = None
        self.current_AST_end_line_number = None

    def apply_ROS_units(self, token, left_token, right_token):
        '''  DECORATE LEAF WITH ROS MESSAGE TYPE UNITS
            input:  token AN AST TOKEN
                    left_token  (ignored)
                    right_token  (ignored)
            returns: None  (side effect: adds units to ROS variable leafs)
        '''
        if self.debug_verbose:
            self.cps_unit_checker.handle_debug_verbose(inspect.stack()[0][3])
        if token.variable:
            # LOOKUP VARIABLE TYPE IN SYMBOL DICTIONARY FOR ROS MESSAGES
            units_as_dict = self.my_symbol_helper.find_units_for_variable(token)
            # ADD THIS DICTIONARY
            if units_as_dict and (units_as_dict not in token.units):
                self.was_some_unit_changed = True
                # CHECK SYMBOL HELPER FOR WEAK INFERENCE IN THE CASE OF
                # JOINT_STATES AND getXYZ()
                if self.my_symbol_helper.is_weak_inference:
                    self.is_unit_propagation_based_on_unknown_variable = True
                    token.is_unit_propagation_based_on_unknown_variable = True
                if self.debug_verbose:
                    s = "tw. units changed in %s" % inspect.stack()[0][3]
                    print s
                token.units.append(units_as_dict)
                self.found_units_in_this_tree = True

    def apply_units_from_scope(self, token, left_token, right_token):
        if self.debug_verbose:
            self.cps_unit_checker.handle_debug_verbose(inspect.stack()[0][3])
        if token.variable:
            compound_var_name = self.my_symbol_helper.\
                    find_compound_variable_name_for_variable_token(token)
            # PATH TO THE SCOPE WHERE THIS VAR DEFINED
            var_scope = token.variable.nameToken.scope
            if token.variable.isArgument:
                if self.current_function_dict:
                    var_scope = self.current_function_dict['scopeObject']
            if self.debug and self.debug_scope:
                s = "%s attempting to apply previous units for %s "
                s += "to scope dict %s"
                s = s % (token.linenr,
                         compound_var_name,
                         var_scope.var_ordered_dict)
                pass
            if compound_var_name in var_scope.var_ordered_dict:
                # GET MOST RECENT ASSIGNMENT IN SCOPE
                #    SORT BY LINE NUMBER - WE WANT LAST (MOST RECENT)
                #    ASSIGNMENT todo: flow sensitive?
                # IF len(dict)==1, ASSUME SORT IS FAST
                s = sorted(var_scope.var_ordered_dict[compound_var_name])
                last_line_number = s[-1]
                # REJECT UNITS FOUND IN SCOPE IF THEY CAME FROM THIS AST
                # todo: potential BUG when previous assignment was from a different file but the same line number
                if last_line_number in range(int(self.current_AST_start_line_number), int(self.current_AST_end_line_number)+1):
                    return

                most_recent_units = var_scope.var_ordered_dict[compound_var_name][last_line_number]['units']
                for u in most_recent_units:
                    if u in token.units:
                        continue
                    token.units.append(u)
                    self.was_some_unit_changed = True
                    self.found_units_in_this_tree = True
                    # IF PREVIOUS ASSIGNMENT WAS BASED ON CONSTANTS OR UNKNOWN VARIABLES, PAY ATTENTION TO IT HERE
                    if var_scope.var_ordered_dict[compound_var_name][last_line_number]['is_unit_propagation_based_on_constants']:
                        self.is_unit_propagation_based_on_constants = True
                        token.is_unit_propagation_based_on_constants = True
                    if var_scope.var_ordered_dict[compound_var_name][last_line_number]['is_unit_propagation_based_on_unknown_variable']:
                        self.is_unit_propagation_based_on_unknown_variable = True
                        token.is_unit_propagation_based_on_unknown_variable = True
                    if self.debug_verbose:
                        s = "tw. units changed in %s" \
                                % inspect.stack()[0][3]
                        print s % inspect.stack()[0][3]
                        s = "len most recent %d"
                        print s % len(most_recent_units)
                        s =  "len most recent %d"
                        print s % len(most_recent_units)
                        s = "FOUND UNITS for %s on line %s from line "
                        s += "%s. %s"
                        print s % (compound_var_name,
                                   token.linenr,
                                   last_line_number,
                                   most_recent_units)
                        print "FOUND UNITS for %s on line %s from line %s. %s" % (compound_var_name, token.linenr, last_line_number, most_recent_units)
                    if self.debug and self.debug_scope:
                        print "FOUND UNITS for %s on line %s from line %s. %s" % (compound_var_name, token.linenr, last_line_number, most_recent_units)

                else:
                    pass # DO NOTHING BECAUSE CURRENT == MOST RECENT UNITS
        if token.function:
            function_return_units = token.function.return_units
            for u in function_return_units:
                if u not in token.units:
                    self.was_some_unit_changed = True
                    self.found_units_in_this_tree = True
                    if self.debug_verbose:
                        s = "tw. units changed in %s" % inspect.stack()[0][3]
                        print s
                    token.units.append(u)
                    self.is_unit_propagation_based_on_constants = \
                            token.function.\
                            is_unit_propagation_based_on_constants
                    token.is_unit_propagation_based_on_constants = \
                            token.function.\
                            is_unit_propagation_based_on_constants
                    self.is_unit_propagation_based_on_unknown_variable = \
                            token.function.\
                            is_unit_propagation_based_on_unknown_variable
                    token.is_unit_propagation_based_on_unknown_variable = \
                            token.function.\
                            is_unit_propagation_based_on_unknown_variable

    def apply_units_known_symbols(self, token, left_token, right_token):
        ''' INCLUDES M_PI
            '''
        if self.debug_verbose:
            self.cps_unit_checker.handle_debug_verbose(inspect.stack()[0][3])
        if token.str == 'M_PI':
            if token.str in self.my_symbol_helper.ros_unit_dictionary:
                a_dict = self.my_symbol_helper.ros_unit_dictionary\
                        [token.str][token.str]
                if a_dict not in token.units:
                    self.was_some_unit_changed = True
                    self.found_units_in_this_tree = True
                    if self.debug_verbose:
                        s = "tw. units changed in %s" % inspect.stack()[0][3]
                        print s
                    if self.cps_unit_checker:
                        if self.cps_unit_checker.SHOULD_FIND_ALL_UNITS:
                            self.cps_unit_checker.\
                                    add_class_and_units_to_all_list\
                                    (token.str, a_dict)
                    token.units.append(a_dict)

    def apply_units_inverse_trig(self, token, left_token, right_token):
        if self.debug_verbose:
            self.cps_unit_checker.handle_debug_verbose(inspect.stack()[0][3])
        if token.str in ['atan2', 'acos', 'asin', 'atan']:
            if token.str in self.my_symbol_helper.ros_unit_dictionary:
                a_dict = self.my_symbol_helper.ros_unit_dictionary[
                         token.str][token.str]
                if a_dict not in token.units:
                    self.was_some_unit_changed = True
                    self.found_units_in_this_tree = True
                    if self.debug_verbose:
                        s = "tw. units changed in %s" % inspect.stack()[0][3]
                        print s
                    if self.cps_unit_checker:
                        if self.cps_unit_checker.SHOULD_FIND_ALL_UNITS:
                            self.cps_unit_checker.\
                                    add_class_and_units_to_all_list\
                                    (token.str, a_dict)
                    token.units.append(a_dict)

    def apply_units_toSec(self, token, left_token, right_token):
        if self.debug_verbose:
            self.cps_unit_checker.handle_debug_verbose(inspect.stack()[0][3])
        if token.str == 'toSec':
            if token.str in self.my_symbol_helper.ros_unit_dictionary:
                toSec_units_dict = self.\
                        my_symbol_helper.ros_unit_dictionary[
                         token.str][token.str]
                if  toSec_units_dict not in token.units:
                    self.was_some_unit_changed = True
                    self.found_units_in_this_tree = True
                    if self.debug_verbose:
                        s = "tw. units changed in %s" % inspect.stack()[0][3]
                        print s
                    if self.cps_unit_checker:
                        if self.cps_unit_checker.SHOULD_FIND_ALL_UNITS:
                            self.cps_unit_checker.\
                                    add_class_and_units_to_all_listi\
                                    (token.str, toSec_units_dict)
                    token.units.append(toSec_units_dict)

    def apply_units_getXYZ(self, token, left_token, right_token):
        if self.debug_verbose:
            self.cps_unit_checker.handle_debug_verbose(inspect.stack()[0][3])
        if token.str not in ['getX', 'getY', 'getZ']:
            return
        # DECLARE VARIABLE HERE FOR SCOPING
        units_for_getXYZ = []
        # TRY TO FIGURE OUT TYPE THIS WAS CALLED ON
        # ASSUME PARENT TOKEN IS '.'
        # OPTION 1 - LOOK FOR getOrigin and getRotation
        if token.astParent.astOperand1 and \
                token.astParent.astOperand1.str == '(':
            paren_token = token.astParent.astOperand1  # CONVENIENCE
            if paren_token.astOperand1 and paren_token.astOperand1.str == '.':
                dot_token = paren_token.astOperand1

                if dot_token.astOperand1 and dot_token.astOperand1.variable:
                    class_name = self.my_symbol_helper.\
                            find_variable_type(dot_token.astOperand1.variable)
                    class_name_sanitized = self.my_symbol_helper.\
                            sanitize_class_name(class_name)
                    if class_name_sanitized in self.my_symbol_helper.\
                            ros_unit_dictionary:
                        if dot_token.astOperand2 and \
                                dot_token.astOperand2.str == 'getOrigin':
                            units_for_getXYZ = self.my_symbol_helper.\
                                    ros_unit_dictionary[
                                     class_name_sanitized]['getOrigin']
                        elif dot_token.astOperand2 and \
                                dot_token.astOperand2.str == 'getRotation':
                            units_for_getXYZ = self.\
                                    my_symbol_helper.ros_unit_dictionary[
                                     class_name_sanitized]['getRotation']
                else:
                    s = 'Failed token: %s %s' 
                    print s % (dot_token.astOperand1.linenr, 
                               dot_token.astOperand1.str)
        elif token.astParent.astOperand1:
            # GET TYPE SO WE CAN INFER CORRECT UNITS BASED
            # ON KNOWLEDGE OF ROS UNIT ASSUMPTIONS
            if token.astParent.astOperand1.variable:
                class_name = self.my_symbol_helper.\
                        find_variable_type(token.astParent.
                        astOperand1.variable)
                class_name_sanitized = self.my_symbol_helper.\
                        sanitize_class_name(class_name)
                # FIND APPROPRIATE UNITS
                if class_name_sanitized in \
                        self.my_symbol_helper.ros_unit_dictionary:
                    units_for_getXYZ = self.my_symbol_helper.\
                            ros_unit_dictionary[
                             class_name_sanitized][token.str]
            else:
                pass

        else:
            # NO AST PARENT OPERAND1 - SHOULD BE IMPOSSIBLE
            # assert False
            pass

        if units_for_getXYZ and (units_for_getXYZ not in token.units):
            self.was_some_unit_changed = True
            self.found_units_in_this_tree = True
            if self.debug_verbose:
                s = "tw. units changed in %s" % inspect.stack()[0][3]
                print s
            if self.cps_unit_checker:
                if self.cps_unit_checker.SHOULD_FIND_ALL_UNITS:
                    self.cps_unit_checker.\
                            add_class_and_units_to_all_list(
                            token.str,
                            units_for_getXYZ)
            token.units.append(units_for_getXYZ)

    def apply_units_quatToRPY(self, token, left_token, right_token):
        if self.debug_verbose:
            self.cps_unit_checker.handle_debug_verbose(inspect.stack()[0][3])
        if token.str not in ['quatToRPY']:
            return
        if token.str not in self.my_symbol_helper.ros_unit_dictionary:
            return
        rpy_tokens_list = []
        if token.astParent.astOperand2.str == ',':
            try:
                # THIS MAKES A LOT OF ASSUMPTIONS ABOUT THE AST STRUCTURE
                roll_token = token.astParent.astOperand2.\
                        astOperand1.astOperand1.astOperand2
                pitch_token = token.astParent.astOperand2.\
                        astOperand1.astOperand2
                yaw_token = token.astParent.astOperand2.astOperand2
                # ADD THESE TOKENS TO A LIST
                rpy_tokens_list = [roll_token, pitch_token, yaw_token]
            except:
                if self.debug:
                    print "quatToRPY 1st Case exception"
        elif token.astParent.str == "::":
            try:
                roll_token = token.astParent.astParent.astOperand2.\
                        astOperand1.astOperand1.astOperand2
                pitch_token = token.astParent.astParent.astOperand2.\
                        astOperand1.astOperand2
                yaw_token = token.astParent.astParent.astOperand2.astOperand2
                # ADD THESE TOKENS TO A LIST
                rpy_tokens_list = [roll_token, pitch_token, yaw_token]
            except:
                if self.debug:
                    print "quatToRPY 2nd Case exception"
        if not rpy_tokens_list:
            if self.debug:
                print "quatToRPY no rpy_tokens_list tokens"
            return
        # GET UNITS FROM SYMBOL HELPER
        quatToRPY_units_dict = self.my_symbol_helper.\
                ros_unit_dictionary[token.str][token.str]
        for rpy_token in rpy_tokens_list:
            rpy_compound_name = self.recurse_and_collect_string(rpy_token)
            # ADD UNITS TO VARIABLE IN SCOPE
            self.return_value_list = []
            self.generic_recurse_and_apply_function(rpy_token,
                                                    self.find_first_variable)
            if len(self.return_value_list) != 1:
                if self.debug:
                    s = "quatToRPY exception: multiple variables "
                    s += "returned in find_first_variable"
                    print s
                return  # todo: track count of this error
            rpy_var = self.return_value_list[0]
            rpy_scope = rpy_var.nameToken.scope
            linenr = int(rpy_token.linenr)
            if rpy_compound_name in rpy_scope.var_ordered_dict:
                # ALREADY HAS ASSIGNMENT, ADD TO IT
                rpy_scope.var_ordered_dict[rpy_compound_name][linenr]\
                        = {'units': [quatToRPY_units_dict],
                           'token': token,  # THIS IS A LIST
                           'is_unit_propagation_based_on_constants': False,
                           'is_unit_propagation_based_on_unknown_variable':
                                   False}
            else:
                rpy_scope.var_ordered_dict[rpy_compound_name]\
                        = {linenr:
                           {'units': [quatToRPY_units_dict],
                           'token': token,
                           'is_unit_propagation_based_on_constants': False,
                           'is_unit_propagation_based_on_unknown_variable':
                                   False}}

    def propagate_units_across_dot_connectors(self,
                                              token,
                                              left_token,
                                              right_token):
        if self.debug_verbose:
            self.cps_unit_checker.handle_debug_verbose(inspect.stack()[0][3])
        self.propagate_units_across_connectors(token,
                                               left_token,
                                               right_token,
                                               '.')
        # DOT CONNECTOR AFTER PAREN WiTH NOTHING BEFORE
        if token and token.str == '.' and \
                token.astParent and \
                token.astParent.str == '(' and not \
                token.astParent.astOperand2:
            self.propagate_units_across_connectors(token.astParent,
                                                   token,
                                                   None,
                                                   '(')

    def propagate_units_across_parenthesis(self,
                                           token,
                                           left_token,
                                           right_token):
        if self.debug_verbose:
            self.cps_unit_checker.handle_debug_verbose(inspect.stack()[0][3])
        if token.str == '(':
            if left_token.function:
                # IF LEFT SIDE IS A FUNCTION, PROPATE ITS RETURN UNITS
                for u in left_token.units:
                    if u not in token.units:
                        self.was_some_unit_changed = True
                        if self.debug_verbose:
                            s = "tw. units changed in %s"
                            s = s % inspect.stack()[0][3]
                            print s
                        token.units.append(u)
            elif left_token.isName:
                return
            elif not right_token:
                self.propagate_units_across_connectors(token,
                                                       left_token,
                                                       right_token,
                                                       '(')

            else:
                pass

    def propagate_units_across_square_brackets(self,
                                               token,
                                               left_token,
                                               right_token):
        if self.debug_verbose:
            self.cps_unit_checker.handle_debug_verbose(inspect.stack()[0][3])
        self.propagate_units_across_connectors(token,
                                               left_token,
                                               right_token,
                                               '[')

    def propagate_units_across_assignment(self,
                                          token,
                                          left_token,
                                          right_token):
        if self.debug_verbose:
            self.cps_unit_checker.handle_debug_verbose(inspect.stack()[0][3])
        # PREVENT PROPAGATION ACROSS MULTIPLE INITIALIZATIONS, LIKE
        # x = y = z = 0
        if token.str == "=":
            if left_token and left_token.str == "=":
                return
            if right_token and right_token.str == "=":
                return
        self.propagate_units_across_connectors(token,
                                               left_token,
                                               right_token,
                                               '=')

    def propagate_units_across_return(self, token, left_token, right_token):
        if self.debug_verbose:
            self.cps_unit_checker.handle_debug_verbose(inspect.stack()[0][3])
        self.propagate_units_across_connectors(token,
                                               left_token,
                                               right_token,
                                               'return')

    def propagate_units_into_function_args(self,
                                           token,
                                           left_token,
                                           right_token):
        if self.debug_verbose:
            self.cps_unit_checker.handle_debug_verbose(inspect.stack()[0][3])

    def propagate_units_sqrt(self, token, left_token, right_token):
        if self.debug_verbose:
            self.cps_unit_checker.handle_debug_verbose(inspect.stack()[0][3])
        if token.str == 'sqrt':
            unit_receiver = None
            # CASE: NAMESPACE
            if token.astParent.str == '::':
                if token.astParent.astParent:
                    unit_receiver = token.astParent.astParent
            # CASE: NAMESPACE STD
            elif token.astParent.str == '(':
                if token.astParent.astOperand2:
                    unit_receiver = token.astParent

            if not unit_receiver:
                # todo:
                return

            left_units = right_units = None
            if unit_receiver.astOperand1:
                left_units = unit_receiver.astOperand1.units
            if unit_receiver.astOperand2:
                right_units = unit_receiver.astOperand2.units
            # GET UNITS FROM INSIDE PARENS
            new_units = self.merge_units_by_set_union(left_units,
                                                      right_units)

            # DIVIDE UNITS BY TWO
            for u in new_units:
                for k, v in u.iteritems():
                    u[k] = v / 2.

            # ATTEMPT TO PROPATE UNITS ACROSS '('
            for u in new_units:
                if u not in unit_receiver.units:
                    # PROPAGATE CHILD TOKEN STATUS TO TOKEN
                    self.apply_propagation_status_to_token(token,
                                                           left_token,
                                                           right_token)
                    self.was_some_unit_changed = True
                    if self.debug_verbose:
                        s = "tw. units changed in %s" % inspect.stack()[0][3]
                        print s
                    unit_receiver.units.append(u)

    def propagate_units_inverse_trig(self, token, left_token, right_token):
        if self.debug_verbose:
            self.cps_unit_checker.handle_debug_verbose(inspect.stack()[0][3])
        if token.str in ['atan2', 'acos', 'asin', 'atan']:
            unit_receiver = None
            # CASE: NAMESPACE
            if token.astParent.str == '::':
                # ADD UNITS TO '::"
                unit_receiver = token.astParent.astParent
            # CASE: NAMESPACE STD
            elif token.astParent.str == '(':
                unit_receiver = token.astParent
            if not unit_receiver:
                s = 'unexpected ast value when processing inverse trig '
                s += 'linenr=%d lines=%d file=%s source_file=%s'
                s = s % (int(token.linenr),
                         len(self.source_file_lines),
                         token.file,
                         self.source_file)
                print s
                return
            for u in token.units:
                if u not in unit_receiver.units:
                    unit_receiver.units.append(u)
                    # PROPAGATE CHILD TOKEN STATUS TO TOKEN
                    self.apply_propagation_status_to_token(token,
                                                           left_token,
                                                           right_token)
                    self.was_some_unit_changed = True
                    if self.debug_verbose:
                        s = "tw. units changed in %s" % inspect.stack()[0][3]
                        print s

    def propagate_units_getXYZ(self, token, left_token, right_token):
        if self.debug_verbose:
            self.cps_unit_checker.handle_debug_verbose(inspect.stack()[0][3])
        if token.str in ['getX', 'getY', 'getZ']:
            unit_receiver = None
            if token.astParent.str == '(':
                unit_receiver = token.astParent
            if not unit_receiver:
                if self.debug:
                    s = 'unexpected ast value when processing getXYX linenr=%d'
                    s += ' lines=%d file=%s source_file=%s'
                    s = s % (int(token.linenr),
                             len(self.source_file_lines),
                             token.file,
                             self.source_file)
                    print s
                return
            for u in token.units:
                if u not in unit_receiver.units:
                    unit_receiver.units.append(u)
                    self.was_some_unit_changed = True
                    # THIS IS TRUE BECAUSE getX is a weaker inference
                    # - usually meters but might be m/s
                    self.is_unit_propagation_based_on_unknown_variable = True
                    token.is_unit_propagation_based_on_unknown_variable = True
                    if self.debug_verbose:
                        s = "tw. units changed in %s" % inspect.stack()[0][3]
                        print s

    def propagate_units_ternary(self, token, left_token, right_token):
        ''' TERNARY OPERATOR    x = 1 > 0 ? true_part : false_part
            '''
        if self.debug_verbose:
            self.cps_unit_checker.handle_debug_verbose(inspect.stack()[0][3])
        if token.str == '?':
            if not right_token or not right_token.str == ':':
                return
            self.propagate_units_across_connectors(right_token,
                                                   right_token.astOperand1,
                                                   right_token.astOperand2,
                                                   ':')

            if right_token.units:
                for u in right_token.units:
                    if u not in token.units:
                        # PROPAGATE CHILD TOKEN STATUS TO TOKEN
                        self.apply_propagation_status_to_token(token,
                                                               left_token,
                                                               right_token)
                        self.was_some_unit_changed = True
                        if self.debug_verbose:
                            s = "tw. units changed in %s"
                            s = s % inspect.stack()[0][3]
                            print s
                        token.units.append(u)

    def propagate_units_pow(self, token, left_token, right_token):
        if self.debug_verbose:
            self.cps_unit_checker.handle_debug_verbose(inspect.stack()[0][3])
        if token.str == 'pow':
            comma_token = None
            unit_receiver = None
            if token.astParent.str == '::':
                if token.astParent.astParent:
                    unit_receiver = token.astParent.astParent
                    comma_token = None
                    if unit_receiver.astOperand2 and \
                            unit_receiver.astOperand2.str == ',':
                        comma_token = unit_receiver.astOperand2
            # CASE: NAMESPACE STD
            elif token.astParent.str == '(':
                if token.astParent.astOperand2:
                    comma_token = token.astParent.astOperand2
                    unit_receiver = token.astParent
            if not comma_token or not unit_receiver:
                s = 'unexpected ast value when processing pow linenr=%d '
                s += 'lines=%d file=%s source_file=%s'
                s = s % (int(token.linenr),
                         len(self.source_file_lines),
                         token.file,
                         self.source_file)
                return
            if comma_token.astOperand2 and comma_token.astOperand2.isNumber:
                s = comma_token.astOperand2.str.replace('f', '')
                power_exponent = float(s)
                if comma_token.astOperand1.units:
                    # APPLY POWER TO UNITS
                    new_units = []
                    # FIRST, DEEPCOPY
                    new_units = copy.deepcopy(comma_token.astOperand1.units)
                    # NEXT, APPLY EXPONENET
                    for unit_dict in new_units:
                        for k, v in unit_dict.iteritems():
                            unit_dict[k] = power_exponent * v
                    for u in new_units:
                        if u not in unit_receiver.units:
                            # PROPAGATE CHILD TOKEN STATUS TO TOKEN
                            self.apply_propagation_status_to_token(token,
                                                                   left_token,
                                                                   right_token)
                            self.was_some_unit_changed = True
                            if self.debug_verbose:
                                s = "tw. units changed in %s"
                                s = s % inspect.stack()[0][3]
                                print s
                            unit_receiver.units.append(u)

    def propagate_units_math_min_max(self, token, left_token, right_token):
        if self.debug_verbose:
            self.cps_unit_checker.handle_debug_verbose(inspect.stack()[0][3])
        if token.str in ['min', 'max']:
            unit_receiver = None
            comma_token = None
            # CASE: NAMESPACE
            if token.astParent.str == '::':
                if token.astParent.astParent:
                    unit_receiver = token.astParent.astParent
                    if unit_receiver.astOperand2 and \
                            unit_receiver.astOperand2.str == ',':
                        comma_token = unit_receiver.astOperand2
            # CASE: NAMESPACE STD
            elif token.astParent.str == '(':
                if token.astParent.astOperand2:
                    if token.astParent.astOperand2.str == ',':
                        comma_token = token.astParent.astOperand2
                    unit_receiver = token.astParent
            if not unit_receiver or not comma_token:
                s = 'unexpected ast value when processing abs/fabs '
                s = 'linenr=%d lines=%d file=%s source_file=%s'
                s = s % (int(token.linenr),
                         len(self.source_file_lines),
                         token.file,
                         self.source_file)
                return
            left = right = None
            if comma_token.astOperand1:
                left = comma_token.astOperand1
            if comma_token.astOperand2:
                right = comma_token.astOperand2
            # SIDE EFFECT OF NEXT LINE SHOULD ADD UNITS TO '('
            self.propagate_units_across_connectors(comma_token,
                                                   left,
                                                   right,
                                                   ',')
            # ATTEMPT TO PROPATE UNITS ACROSS '('
            for u in comma_token.units:
                if u not in unit_receiver.units:
                    # PROPAGATE CHILD TOKEN STATUS TO TOKEN
                    self.apply_propagation_status_to_token(token,
                                                           left_token,
                                                           right_token)
                    self.was_some_unit_changed = True
                    if self.debug_verbose:
                        s = "tw. units changed in %s" % inspect.stack()[0][3]
                        print s
                    unit_receiver.units.append(u)

    def propagate_units_math_abs_fabs_floor_ceil(self,
                                                 token,
                                                 left_token,
                                                 right_token):
        if self.debug_verbose:
            self.cps_unit_checker.handle_debug_verbose(inspect.stack()[0][3])
        if token.str in ['abs', 'fabs', 'floor', 'ceil']:
            paren_token = None
            unit_receiver = None
            # CASE: NAMESPACE
            if token.astParent.str == '::':
                if token.astParent.astParent:
                    unit_receiver = token.astParent.astParent
                    paren_token = None
                    if unit_receiver.astOperand2 and \
                            unit_receiver.astOperand2.str == '(':
                        paren_token = unit_receiver.astOperand2
            # CASE: NAMESPACE STD
            elif token.astParent.str == '(':
                if token.astParent.astOperand2:
                    paren_token = token.astParent.astOperand2
                    unit_receiver = token.astParent

            if not paren_token or not unit_receiver:
                s = 'unexpected ast value when processing abs/fabs '
                s += 'linenr=%d lines=%d file=%s source_file=%s'
                s = s % (int(token.linenr),
                         len(self.source_file_lines),
                         token.file,
                         self.source_file)
                return

            left = right = None
            if paren_token.astOperand1:
                left = paren_token.astOperand1
            if paren_token.astOperand2:
                right = paren_token.astOperand2
            # SIDE EFFECT OF NEXT LINE SHOULD ADD UNITS TO '('
            self.propagate_units_across_connectors(paren_token,
                                                   left,
                                                   right,
                                                   '(')
            # ATTEMPT TO PROPATE UNITS ACROSS '('
            for u in paren_token.units:
                if u not in unit_receiver.units:
                    # PROPAGATE CHILD TOKEN STATUS TO TOKEN
                    self.apply_propagation_status_to_token(token,
                                                           left_token,
                                                           right_token)
                    self.was_some_unit_changed = True
                    if self.debug_verbose:
                        s = "tw. units changed in %s" % inspect.stack()[0][3]
                        print s
                    unit_receiver.units.append(u)

    def propagate_units_across_connectors(self,
                                          token,
                                          left_token,
                                          right_token,
                                          connector):
        if self.debug_verbose:
            self.cps_unit_checker.handle_debug_verbose(inspect.stack()[0][3])
        '''  DECORATE TREE WITH ROS MESSAGE TYPE UNITS
            input:  token  AN AST TOKEN
                    left_token  left child of token, or None
                    right_token  left child of token, or None
            returns: None  side effect on token is to modify units
        '''
        left_units = right_units = []           # INIT

        if left_token:
            left_units = left_token.units
        if right_token:
            right_units = right_token.units

        # APPLY SET UNION
        if token.str == connector:
            new_units = self.merge_units_by_set_union(left_units,
                                                      right_units)
            # UNIFY TOKENS FROM CHILDREN WITH CURRENT TOKENS
            if new_units != token.units:
                token.units = new_units
                self.was_some_unit_changed = True
                # PROPAGATE CHILD TOKEN STATUS TO TOKEN
                self.apply_propagation_status_to_token(token,
                                                       left_token,
                                                       right_token)
                if self.debug_verbose:
                    s = "tw. units changed in %s" % inspect.stack()[0][3]
                    print s

    def error_check_multiple_units(self, token, left_token, right_token):
        if self.debug_verbose:
            self.cps_unit_checker.handle_debug_verbose(inspect.stack()[0][3])
        if len(token.units) > 1:
            # MULTIPLE HYPOTHESIS
            if token.str == '=':
                error_string = "%s : %s  MULTIPLE UNITS BY ASSIGNMENT: %s"
                error_string = error_string % (token.file,
                                               token.linenr,
                                               token.units)
            else:
                error_string = "%s : %s  %s has MULTIPLE UNITS: %s"
                error_string = error_string % (token.file,
                                               token.linenr,
                                               token.str,
                                               token.units)
            self.handle_error(token, error_string)

    def error_check_logical_operators(self, token, left_token, right_token):
        if self.debug_verbose:
            self.cps_unit_checker.handle_debug_verbose(inspect.stack()[0][3])
        # NOT << and >> since theses are used in PRINT OUTS
        if token.isOp and token.str in ['&&', '||', '!']:
            error_units = ''
            if left_token and left_token.units:
                error_units += str(left_token.units)
            if right_token and right_token.units:
                error_units += str(right_token.units)
            if error_units:
                error_string = "%s : %s LOGICAL OP %s used : %s"
                error_string = error_string % (token.file,
                                               token.linenr,
                                               token.str,
                                               error_units)
                self.handle_error(token, error_string)

    def error_check_bitwise_operators(self, token, left_token, right_token):
        if self.debug_verbose:
            self.cps_unit_checker.handle_debug_verbose(inspect.stack()[0][3])
        # NOT << and >> since theses are used in PRINT OUTS
        if token.isOp and token.str in ['&', '|', '^', '~', '~=', '&=', '|=']:
            if left_token or right_token:
                error_units = ''
                if left_token and left_token.units:
                    error_units += str(left_token.units)
                if right_token and right_token.units:
                    error_units += str(right_token.units)
                error_string = "%s : %s BITWISE OP %s used : %s"
                error_string = error_string % (token.file,
                                               token.linenr,
                                               token.str,
                                               error_units)
                self.handle_error(token, error_string)

    def error_check_modulo(self, token, left_token, right_token):
        if self.debug_verbose:
            self.cps_unit_checker.handle_debug_verbose(inspect.stack()[0][3])
        # NOT << and >> since theses are used in PRINT OUTS
        if token.isOp and token.str in ['%', '%=']:
            if left_token or right_token:
                error_units = ''
                if left_token and left_token.units:
                    error_units += str(left_token.units)
                if right_token and right_token.units:
                    error_units += str(right_token.units)
                error_string = "%s : %s MODULO OP %s used : %s"
                error_string = error_string % (token.file,
                                               token.linenr,
                                               token.str,
                                               error_units)
                self.handle_error(token, error_string)

    def error_check_unitless_operators(self, token, left_token, right_token):
        ''' LOOK FOR STRONG UNITLESS TOKENS
            input:  token   cppcheck token object
                    left_token, right_token
            returns: nothing, with possible side effect of adding errors
            '''
        if self.debug_verbose:
            self.cps_unit_checker.handle_debug_verbose(inspect.stack()[0][3])
        if token.units:
            for token_dict in token.units:
                for value in token_dict.values():
                    # MUST HAVE BECOME UNITLESS THROUGH AN OPERATIONS
                    if value == 0:
                        error_units = str(token.units)
                        error_string = "%s : %s TOKEN IS UNITLESS %s"
                        error_string = error_string % (token.file,
                                                       token.linenr,
                                                       error_units)
                        self.handle_error(token, error_string)

    def error_check_comparison_operators(self, token, left_token, right_token):
        ''' COMPARISON OPERATORS - MUST BE THE SAME ON BOTH SIDES
            input:  token   cppcheck token object
                    left_token, right_token
            returns: nothing, with possible side effect of adding errors
            '''
        if self.debug_verbose:
            self.cps_unit_checker.handle_debug_verbose(inspect.stack()[0][3])
        if token.isComparisonOp:
            if left_token.units and right_token.units:  # BOTH HAVE UNITS
                if left_token.units != right_token.units:
                    error_units = str(left_token.units)
                    error_units += str(right_token.units)
                    error_string = "%s : %s COMPARISON AGAINST "
                    error_string += "DIFFERENT UNITS %s"
                    error_string = error_string % (token.file,
                                                   token.linenr,
                                                   error_units)
                    self.handle_error(token, error_string)

    def handle_error(self, token, error_string):
        if self.debug_verbose:
            self.cps_unit_checker.handle_debug_verbose(inspect.stack()[0][3])
        line = ''
        if self.source_file_lines:
            if str(token.file).endswith('cpp'):
                if int(token.linenr) > len(self.source_file_lines):
                    s = "token line number greater than size of source file?"
                    s += "linenr=%d lines=%d file=%s error=%s, source_file=%s"
                    s = s % (int(token.linenr),
                             len(self.source_file_lines),
                             token.file,
                             error_string,
                             self.source_file)
                    raise ValueError(s)
                line = self.source_file_lines[int(token.linenr)-1]
        error_dict = {'token': token,
                      'error_msg': error_string,
                      'line': line,
                      'file': self.current_file}
        self.errors.append(error_dict)

    def merge_units_by_set_union(self, left_units, right_units):
        ''' input: {left, right}_units - lists of unit dictionaries.
            result: set union of inputs
            '''
        if self.debug_verbose:
            self.cps_unit_checker.handle_debug_verbose(inspect.stack()[0][3])
        new_units = []                      # INIT
        if left_units and right_units:
            # UNITS ARE SAME - PASS THEM UP
            if left_units == right_units:
                # COPY EITHER ONE BECAUSE SAME
                new_units = copy.deepcopy(right_units)
            # BOTH RIGHT AND LEFT SIDE OF AST HAS UNITS -- GET THE 'SET'
            new_units = copy.deepcopy(left_units)
            for r in right_units:
                if r not in new_units:
                    new_units.append(copy.deepcopy(r))
        # ONLY HAVE UNITS FROM ONE SIDE OF AST TREE
        else:
            if left_units:
                new_units = copy.deepcopy(left_units)
            elif right_units:
                new_units = copy.deepcopy(right_units)
        return new_units

    def propagate_units_across_operators(self, token, left_token, right_token):
        '''  PROPAGATE UNITS ACROSS OPERATORS
            input:  AN AST TOKEN
            returns: None
        '''
        if self.debug_verbose:
            self.cps_unit_checker.handle_debug_verbose(inspect.stack()[0][3])
        if token.str not in ['+',
                             '-',
                             '+=',
                             '-=',
                             '*',
                             '/',
                             '*=',
                             '/=',
                             '*',
                             '*=']:
            return
        left_units = right_units = []  # INIT
        # LEFT BRANCH
        if left_token:
            left_units = left_token.units
        # RIGHT BRANCH
        if right_token:
            right_units = right_token.units
        # PROPAGATE EVEN IF WE DON'T KNOW -- ASSUME SCALAR
        if not (left_units and right_units):
            new_units = self.merge_units_by_set_union(left_units, right_units)
            # CHECK FOR DIVISION AND EMPTY LEFT BRANCH
            if token.str in ['/', '/='] and not left_units:
                # FLIP SIGNS ON NEW UNITS
                for u in new_units:
                    for k, v in u.iteritems():
                        u[k] = -1 * v
            # WEAKEN INFERENCE IF WE'RE MULTIPLYING OR
            # DIVIDING ON CONSTANTS OR UNKNOWN VARIABLES
            if token.str in ['*', '/', '*=', '/=']:
                if new_units in \
                        self.my_symbol_helper.dimensionless_units_as_lists:
                    new_units = []
                # DETECT OPERATIONS ON CONSTANTS, like x = y * 1024.23
                if left_token and not left_units:
                    if left_token.isNumber:  # TRUE FOR EITHER INT OR FLOAT
                        self.is_unit_propagation_based_on_constants = True
                        token.is_unit_propagation_based_on_constants = True
                if right_token and not right_units:
                    if right_token.isNumber:  # TRUE FOR EITHER INT OR FLOAT
                        self.is_unit_propagation_based_on_constants = True
                        token.is_unit_propagation_based_on_constants = True
                # DETECT OPERATIONS ON UNKNOWN VARIABLES
                if left_units or right_units:  # AT LEAST ONE
                    if left_token and not left_units:
                        self.is_unit_propagation_based_on_unknown_variable \
                            = True
                        token.is_unit_propagation_based_on_unknown_variable \
                            = True
                    if right_token and not right_units:
                        self.is_unit_propagation_based_on_unknown_variable \
                            = True
                        token.is_unit_propagation_based_on_unknown_variable \
                            = True

            if new_units != token.units:
                token.units = new_units
                # PROPAGATE UNCERTAINTY BASED ON CONSTANTS OR
                # UNKNOWN VARIABLES
                self.apply_propagation_status_to_token(token,
                                                       left_token,
                                                       right_token)
                self.was_some_unit_changed = True
                if self.debug_verbose:
                    s = "tw. units changed in %s" % inspect.stack()[0][3]
                    print s
            return

        new_units = []
        if token.str in ['+', '-', '+=', '-=', '*', '/', '*=', '/='] \
                and token.isOp:
            # ADDITION / SUBTRACTION - ATTEMPT TO MERGE
            if token.str in ['+', '-', '+=', '-=']:
                # ADDITION - APPLY SET UNION
                new_units = self.merge_units_by_set_union(left_units,
                                                          right_units)
            # MULTIPLICATION / DIVISION
            elif token.str in ['*', '*=', '/', '/=']:
                all_unit_dicts_from_multiplication = []
                for unit_dict_left in left_units:
                    for unit_dict_right in right_units:
                        if self.debug:
                            s = "info: calling "
                            s += "apply_multiplication_to_unit_dict: "
                            s += "%s:%s %s %d %d"
                            print s % (token.file,
                                       str(token.linenr),
                                       token.str,
                                       len(left_units),
                                       len(right_units))
                            s = "info: calling "
                            s += "apply_multiplication_to_unit_dict: "
                            s += "%s:%s %s %d %d"
                        result_units = self.apply_multiplication_to_unit_dicts(
                                unit_dict_left,
                                unit_dict_right,
                                token.str)
                        if result_units:
                            all_unit_dicts_from_multiplication.append(
                                    result_units)
                for u in all_unit_dicts_from_multiplication:
                    if u not in new_units:
                        # PROPAGATE UNCERTAINTY BASED ON
                        # CONSTANTS OR UNKNOWN VARIABLES
                        self.apply_propagation_status_to_token(token,
                                                               left_token,
                                                               right_token)
                        new_units.append(u)

        # CHECK FOR UNITLESS TOKENS AS THE RESULT OF OPERATIONS
        if self.should_check_unitless_during_multiplication:
            for unit_dict in new_units:
                for k, v in unit_dict.iteritems():
                    if v == 0:
                        # UNITLESS
                        error_string = "%s : %s UNITLESS %s" % (
                                token.file,
                                token.linenr,
                                new_units)
                        self.handle_error(token, error_string)

        # UNIFY TOKENS FROM CHILDREN WITH CURRENT TOKENS
        if new_units != token.units:
            are_all_units_zero = True  # ASSUME TRUE
            are_any_units_zero = False
            for i, u in enumerate(new_units):
                zero_test_list = [v == 0 for v in u.values()]
                are_all_units_zero = all(zero_test_list)
                are_any_units_zero = any(zero_test_list)
                # WHEN WE HAVE SOME ZERO BUT NOT ALL, FILTER THE ZERO UNITS
                if are_any_units_zero and not are_all_units_zero:
                    new_units[i] = {k: v for k, v in u.iteritems() if v != 0}
        if new_units != token.units:
            # PROPAGATE UNCERTAINTY BASED ON CONSTANTS OR UNKNOWN VARIABLES
            self.apply_propagation_status_to_token(token,
                                                   left_token,
                                                   right_token)
            token.units = new_units
            self.was_some_unit_changed = True
            if self.debug_verbose:
                s = "tw. units changed in %s" % inspect.stack()[0][3]
                print s

    def apply_propagation_status_to_token(self,
                                          token,
                                          left_token,
                                          right_token):
        ''' APPLIES PROPAGATION WEAKENING FROM CHILD TOKENS TO PARENT TOKEN
            input:  token, left_token, right_token  all tokens
            returns: none.  side effect can change variables on 'token'
            '''
        if left_token:
            if left_token.is_unit_propagation_based_on_constants:
                token.is_unit_propagation_based_on_constants = True
            if left_token.is_unit_propagation_based_on_unknown_variable:
                token.is_unit_propagation_based_on_unknown_variable = True
        if right_token:
            if right_token.is_unit_propagation_based_on_constants:
                token.is_unit_propagation_based_on_constants = True
            if right_token.is_unit_propagation_based_on_unknown_variable:
                token.is_unit_propagation_based_on_unknown_variable = True

    def apply_multiplication_to_unit_dicts(self,
                                           unit_dict_left,
                                           unit_dict_right,
                                           op):
        ''' APPLIES MULTIPLICATION AND DIVISION TO UNIT DICTIONARIES
            (BY ADDING EXPONENTS)
            input:  unit_dict_left   dictionary of units.
                ex:  {'m':1, 's':-1} is meters per second
                    unit_dict_right  same
                    op   string representing mult or div operators
            returns: new dict  with resulting units  ex: {'m':2, 's':-2}
            '''
        if self.debug_verbose:
            self.cps_unit_checker.handle_debug_verbose(inspect.stack()[0][3])
        return_dict = {}
        # SPECIAL HANDLING FOR RADIANS AND QUATERNIONS
        if unit_dict_left in self.my_symbol_helper.dimensionless_units \
                and unit_dict_right \
                in self.my_symbol_helper.dimensionless_units:
            # SPECIAL CASE BOTH ARE RADIANS.  CLOSED UNDER MULTIPLICATION
            return copy.deepcopy(unit_dict_left)
        if unit_dict_left not in self.my_symbol_helper.dimensionless_units \
                and unit_dict_right \
                not in self.my_symbol_helper.dimensionless_units:
            return_dict = copy.deepcopy(unit_dict_left)
        if unit_dict_left in self.my_symbol_helper.dimensionless_units:
            # DON'T PROPAGATE RADIANS
            unit_dict_left = {}
            return_dict = copy.deepcopy(unit_dict_right)
        if unit_dict_right in self.my_symbol_helper.dimensionless_units:
            # DON'T PROPAGATE RADIANS
            unit_dict_right = {}
            return_dict = copy.deepcopy(unit_dict_left)

        return_dict = copy.deepcopy(unit_dict_left)
        for unit in unit_dict_right:
            if unit in return_dict:
                # MULT WHEN BOTH DICTS HAVE SAME UNIT
                if op in ['*', '*=']:
                    # ADD OF EXPONENT IS MULT
                    return_dict[unit] += unit_dict_right[unit]
                # DIV WHEN BOTH DICTS HAVE SAME UNIT
                elif op in ['/', '/=']:
                    # SUBTRACTION OF EXPONENT IS DIV
                    return_dict[unit] -= unit_dict_right[unit]
            else:
                # ONLY ONE SIDE HAS UNIT
                if op in ['*', '*=']:
                    # COPY - THIS IS NOT A REFERNCE
                    return_dict[unit] = unit_dict_right[unit]
                elif op in ['/', '/=']:
                    return_dict[unit] = -1 * unit_dict_right[unit]  # DIVSION
        # FILTER OUT ZEROs - UNITLESS
        return_dict = {k: v for k, v in return_dict.items() if v != 0}
        return return_dict

    def collect_function_parameters_units_and_decorate_function_objects(
            self, token, left_token, right_token):
        ''' COLLECT AVAILABLE UNITS ON FUNCTION PARAMETERS FROM
            AST AND ADD THEM TO FUNCTION OBJECT
        '''
        if token.function:
            function_args_units = []
            if token.astParent.astOperand2:
                if token.astParent.astOperand2.str == ',':
                    # MULITPLE ARGS
                    # EXPECT A LIST OF LISTS
                    function_args_units = \
                            self.recurse_on_function_args(
                                    token.astParent.astOperand2)
                else:
                    # ONLY ONE ARG
                    if token.astParent.astOperand2.units:
                        # LIST OF LIST
                        function_args_units = \
                                [token.astParent.astOperand2.units]
            if function_args_units:
                if len(function_args_units) != len(token.function.arg_units):
                    if self.debug:
                        print token.file
                        s = 'line %s   '
                        s += 'len(function_args_units)'
                        s += '== %d len(token.function.arg_units) = %d'
                        print s % (token.linenr,
                                   len(function_args_units),
                                   len(token.function.arg_units))
                    return
                for i, u in enumerate(function_args_units):
                    new_dict = {'linenr': int(token.linenr),
                                'units': u,
                                'functionId': token.function.Id,
                                'token': token}
                    if new_dict not in token.function.arg_units[i]:
                        token.function.arg_units[i].append(new_dict)

    def recurse_on_function_args(self, comma_token):
        ''' RECURSIVELY COLLECT UNITS FOR FUNCTION ARGUMENTS
            input: ccpcheck token object str=','
            output: list of units in arg order
            '''
        my_return_list = []
        if comma_token.astOperand1:
            if comma_token.astOperand1.str == ',':
                left_branch_units_list =  \
                    self.recurse_on_function_args(comma_token.astOperand1)
                if left_branch_units_list and \
                        isinstance(left_branch_units_list[0], list):
                    for u in left_branch_units_list:
                        my_return_list.append(u)
                else:
                    my_return_list.append(left_branch_units_list)
            else:
                my_return_list.append(comma_token.astOperand1.units)
        if comma_token.astOperand2:
            my_return_list.append(comma_token.astOperand2.units)
        return my_return_list

    def debug_walk(self, token):
        ''' RUDIMENTARY TERMINAL PRINT HOTWASH DEBUG
        '''
        if self.debug_verbose:
            self.cps_unit_checker.handle_debug_verbose(inspect.stack()[0][3])

        if token.isRoot:
            print '- '*42 + ' ROOT'
            if self.source_file_lines:
                print "lines yes",
                if self.source_file in token.file:
                    print 'source in token'
                    print self.source_file_lines[int(token.linenr)-1],
                else:
                    print 'self.source_file not in token.file'

        self.recursive_debug_walk(token, 0)

    def recursive_debug_walk(self, token, indent):
        if self.debug_verbose:
            self.cps_unit_checker.handle_debug_verbose(inspect.stack()[0][3])
        if not token:
            return

        print '%s %s   %s             ' % (token.linenr,
                                           "-" * indent,
                                           token.str),
        if token.variable:
            print self.my_symbol_helper.find_variable_type(token.variable),
            print self.my_symbol_helper.find_compound_variable_name_for_variable_token(token),
        if token.units:
            print token.units,
        if token.function:
            print token.function.argument,
            print 'ret: ' + str(token.function.return_units),
        if token.is_unit_propagation_based_on_constants:
            print ' C ',
        if token.is_unit_propagation_based_on_unknown_variable:
            print ' UV ',
        print
        self.recursive_debug_walk(token.astOperand1, indent + 1)  # LEFT
        self.recursive_debug_walk(token.astOperand2, indent + 1)  # RIGHT
        if token.link:
            print '%s %s   %s' % (token.link.linenr,
                                  "+" * indent,
                                  token.link.str)

    def find_first_variable(self, token, left_token, right_token):
        if self.debug_verbose:
            self.cps_unit_checker.handle_debug_verbose(inspect.stack()[0][3])
        ''' ASSUME THIS WILL BE CALLED ON LHS OF ASSIGNMENT.
            SHOULD ONLY BE ONE VARIABLE
        '''
        if token.variable:
            self.return_value_list.append(token.variable)

    def recurse_and_collect_string(self, AST_token):
        if self.debug_verbose:
            self.cps_unit_checker.handle_debug_verbose(inspect.stack()[0][3])
        """ SIMPLE RECURSION FOR LEFT-HAND SIDE OF ASSIGNMENT STATMENTS
        """
        # PROTECT FROM NULL
        if not AST_token:
            return ''
        my_return_string = ''
        # LEFT RECURSE
        if AST_token.astOperand1:
            my_return_string +=  \
                self.recurse_and_collect_string(AST_token.astOperand1)
        # SELF
        my_return_string += AST_token.str
        # RIGHT RECURSE
        if AST_token.astOperand2:
            my_return_string +=  \
                self.recurse_and_collect_string(AST_token.astOperand2)

        return my_return_string
