import copy
import re

class SymbolHelper:
    ''' HELPS FIND DEFINITIONS OF SYMBOLS AND DECORATES CPPCHECK SYMBOL TABLE
    '''

    def __init__(self, function_dictionary):
        self.function_dictionary = function_dictionary
        self.should_use_variable_names = False
        self.should_ignore_time_and_math = False
        self.should_use_dt_heuristic = True
        self.is_weak_inference = False
        self.debug = False
        self.ros_unit_dictionary = {}
        self.list_of_vetted_units = []
        self.cps_unit_checker = None   # DEPENDENCY INJECTION.  SEE MARTIN FOWLER
        self.initialize_ros_unit_dictionary()
        self.initialize_function_dictionary()
        self.initialize_list_of_vetted_units()
        self.should_match_on_heuristic_variable_names = False #USED TO COMPARE MORE NAIVE METHOD OF ASSIGNING UNITS BY SUBSTRING, LIKE 'linear' = {'meter':1, 'second':-1}
        self.debug_missed_class_names_output_file = 'all_missed_class_name_lookups.txt'
        self.debug_log_missed_class_names = False
        self.weak_inference_classes = ['sensor_msgs::JointState']
        # THESE DIMENSIONLESS UNITS ACT AS A '1' DURTION DIMENSION OPERATIONS, EXEPCT FOR ADDITION.  
        # EXAMPLES    radians + meters NOT OK.  radians * meters = meters.  quaternion * quaternion = quaternion
        self.dimensionless_units = [{'radian':1.}, {'quaternion':1.}]
        self.dimensionless_units_as_lists = [[{'radian':1.}], [{'quaternion':1.}]]


    # STACK OVERFLOW
    def rreplace(self, s, old, new, occurance):
        li = s.rsplit(old, occurance)
        return new.join(li)


    def sanitize_class_name(self, class_name):
        ''' TAKES A VARIABLE NAME AND STRIPTS EXTRA CLASS INFORMATION FROM FRONT AND BACK
            input: class_name   'constPtr<tf2::Transform::iterator>'  <-- LOL
            output: str         'trf2::Transform'
            '''
        is_some_change = True
        while (is_some_change):# SOME OF THESE CAN BE CASCADED IN DIFFERENT ORDERS
            is_some_change = False

            # SIMPLIFY ITERATORS
            if str(class_name).endswith('::iterator'):
                class_name = self.rreplace(class_name, '::iterator', '', 1)
                is_some_change = True
            if str(class_name).endswith('::const_iterator'):
                class_name = self.rreplace(class_name, '::const_iterator', '', 1)
                is_some_change = True


            # SIMPLIFY REFERNCES
            if str(class_name).endswith('&'):
                class_name = class_name.replace('&', '')
                is_some_change = True
            # SIMPLIFY CONST
            if str(class_name).endswith('const'):
                class_name = self.rreplace(class_name, 'const', '', 1)
                is_some_change = True
            # SIMPLIFY CONSTANT POINTERS (REFRENCES)
            if str(class_name).endswith('ConstPtr'):
                class_name = class_name.replace('ConstPtr', '')
                is_some_change = True
            # SIMPLIFY POINTERS
            if str(class_name).endswith('::Ptr'):
                class_name = self.rreplace(class_name, '::Ptr', '', 1)
                is_some_change = True
            if str(class_name).endswith('Ptr'):
                class_name = self.rreplace(class_name, 'Ptr', '', 1)
                is_some_change = True
            # SIMPLIFY BOOST POINTERS
            if str(class_name).startswith('boost::shared_ptr<') and str(class_name).endswith('>'):
                class_name = class_name.replace('boost::shared_ptr<', '')
                class_name = self.rreplace(class_name, '>', '', 1)
                is_some_change = True
            if str(class_name).startswith('shared_ptr<') and str(class_name).endswith('>'):
                class_name = class_name.replace('shared_ptr<', '')
                class_name = self.rreplace(class_name, '>', '', 1)
                is_some_change = True
            if str(class_name).startswith('boost::scoped_ptr<') and str(class_name).endswith('>'):
                class_name = class_name.replace('boost::scoped_ptr<', '')
                class_name = self.rreplace(class_name, '>', '', 1)
                is_some_change = True
            if str(class_name).startswith('scoped_ptr<') and str(class_name).endswith('>'):
                class_name = class_name.replace('scoped_ptr<', '')
                class_name = self.rreplace(class_name, '>', '', 1)
                is_some_change = True

            if str(class_name).startswith('tf::MessageFilter<') and str(class_name).endswith('>'):
                class_name = class_name.replace('tf::MessageFilter<', '')
                class_name = self.rreplace(class_name, '>', '', 1)
                is_some_change = True
            if str(class_name).startswith('tf2_ros::MessageFilter<') and str(class_name).endswith('>'):
                class_name = class_name.replace('tf_ros::MessageFilter<', '')
                class_name = self.rreplace(class_name, '>', '', 1)
                is_some_change = True
            if str(class_name).startswith('MessageFilter<') and str(class_name).endswith('>'):
                class_name = class_name.replace('MessageFilter<', '')
                class_name = self.rreplace(class_name, '>', '', 1)
                is_some_change = True

            # SIMPLIFY VECTORS
            if str(class_name).startswith('std::vector<') and str(class_name).endswith('>'):
                class_name = class_name.replace('std::vector<', '')
                class_name = self.rreplace(class_name, '>', '', 1)
                is_some_change = True
            if str(class_name).startswith('conststd::vector<') and str(class_name).endswith('>'):
                class_name = class_name.replace('std::vector<', '')
                class_name = self.rreplace(class_name, '>', '', 1)
                is_some_change = True
            if str(class_name).startswith('vector<') and str(class_name).endswith('>'):
                class_name = class_name.replace('vector<', '')
                class_name = self.rreplace(class_name, '>', '', 1)
                is_some_change = True
            if str(class_name).startswith('std::deque<') and str(class_name).endswith('>'):
                class_name = class_name.replace('std::vector<', '')
                class_name = self.rreplace(class_name, '>', '', 1)
                is_some_change = True
            if str(class_name).startswith('deque<') and str(class_name).endswith('>'):
                class_name = class_name.replace('vector<', '')
                class_name = self.rreplace(class_name, '>', '', 1)
                is_some_change = True
            # SIMPLIFY POINTERS
            if str(class_name).endswith('*'):
                class_name = self.rreplace(class_name, '*', '', 1)
                is_some_change = True
            # SIMPLIFY DANGLING ::
            if str(class_name).endswith('::'):
                class_name = self.rreplace(class_name, '::', '', 1)
                is_some_change = True
        # SANITIZED VERSION
        return class_name



    def find_units_for_variable(self, token):
        ''' input:  token       a cppcheckdata token that contains a variable 
            returns: a dictionary of unit types
            '''
        # SIMPLE NAME MATCHING HEURISTIC
        if self.should_match_on_heuristic_variable_names:
            return self.get_unit_dict_STRING_MATCH_HEURISTIC(token)

        # ASSUME FALSE
        self.is_weak_inference = False

        raw_class_name = self.find_variable_type(token.variable)

        # REMOVE EXTRA, UNNECESSARY INFORMATION FROM FRONT AND BACK
        class_name = self.sanitize_class_name(raw_class_name)

        # IF CLASS IS A ROS MESSAGE
        if class_name in self.ros_unit_dictionary:
            # FIND POSSIBLE ATTRIBUTES
            possible_attributes_as_list = self.find_compound_variable_name_for_variable_token(token).split(".")
            my_return_units = None
            if possible_attributes_as_list:
                # STARTED AS SPECIAL CASE FOR LaserScan
                # CORNER CASES WHEN WE DONT' ASSIGN UNITS
                # if (class_name == 'sensor_msgs::LaserScan') and 'size' in possible_attributes_as_list:
                if 'size' == possible_attributes_as_list[-1]:
                    return {}
                if 'empty' == possible_attributes_as_list[-1]:
                    return {}
                if 'isZero' ==  possible_attributes_as_list[-1]:
                    return {}
                # THE USUAL EXPECTED CASE 
                for p in possible_attributes_as_list:
                    if p in self.ros_unit_dictionary[class_name]:
                        my_return_units = self.ros_unit_dictionary[class_name][p]
            if not self.should_ignore_time_and_math:
                if class_name in [ 'ros::Time', 'ros::Duration', 'std::vector<ros::Duration>']:
                    my_return_units = self.ros_unit_dictionary[class_name]['secs']
            if my_return_units:
                if self.cps_unit_checker:
                    if self.cps_unit_checker.SHOULD_FIND_ALL_UNITS:
                        self.cps_unit_checker.add_class_and_units_to_all_list(class_name, my_return_units)
                        if not class_name in self.cps_unit_checker.all_classes_and_units_for_this_file_as_dict:
                            self.cps_unit_checker.all_classes_and_units_for_this_file_as_dict[class_name] = my_return_units
                        else:
                            pass  # WE DON'T CASE BECAUSE WE'RE ONLY TRACKING UNIQUE UNITS, NOT COUNTS
                # CHECK FOR WEAK INFERENCE CLASSES, SUCH AT JOINT_STATE BECAUSE THAT COULD BE r/s or m/s
                if class_name in self.weak_inference_classes:
                    self.is_weak_inference = True
                return my_return_units
        else:
            if self.should_use_dt_heuristic and not self.should_ignore_time_and_math:
                if token.str.lower() == 'dt':
                    return self.ros_unit_dictionary['dt']['dt']
            if self.debug_log_missed_class_names:
                with open(self.debug_missed_class_names_output_file, 'a') as f:
                    f.write(class_name + '\n')
        # REACHNG HERE MEANS WE FOUND NOTHING
        return {}


    def find_compound_variable_name_for_variable_token(self, token):
        ''' input: a variable token from cppcheckdata
            returns: string containing the compound variable name.  ex:  'my_var_.linear.x'
            '''
        if not token.variable:
            raise ValueError('received a non variable token for tokenid:%s str:%s' % (token.Id, token.str))
        if token.astParent.str != '.':
            return token.str
        compound_variable_root_token = token.astParent
        while compound_variable_root_token.astParent and compound_variable_root_token.astParent.str == '.':
            compound_variable_root_token = compound_variable_root_token.astParent
        return self.recursively_visit(compound_variable_root_token)
            
    def recursively_visit(self, token):
        ''' input: a cppcheckdata token
            returns: string aggregation of tokens under the root
            '''
        my_return_string = ''
        if token.astOperand1: my_return_string += self.recursively_visit(token.astOperand1)
        my_return_string += token.str 
        if token.astOperand2:  my_return_string += self.recursively_visit(token.astOperand2)
        return my_return_string


    #def get_unit_dict_given_class_and_property_as_strings(self, class_name, property_name):
        #''' input:  class name (ex: 'sensor_msgs::BatteryState')
                    #property_name (ex: 'cell_voltage')
            #returns: dict containing units, if found, otherwise empty dict  (ex: '{'volt': 1})
            #'''
        #if self.should_match_on_heuristic_variable_names:
            #return self.get_unit_dict_STRING_MATCH_HEURISTIC()

        #nope
        #print class_name
        #foo
        #class_name = class_name.replace('::ConstPtr&', '')

        #if class_name not in self.ros_unit_dictionary:
            #return {}
        #if property_name not in self.ros_unit_dictionary[class_name]:
            #return {}
        #return self.ros_unit_dictionary[class_name][property_name]

    def initialize_function_dictionary(self):
        #self.functionDictionary
        pass

    def initialize_ros_unit_dictionary(self):


        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  WRENCH
        twist_dict                                      = {'linear': {'meter': 1., 'second': -1.}, 
                                                            'angular': {'second': -1.}}
        accel_dict                                      = {'linear': {'meter': 1., 'second': -2.}, 
                                                            'angular': {'second': -2.}}
                                                            #'covariance'} todo
        wrench_dict                                     = {'force': {'kilogram':1., 'meter': 1., 'second':-2.}, 
                                                            'torque': {'kilogram':1., 'meter': 2., 'second':-2.}}
        inertia_dict                                    = {'m': {'kilogram': 1.},
                                                            'com': {'meter':1.},
                                                            'ixx': {'kilogram':1., 'meter':-2. },
                                                            'ixy': {'kilogram':1., 'meter':-2. },
                                                            'ixz': {'kilogram':1., 'meter':-2. },
                                                            'iyy': {'kilogram':1., 'meter':-2. },
                                                            'iyz': {'kilogram':1., 'meter':-2. },
                                                            'izz': {'kilogram':1., 'meter':-2. }}
        transform_dict                                  = {'translation': {'meter': 1.},
                                                            'rotation': {'quaternion':1.},
                                                            'stamp':{'second':1.}}
    # GEOMETRY MSGS   http://docs.ros.org/api/geometry_msgs/html/index-msg.html
        
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  ACCEL
        self.ros_unit_dictionary['geometry_msgs::Accel'] = accel_dict
        self.ros_unit_dictionary['geometry_msgs::AccelStamped'] = copy.deepcopy(accel_dict)
        self.ros_unit_dictionary['geometry_msgs::AccelStamped']['stamp'] = {'second':1.}
        self.ros_unit_dictionary['geometry_msgs::AccelWithCovariance'] = accel_dict
        self.ros_unit_dictionary['geometry_msgs::AccelWithCovarianceStamped'] = copy.deepcopy(accel_dict)
        self.ros_unit_dictionary['geometry_msgs::AccelWithCovarianceStamped']['stamp'] = {'second':1.}

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  INERTIA
        self.ros_unit_dictionary['geometry_msgs::Inertia'] = inertia_dict
        self.ros_unit_dictionary['geometry_msgs::InertiaStamped'] = inertia_dict

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  POINT
        self.ros_unit_dictionary['geometry_msgs::Point'] =       {'x': {'meter': 1.},
                                                                  'y': {'meter':1.},
                                                                  'z': {'meter':1.}}
        self.ros_unit_dictionary['std::vector<geometry_msgs::Point>'] =       {'x': {'meter': 1.},
                                                                  'y': {'meter':1.},
                                                                  'z': {'meter':1.}}
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  POINT
        self.ros_unit_dictionary['geometry_msgs::Point32'] =       {'x': {'meter': 1.},
                                                                  'y': {'meter':1.},
                                                                  'z': {'meter':1.}}
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  POINT
        self.ros_unit_dictionary['geometry_msgs::PointStamped'] = {'x': {'meter': 1.},
                                                                  'y': {'meter':1.},
                                                                  'z': {'meter':1.},
                                                                    'stamp':{'second':1.}}
        self.ros_unit_dictionary['geometry_msgs::PointStampedPtr'] = {'x': {'meter': 1.},
                                                                  'y': {'meter':1.},
                                                                  'z': {'meter':1.},
                                                                    'stamp':{'second':1.}}
        self.ros_unit_dictionary['MessageFilter<geometry_msgs::PointStamped>'] = {'x': {'meter': 1.},
                                                                  'y': {'meter':1.},
                                                                  'z': {'meter':1.},
                                                                    'stamp':{'second':1.}}
        self.ros_unit_dictionary['std::vector<geometry_msgs::PointStamped>'] = {'x': {'meter': 1.},
                                                                  'y': {'meter':1.},
                                                                  'z': {'meter':1.},
                                                                    'stamp':{'second':1.}}
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  POLYGON
        self.ros_unit_dictionary['geometry_msgs::Polygon'] = {'points': {'meter': 1.}}
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  POLYGONSTAMPED
        self.ros_unit_dictionary['geometry_msgs::PolygonStamped'] = {'points': {'meter': 1.},
                                                                    'stamp':{'second':1.}}
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  POSE
        self.ros_unit_dictionary['geometry_msgs::Pose'] =       {'position': {'meter': 1.},
                                                                  'orientation': {'quaternion':1.}}
        self.ros_unit_dictionary['std::Vector<geometry_msgs::Pose>'] =       {'position': {'meter': 1.},
                                                                  'orientation': {'quaternion':1.}}

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  POSE
        self.ros_unit_dictionary['geometry_msgs::Pose2D'] =       {'x': {'meter': 1.},
                                                                  'y': {'meter':1.}}
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  POSESTAMPED
        self.ros_unit_dictionary['geometry_msgs::PoseStamped'] = {'position': {'meter': 1.},
                                                                  'orientation': {'quaternion':1.},
                                                                    'stamp':{'second':1.}}
        self.ros_unit_dictionary['std::Vector<geometry_msgs::PoseStamped>'] = {'position': {'meter': 1.},
                                                                  'orientation': {'quaternion':1.},
                                                                    'stamp':{'second':1.}}
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  POSESTAMPED
        self.ros_unit_dictionary['geometry_msgs::PoseWithCovariance'] = {'position': {'meter': 1.},
                                                                  'orientation': {'quaternion':1.}}
        self.ros_unit_dictionary['geometry_msgs::PoseWithCovariance::_covariance_type'] = {'position': {'meter': 1.},
                                                                  'orientation': {'quaternion':1.}}
        self.ros_unit_dictionary['geometry_msgs::PoseWithCovarianceStamped'] = {'position': {'meter': 1.},
                                                                  'orientation': {'quaternion':1.},
                                                                    'stamp':{'second':1.}}
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  POSE ARRAY  todo
        #self.ros_unit_dictionary['geometry_msgs::PoseArray'] = {'position': {'meter': 1.},
                                                                  #'orientation': {'quaternion':1.}}
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  Quaternion
        self.ros_unit_dictionary['geometry_msgs::Quaternion'] =       {'x': {'quaternion': 1.},
                                                                  'y': {'quaternion':1.},
                                                                  'z': {'quaternion':1.},
                                                                  'w': {'quaternion':1.}}
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  QUATERNION STAMPED
        self.ros_unit_dictionary['geometry_msgs::QuaternionStamped'] =       {'x': {'quaternion': 1.},
                                                                  'y': {'quaternion':1.},
                                                                  'z': {'quaternion':1.},
                                                                  'w': {'quaternion':1.},
                                                                    'stamp':{'second':1.}}
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  TRANSFORM
        self.ros_unit_dictionary['geometry_msgs::Transform'] = transform_dict
        self.ros_unit_dictionary['geometry_msgs::TransformStamped'] = transform_dict
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  TWIST
        self.ros_unit_dictionary['geometry_msgs::Twist'] = twist_dict
        self.ros_unit_dictionary['geometry_msgs::TwistStamped'] = copy.deepcopy(twist_dict)
        self.ros_unit_dictionary['geometry_msgs::TwistStamped']['stamp'] = {'second':1.}
        self.ros_unit_dictionary['geometry_msgs::TwistWithCovariance'] = copy.deepcopy(twist_dict)
        self.ros_unit_dictionary['geometry_msgs::TwistWithCovarianceStamped'] = copy.deepcopy(twist_dict)
        self.ros_unit_dictionary['geometry_msgs::TwistWithCovarianceStamped']['stamp'] = {'second':1.}
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  WRENCH
        self.ros_unit_dictionary['geometry_msgs::Wrench'] = wrench_dict
        self.ros_unit_dictionary['geometry_msgs::WrenchStamped'] = copy.deepcopy(wrench_dict)
        self.ros_unit_dictionary['geometry_msgs::WrenchStamped']['stamp'] = {'second':1.}


    # NAVIGATION MSGS       http://docs.ros.org/api/nav_msgs/html/index-msg.html
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  GRID CELLS
        self.ros_unit_dictionary['nav_msgs::GridCells'] = {'cell_width': {'meter': 1.}, 
                                                                  'cell_height': {'meter':1.}}
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  MAP META DATA
        self.ros_unit_dictionary['nav_msgs::MapMetaData'] = {'map_load_time': {'second': 1.}} 
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  ODOMETRY
        self.ros_unit_dictionary['nav_msgs::Odometry'] = {'position': {'meter': 1.},
                                                                  'orientation': {'quaternion':1.},
                                                                    'stamp':{'second':1.},
                                                                    'linear':{'meter':1.,'second':-1.},
                                                                    'angular':{'second':-1.}}
        self.ros_unit_dictionary['nav_msgs::Plan'] = {'position': {'meter': 1.},
                                                                  'orientation': {'quaternion':1.},
                                                                    'stamp':{'second':1.}}


    # SENSOR MSGS   http://docs.ros.org/api/sensor_msgs/html/index-msg.html
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  BATTERY   todo: not implemented in Indigo
        self.ros_unit_dictionary['sensor_msgs::BatteryState'] = {'voltage': {'volt': 1.},
                                                             'cell_voltage': {'volt': 1.},
                                                             'current': {'amp': 1., 'hour':1.},
                                                             'capacity': {'amp': 1., 'hour':1.},
                                                             'design_capacity': {'amp': 1., 'hour':1.},
                                                             'stamp':{'second':1.}}
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  FLUID PRESSURE 
        self.ros_unit_dictionary['sensor_msgs::FluidPressure'] = {'fluid_pressure': {'pascal': 1.},
                                                                 'variance': {'pascal': 2.},
                                                                 'stamp':{'second':1.}}
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  ILLUMINANCE
        self.ros_unit_dictionary['sensor_msgs::Illuminance'] = {'illuminance': {'lux': 1.},
                                                                 'variance': {'lux': 2.},
                                                                 'stamp':{'second':1.}}
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  IMU
        self.ros_unit_dictionary['sensor_msgs::Imu'] =      {'angular_velocity': {'second': -1.},
                                                             'angular_velocity_covariance': {'second': -2.},
                                                             'linear_acceleration': {'meter': 1., 'second':-2.},
                                                             'linear_acceleration_covariance': {'meter': 2., 'second':-4.},
                                                             'orientation': {'quaternion': 1.},
                                                             'orientation_covariance': {'quaternion': 2.},
                                                             'stamp':{'second':1.}}
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  JOINT STATE  #todo: Improve on revolute assumption
        self.ros_unit_dictionary['sensor_msgs::JointState'] = { 'velocity': {'second': -1.},
                                                                'effort': {'kilogram': 1., 'meter':2., 'second':-2.}}
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  LASER ECHO
        self.ros_unit_dictionary['sensor_msgs::LaserEcho'] = {'echoes': {'meter': 1.}}
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  LASERSCAN
        self.ros_unit_dictionary['sensor_msgs::LaserScan'] = {'time_increment': {'second': 1.},
                                                             'scan_time': {'second': 1.},
                                                             'range_min': {'meter': 1.},
                                                             'range_max': {'meter': 1.},
                                                             'ranges': {'meter': 1.},
                                                             'stamp':{'second':1.}}
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  MAGNETIC FIELD
        self.ros_unit_dictionary['sensor_msgs::MagneticField'] = {'magnetic_field': {'tesla': 1.},
                                                                    'magnetic_field_covariance': {'tesla': 2.},
                                                                    'stamp':{'second':1.}}
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  MULTI DOF JOINT STATE
        self.ros_unit_dictionary['sensor_msgs::MultiDOFJointState'] = {'twist': twist_dict,
                                                                        'wrench': wrench_dict,
                                                                        'transforms': transform_dict,
                                                                         'stamp':{'second':1.}}
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  MULTI ECHO LASERSCAN
        self.ros_unit_dictionary['sensor_msgs::MultiEchoLaserScan'] = {'time_increment': {'second': 1.},
                                                             'scan_time': {'second': 1.},
                                                             'range_min': {'meter': 1.},
                                                             'range_max': {'meter': 1.},
                                                             'ranges': {'meter': 1.},
                                                             'stamp':{'second':1.}}
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  NAV SAT FIX
        self.ros_unit_dictionary['sensor_msgs::NavSatFix'] = {'latitude': {'degree_360': 1.},
                                                             'longitude': {'degree_360': 1.},
                                                             'altitude': {'meter': 1.},
                                                             'position_covariance': {'meter': 2.},
                                                             'stamp':{'second':1.}}
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  POINT CLOUD
        self.ros_unit_dictionary['sensor_msgs::PointCloud'] = {'points': {'meter': 1.},
                                                             'stamp':{'second':1.}}
        self.ros_unit_dictionary['sensor_msgs::PointCloud2'] = {'points': {'meter': 1.},
                                                             'stamp':{'second':1.}}
        self.ros_unit_dictionary['sensor_msgs::PointCloud2Iterator<float>'] = {'points': {'meter': 1.},
                                                             'stamp':{'second':1.}}
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  RANGE
        self.ros_unit_dictionary['sensor_msgs::Range'] =  {'min_range': {'meter':1.},
                                                                'max_range': {'meter':1.},
                                                                'range': {'meter':1.},
                                                                'stamp':{'second':1.},
                                                                'field_of_view':{'radian':1.}}
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  TEMPERATURE
        self.ros_unit_dictionary['sensor_msgs::Temperature'] = {'temperature': {'degree_celsius': 1.},
                                                                'stamp':{'second':1.}}



    # STEREO MSGS       http://docs.ros.org/api/stereo_msgs/html/index-msg.html
        self.ros_unit_dictionary['stereo_msgs::DisparityImage'] =  {'T': {'meter': 1.}, 
                                                                'min_disparity': {'meter':1.},
                                                                'delta_d': {'meter':1.},
                                                                'max_disparity': {'meter':1.}}
    # TRANSFORM MSGS
        self.ros_unit_dictionary['StampedTransform'] =       {'getOrigin': {'meter': 1.}, 
                                                            'getRotation': {'quaternion': 1.}}
        self.ros_unit_dictionary['tf::Quaternion'] =     {'getX': {'quaternion': 1.}, 
                                                            'getY': {'quaternion': 1.},
                                                            'getZ': {'quaternion': 1.},
                                                            'getW': {'quaternion': 1.}}
        self.ros_unit_dictionary['tf::Stamped<tf::Quaternion>'] =     {'getX': {'quaternion': 1.}, 
                                                            'getY': {'quaternion': 1.},
                                                            'getZ': {'quaternion': 1.},
                                                            'getW': {'quaternion': 1.}}
        self.ros_unit_dictionary['tf::Vector3'] =       {'getX': {'meter': 1.}, 
                                                            'getY': {'meter': 1.},
                                                            'getZ': {'meter': 1.},
							     'x': {'meter':1.},
							     'y': {'meter':1.},
							     'z': {'meter':1.}}
        self.ros_unit_dictionary['tf::Stamped<tf::Vector3>'] =       {'getX': {'meter': 1.}, 
                                                            'getY': {'meter': 1.},
                                                            'getZ': {'meter': 1.},
							     'x': {'meter':1.},
							     'y': {'meter':1.},
							     'z': {'meter':1.}}
        self.ros_unit_dictionary['tf::Pose'] =          {'getOrigin': {'meter': 1.}, 
                                                            'getRotation': {'quaternion': 1.}}
        self.ros_unit_dictionary['tf::Stamped<tf::Pose>'] =          {'getOrigin': {'meter': 1.}, 
                                                            'getRotation': {'quaternion': 1.}}
        self.ros_unit_dictionary['tf::Transform'] =       {'getOrigin': {'meter': 1.}, 
                                                            'getRotation': {'quaternion': 1.}}
        self.ros_unit_dictionary['tf::StampedTransform'] =       {'getOrigin': {'meter': 1.}, 
                                                            'getRotation': {'quaternion': 1.}}
        self.ros_unit_dictionary['tf2::Stamped<tf::Transform>'] =       {'getOrigin': {'meter': 1.}, 
                                                            'getRotation': {'quaternion': 1.}}
        self.ros_unit_dictionary['tf2::Quaternion'] =       {'getX': {'quaternion': 1.}, 
                                                            'getY': {'quaternion': 1.},
                                                            'getZ': {'quaternion': 1.},
                                                            'getW': {'quaternion': 1.}}
        self.ros_unit_dictionary['tf2::Stamped<tf2::Quaternion>'] =       {'getX': {'quaternion': 1.}, 
                                                            'getY': {'quaternion': 1.},
                                                            'getZ': {'quaternion': 1.},
                                                            'getW': {'quaternion': 1.}}
        self.ros_unit_dictionary['tf2::Vector3'] =       {'getX': {'meter': 1.}, 
                                                            'getY': {'meter': 1.},
                                                            'getZ': {'meter': 1.}}
        self.ros_unit_dictionary['tf2::Stamped<tf2::Vector3>'] =       {'getX': {'meter': 1.}, 
                                                            'getY': {'meter': 1.},
                                                            'getZ': {'meter': 1.}}
        self.ros_unit_dictionary['tf2::Pose'] =          {'getOrigin': {'meter': 1.}, 
                                                            'getRotation': {'quaternion': 1.}}
        self.ros_unit_dictionary['tf2::Stamped<tf2::Pose>'] =          {'getOrigin': {'meter': 1.}, 
                                                            'getRotation': {'quaternion': 1.}}
        self.ros_unit_dictionary['tf2::Transform'] =       {'getOrigin': {'meter': 1.}, 
                                                            'getRotation': {'quaternion': 1.}}
        self.ros_unit_dictionary['tf2::StampedTransform'] =       {'getOrigin': {'meter': 1.}, 
                                                            'getRotation': {'quaternion': 1.}}
        self.ros_unit_dictionary['tf2::Stamped<tf2::Transform>'] =       {'getOrigin': {'meter': 1.}, 
                                                            'getRotation': {'quaternion': 1.}}


    # ROS TIME
        if not self.should_ignore_time_and_math:
            self.ros_unit_dictionary['ros::Time'] =  {'secs': {'second': 1.}, 
                                                     'nsec': {'second': 1.}}  # ALL TIME UNITS ARE SECONDS REGARDLESS OF SCALE
            self.ros_unit_dictionary['ros::Duration'] =  {'secs': {'second': 1.}, 
                                                     'nsec': {'second': 1.}}  # ALL TIME UNITS ARE SECONDS REGARDLESS OF SCALE
            self.ros_unit_dictionary['std::vector<ros::Duration>'] =  {'secs': {'second': 1.}, 
                                                     'nsec': {'second': 1.}}  # ALL TIME UNITS ARE SECONDS REGARDLESS OF SCALE
    # KNOWN FUNCTIONS
        if not self.should_ignore_time_and_math:
            self.ros_unit_dictionary['atan2'] =  {'atan2': {'radian': 1.}} 
            self.ros_unit_dictionary['acos'] =  {'acos': {'radian': 1.}} 
            self.ros_unit_dictionary['asin'] =  {'asin': {'radian': 1.}} 
            self.ros_unit_dictionary['atan'] =  {'atan': {'radian': 1.}} 
            self.ros_unit_dictionary['toSec'] =  {'toSec': {'second': 1.}} 
        # self.ros_unit_dictionary['getX'] =  {'getX': {'meter': 1.}} 
        # self.ros_unit_dictionary['getY'] =  {'getY': {'meter': 1.}} 
        # self.ros_unit_dictionary['getZ'] =  {'getZ': {'meter': 1.}} 
        self.ros_unit_dictionary['quatToRPY'] =  {'quatToRPY': {'radian': 1.}} 
    # KNOWN SYMBOL
        # self.ros_unit_dictionary['M_PI'] =  {'M_PI': {'radian': 1}} 
        if self.should_use_dt_heuristic:
            self.ros_unit_dictionary['dt'] =  {'dt': {'second': 1}} 


            

    def find_variable_type(self, variable):
        ''' input: cppcheckdata variable object
            output: string of variable type  (ex:  'int'  or 'std::vector')
            '''
        my_return_string = variable.typeStartToken.str

        if variable.typeStartToken != variable.typeEndToken:
            nextToken = variable.typeStartToken.next
            hasNext = True
            while(hasNext and nextToken):  # BREAK at end of type region
                my_return_string += nextToken.str
                if nextToken.Id == variable.typeEndTokenId:
                    hasNext = False;
                nextToken = nextToken.next
        return my_return_string

    #    - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
    #       old method - matches variable names by string myVar.linear.x  matched on 'linear'
    #    - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 



    def initialize_list_of_vetted_units(self):
        ''' INITIALIZES A LIST OF UNIT DICTIONARIES USED TO TEST UNIT SMELL
            input: none
            output:  none. modified a class level data structure as a side effect
            '''

        self.list_of_vetted_units = [ 
	    {'degree_360': 1},
	    {'degree_celsius': 1},
	    {'meter': 1.0},
	    {'meter': 1},
	    {'meter': 2.0},
	    {'meter': 2},
	    {'pascal': 1},
	    {'quaternion': 1},
	    {'quaternion': 2},
	    {'radian': -0.5, 'meter': 0.5},
	    {'radian': -1, 'meter': 1.0},
	    {'radian': -1, 'meter': 1},
	    {'radian': -1.0, 'meter': 1},
	    {'radian': -1},
	    {'radian': -2.0},
	    {'radian': -2},
	    {'radian': 0.5, 'meter': -0.5},
	    {'radian': 1, 'meter': -1.0},
	    {'radian': 1, 'meter': -1},
	    {'radian': 1.0},
	    {'radian': 1},
	    {'radian': 2.0},
	    {'radian': 2},
	    {'second': -1, 'meter': 1.0},
	    {'second': -1, 'meter': 1},
	    {'second': -1, 'radian': 1},
	    {'second': -1.0, 'meter': 1.0},
	    {'second': -1.0, 'radian': 1.0},
	    {'second': -1},
	    {'second': -1.0},
	    {'second': -2, 'kilogram': 1, 'meter': 1},
	    {'second': -2, 'kilogram': 1, 'meter': 2},
	    {'second': -2, 'meter': 1, 'kilogram': 1},
	    {'second': -2, 'meter': 1},
	    {'second': -2, 'meter': 2},
	    {'second': -2, 'meter': 2, 'kilogram': 1},
	    {'second': -2, 'radian': 1, 'kilogram': 1},
	    {'second': -2, 'radian': 1, 'meter': 1},
	    {'second': -2, 'radian': 1},
	    {'second': -2, 'radian': 2, 'kilogram': 1},
	    {'second': -2, 'radian': 2},
	    {'second': -2.0, 'kilogram': 1.0, 'meter': 1.0},
	    {'second': -2.0, 'kilogram': 1.0, 'meter': 2.0},
	    {'second': -2.0, 'meter': 2.0, 'kilogram': 1.0},
	    {'second': -2.0},
	    {'second': -2},
	    {'second': -4, 'meter': 2},
	    {'second': 1, 'meter': -1.0},
	    {'second': 1, 'meter': -1},
	    {'second': 1, 'meter': 1.0},
	    {'second': 1.0, 'meter': -1.0},
	    {'second': 1.0, 'radian': -1.0},
	    {'second': 1},
	    {'second': 2, 'radian': 2, 'meter': -2},
	    {'second': 2.0, 'meter': -1.0},
	    {'second': 2.0},
	    {'second': 2},
	    {'second': 4.0},
	    {'tesla': 1},
	    {'tesla': 2}
	     ]






