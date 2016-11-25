class UnitErrorTypes:
    """ DEFINES TYPES OF UNIT ERRORS
    """
    VARIABLE_MULTIPLE_UNITS = 0
    COMPARISON_INCOMPATIBLE_UNITS = 1
    VARIABLE_BECOME_UNITLESS = 2
    FUNCTION_CALLED_WITH_DIFFERENT_UNIT_ARGUMENTS = 3
    VARIABLE_WITH_UNUSUAL_UNITS = 4
    ADDITION_OF_INCOMPATIBLE_UNITS = 5
    LOGICAL_OPERATOR_USED_ON_UNITS = 6
    UNIT_SMELL = 7

    SEVERITY = [                    # MAPPED DIRECTLY FROM ROS VERBOSITY LEVELS
                    'DEBUG',
                    'INFO',
                    'WARN',
                    'ERROR',
                    'FATAL',
                   ]
    ERR_TYPE_NAMES = [              # MUST BE SAME INDEX AS ERRORS LISTED ABOVE
                        'MULTIPLE UNITS FOR VARIABLE',
                        'COMPARISON OF INCOMPATIBLE UNITS',
                        'VARIABLE BECAME UNITLESS',
                        'FUNCTION CALLED WITH DIFFERENT UNIT ARGUMENTS',
                        'VARIABLE WITH UNUSUAL UNITS',
                        'ADDITION OF INCOMPATIBLE UNITS',
                        'LOGICAL OPERATOR USED ON UNITS',
                        'UNIT SMELL',
                        ]

    def __init__(self):
        pass

    def get_err_short_discription(self, num):
        return self.ERR_TYPE_NAMES[num]

s = '''
VARIABLE MULTIPLE UNITS
COMPARISON INCOMPATIBLE UNITS
VARIABLE BECOME UNITLESS
FUNCTION CALLED WITH DIFFERENT UNIT ARGUMENTS
VARIABLE WITH UNUSUAL UNITS
ADDITION OF INCOMPATIBLE UNITS
LOGICAL OPERATOR USED ON UNITS
UNIT_SMELL
'''
