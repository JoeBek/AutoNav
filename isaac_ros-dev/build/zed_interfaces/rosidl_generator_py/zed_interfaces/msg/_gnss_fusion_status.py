# generated from rosidl_generator_py/resource/_idl.py.em
# with input from zed_interfaces:msg/GnssFusionStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GnssFusionStatus(type):
    """Metaclass of message 'GnssFusionStatus'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'OK': 0,
        'OFF': 1,
        'CALIBRATION_IN_PROGRESS': 2,
        'RECALIBRATION_IN_PROGRESS': 3,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('zed_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'zed_interfaces.msg.GnssFusionStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__gnss_fusion_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__gnss_fusion_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__gnss_fusion_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__gnss_fusion_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__gnss_fusion_status

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'OK': cls.__constants['OK'],
            'OFF': cls.__constants['OFF'],
            'CALIBRATION_IN_PROGRESS': cls.__constants['CALIBRATION_IN_PROGRESS'],
            'RECALIBRATION_IN_PROGRESS': cls.__constants['RECALIBRATION_IN_PROGRESS'],
        }

    @property
    def OK(self):
        """Message constant 'OK'."""
        return Metaclass_GnssFusionStatus.__constants['OK']

    @property
    def OFF(self):
        """Message constant 'OFF'."""
        return Metaclass_GnssFusionStatus.__constants['OFF']

    @property
    def CALIBRATION_IN_PROGRESS(self):
        """Message constant 'CALIBRATION_IN_PROGRESS'."""
        return Metaclass_GnssFusionStatus.__constants['CALIBRATION_IN_PROGRESS']

    @property
    def RECALIBRATION_IN_PROGRESS(self):
        """Message constant 'RECALIBRATION_IN_PROGRESS'."""
        return Metaclass_GnssFusionStatus.__constants['RECALIBRATION_IN_PROGRESS']


class GnssFusionStatus(metaclass=Metaclass_GnssFusionStatus):
    """
    Message class 'GnssFusionStatus'.

    Constants:
      OK
      OFF
      CALIBRATION_IN_PROGRESS
      RECALIBRATION_IN_PROGRESS
    """

    __slots__ = [
        '_gnss_fusion_status',
    ]

    _fields_and_field_types = {
        'gnss_fusion_status': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.gnss_fusion_status = kwargs.get('gnss_fusion_status', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.gnss_fusion_status != other.gnss_fusion_status:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def gnss_fusion_status(self):
        """Message field 'gnss_fusion_status'."""
        return self._gnss_fusion_status

    @gnss_fusion_status.setter
    def gnss_fusion_status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'gnss_fusion_status' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'gnss_fusion_status' field must be an unsigned integer in [0, 255]"
        self._gnss_fusion_status = value
