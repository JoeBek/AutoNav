# generated from rosidl_generator_py/resource/_idl.py.em
# with input from more_interfaces:msg/AddressBook.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_AddressBook(type):
    """Metaclass of message 'AddressBook'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'PHONE_TYPE_HOME': 0,
        'PHONE_TYPE_WORK': 1,
        'PHONE_TYPE_MOBILE': 2,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('more_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'more_interfaces.msg.AddressBook')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__address_book
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__address_book
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__address_book
            cls._TYPE_SUPPORT = module.type_support_msg__msg__address_book
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__address_book

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'PHONE_TYPE_HOME': cls.__constants['PHONE_TYPE_HOME'],
            'PHONE_TYPE_WORK': cls.__constants['PHONE_TYPE_WORK'],
            'PHONE_TYPE_MOBILE': cls.__constants['PHONE_TYPE_MOBILE'],
        }

    @property
    def PHONE_TYPE_HOME(self):
        """Message constant 'PHONE_TYPE_HOME'."""
        return Metaclass_AddressBook.__constants['PHONE_TYPE_HOME']

    @property
    def PHONE_TYPE_WORK(self):
        """Message constant 'PHONE_TYPE_WORK'."""
        return Metaclass_AddressBook.__constants['PHONE_TYPE_WORK']

    @property
    def PHONE_TYPE_MOBILE(self):
        """Message constant 'PHONE_TYPE_MOBILE'."""
        return Metaclass_AddressBook.__constants['PHONE_TYPE_MOBILE']


class AddressBook(metaclass=Metaclass_AddressBook):
    """
    Message class 'AddressBook'.

    Constants:
      PHONE_TYPE_HOME
      PHONE_TYPE_WORK
      PHONE_TYPE_MOBILE
    """

    __slots__ = [
        '_first_name',
        '_last_name',
        '_phone_number',
        '_phone_type',
    ]

    _fields_and_field_types = {
        'first_name': 'string',
        'last_name': 'string',
        'phone_number': 'string',
        'phone_type': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.first_name = kwargs.get('first_name', str())
        self.last_name = kwargs.get('last_name', str())
        self.phone_number = kwargs.get('phone_number', str())
        self.phone_type = kwargs.get('phone_type', int())

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
        if self.first_name != other.first_name:
            return False
        if self.last_name != other.last_name:
            return False
        if self.phone_number != other.phone_number:
            return False
        if self.phone_type != other.phone_type:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def first_name(self):
        """Message field 'first_name'."""
        return self._first_name

    @first_name.setter
    def first_name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'first_name' field must be of type 'str'"
        self._first_name = value

    @builtins.property
    def last_name(self):
        """Message field 'last_name'."""
        return self._last_name

    @last_name.setter
    def last_name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'last_name' field must be of type 'str'"
        self._last_name = value

    @builtins.property
    def phone_number(self):
        """Message field 'phone_number'."""
        return self._phone_number

    @phone_number.setter
    def phone_number(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'phone_number' field must be of type 'str'"
        self._phone_number = value

    @builtins.property
    def phone_type(self):
        """Message field 'phone_type'."""
        return self._phone_type

    @phone_type.setter
    def phone_type(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'phone_type' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'phone_type' field must be an unsigned integer in [0, 255]"
        self._phone_type = value
