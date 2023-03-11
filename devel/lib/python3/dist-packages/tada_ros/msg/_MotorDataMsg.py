# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from tada_ros/MotorDataMsg.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class MotorDataMsg(genpy.Message):
  _md5sum = "0122060c184091d374c522c731386ece"
  _type = "tada_ros/MotorDataMsg"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """int32 mode
int32 duration
int32 motor1_move
int32 motor2_move
int32 motor1_torque
int32 motor2_torque
float32 PF
float32 EV
float32 t
"""
  __slots__ = ['mode','duration','motor1_move','motor2_move','motor1_torque','motor2_torque','PF','EV','t']
  _slot_types = ['int32','int32','int32','int32','int32','int32','float32','float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       mode,duration,motor1_move,motor2_move,motor1_torque,motor2_torque,PF,EV,t

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(MotorDataMsg, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.mode is None:
        self.mode = 0
      if self.duration is None:
        self.duration = 0
      if self.motor1_move is None:
        self.motor1_move = 0
      if self.motor2_move is None:
        self.motor2_move = 0
      if self.motor1_torque is None:
        self.motor1_torque = 0
      if self.motor2_torque is None:
        self.motor2_torque = 0
      if self.PF is None:
        self.PF = 0.
      if self.EV is None:
        self.EV = 0.
      if self.t is None:
        self.t = 0.
    else:
      self.mode = 0
      self.duration = 0
      self.motor1_move = 0
      self.motor2_move = 0
      self.motor1_torque = 0
      self.motor2_torque = 0
      self.PF = 0.
      self.EV = 0.
      self.t = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_6i3f().pack(_x.mode, _x.duration, _x.motor1_move, _x.motor2_move, _x.motor1_torque, _x.motor2_torque, _x.PF, _x.EV, _x.t))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 36
      (_x.mode, _x.duration, _x.motor1_move, _x.motor2_move, _x.motor1_torque, _x.motor2_torque, _x.PF, _x.EV, _x.t,) = _get_struct_6i3f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_6i3f().pack(_x.mode, _x.duration, _x.motor1_move, _x.motor2_move, _x.motor1_torque, _x.motor2_torque, _x.PF, _x.EV, _x.t))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 36
      (_x.mode, _x.duration, _x.motor1_move, _x.motor2_move, _x.motor1_torque, _x.motor2_torque, _x.PF, _x.EV, _x.t,) = _get_struct_6i3f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_6i3f = None
def _get_struct_6i3f():
    global _struct_6i3f
    if _struct_6i3f is None:
        _struct_6i3f = struct.Struct("<6i3f")
    return _struct_6i3f
