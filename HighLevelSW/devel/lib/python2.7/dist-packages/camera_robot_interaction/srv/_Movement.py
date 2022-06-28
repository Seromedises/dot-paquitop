# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from camera_robot_interaction/MovementRequest.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class MovementRequest(genpy.Message):
  _md5sum = "5f15cf2b3ea3f6f2b16936c710d373fd"
  _type = "camera_robot_interaction/MovementRequest"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """float32 r11
float32 r12
float32 r13
float32 r14
float32 r21
float32 r22
float32 r23
float32 r24
float32 r31
float32 r32
float32 r33
float32 r34
float32 gripper
bool rotate
bool rest_pos
"""
  __slots__ = ['r11','r12','r13','r14','r21','r22','r23','r24','r31','r32','r33','r34','gripper','rotate','rest_pos']
  _slot_types = ['float32','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32','bool','bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       r11,r12,r13,r14,r21,r22,r23,r24,r31,r32,r33,r34,gripper,rotate,rest_pos

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(MovementRequest, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.r11 is None:
        self.r11 = 0.
      if self.r12 is None:
        self.r12 = 0.
      if self.r13 is None:
        self.r13 = 0.
      if self.r14 is None:
        self.r14 = 0.
      if self.r21 is None:
        self.r21 = 0.
      if self.r22 is None:
        self.r22 = 0.
      if self.r23 is None:
        self.r23 = 0.
      if self.r24 is None:
        self.r24 = 0.
      if self.r31 is None:
        self.r31 = 0.
      if self.r32 is None:
        self.r32 = 0.
      if self.r33 is None:
        self.r33 = 0.
      if self.r34 is None:
        self.r34 = 0.
      if self.gripper is None:
        self.gripper = 0.
      if self.rotate is None:
        self.rotate = False
      if self.rest_pos is None:
        self.rest_pos = False
    else:
      self.r11 = 0.
      self.r12 = 0.
      self.r13 = 0.
      self.r14 = 0.
      self.r21 = 0.
      self.r22 = 0.
      self.r23 = 0.
      self.r24 = 0.
      self.r31 = 0.
      self.r32 = 0.
      self.r33 = 0.
      self.r34 = 0.
      self.gripper = 0.
      self.rotate = False
      self.rest_pos = False

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
      buff.write(_get_struct_13f2B().pack(_x.r11, _x.r12, _x.r13, _x.r14, _x.r21, _x.r22, _x.r23, _x.r24, _x.r31, _x.r32, _x.r33, _x.r34, _x.gripper, _x.rotate, _x.rest_pos))
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
      end += 54
      (_x.r11, _x.r12, _x.r13, _x.r14, _x.r21, _x.r22, _x.r23, _x.r24, _x.r31, _x.r32, _x.r33, _x.r34, _x.gripper, _x.rotate, _x.rest_pos,) = _get_struct_13f2B().unpack(str[start:end])
      self.rotate = bool(self.rotate)
      self.rest_pos = bool(self.rest_pos)
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
      buff.write(_get_struct_13f2B().pack(_x.r11, _x.r12, _x.r13, _x.r14, _x.r21, _x.r22, _x.r23, _x.r24, _x.r31, _x.r32, _x.r33, _x.r34, _x.gripper, _x.rotate, _x.rest_pos))
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
      end += 54
      (_x.r11, _x.r12, _x.r13, _x.r14, _x.r21, _x.r22, _x.r23, _x.r24, _x.r31, _x.r32, _x.r33, _x.r34, _x.gripper, _x.rotate, _x.rest_pos,) = _get_struct_13f2B().unpack(str[start:end])
      self.rotate = bool(self.rotate)
      self.rest_pos = bool(self.rest_pos)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_13f2B = None
def _get_struct_13f2B():
    global _struct_13f2B
    if _struct_13f2B is None:
        _struct_13f2B = struct.Struct("<13f2B")
    return _struct_13f2B
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from camera_robot_interaction/MovementResponse.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class MovementResponse(genpy.Message):
  _md5sum = "0baab5e2da097063283a0661272a622e"
  _type = "camera_robot_interaction/MovementResponse"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """int64 Success
float32 gripper_fb

"""
  __slots__ = ['Success','gripper_fb']
  _slot_types = ['int64','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       Success,gripper_fb

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(MovementResponse, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.Success is None:
        self.Success = 0
      if self.gripper_fb is None:
        self.gripper_fb = 0.
    else:
      self.Success = 0
      self.gripper_fb = 0.

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
      buff.write(_get_struct_qf().pack(_x.Success, _x.gripper_fb))
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
      end += 12
      (_x.Success, _x.gripper_fb,) = _get_struct_qf().unpack(str[start:end])
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
      buff.write(_get_struct_qf().pack(_x.Success, _x.gripper_fb))
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
      end += 12
      (_x.Success, _x.gripper_fb,) = _get_struct_qf().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_qf = None
def _get_struct_qf():
    global _struct_qf
    if _struct_qf is None:
        _struct_qf = struct.Struct("<qf")
    return _struct_qf
class Movement(object):
  _type          = 'camera_robot_interaction/Movement'
  _md5sum = '4787e27e3721f1f2004cf17997b3e3ae'
  _request_class  = MovementRequest
  _response_class = MovementResponse