"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class aliengo_leg_data_lcmt(object):
    __slots__ = ["q", "qd", "p", "v", "tau_est", "timestamp_us", "id", "robot_id"]

    __typenames__ = ["float", "float", "float", "float", "float", "int64_t", "int64_t", "int64_t"]

    __dimensions__ = [[12], [12], [12], [12], [12], None, None, None]

    def __init__(self):
        self.q = [ 0.0 for dim0 in range(12) ]
        self.qd = [ 0.0 for dim0 in range(12) ]
        self.p = [ 0.0 for dim0 in range(12) ]
        self.v = [ 0.0 for dim0 in range(12) ]
        self.tau_est = [ 0.0 for dim0 in range(12) ]
        self.timestamp_us = 0
        self.id = 0
        self.robot_id = 0

    def encode(self):
        buf = BytesIO()
        buf.write(aliengo_leg_data_lcmt._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack('>12f', *self.q[:12]))
        buf.write(struct.pack('>12f', *self.qd[:12]))
        buf.write(struct.pack('>12f', *self.p[:12]))
        buf.write(struct.pack('>12f', *self.v[:12]))
        buf.write(struct.pack('>12f', *self.tau_est[:12]))
        buf.write(struct.pack(">qqq", self.timestamp_us, self.id, self.robot_id))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != aliengo_leg_data_lcmt._get_packed_fingerprint():
            raise ValueError("Decode error")
        return aliengo_leg_data_lcmt._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = aliengo_leg_data_lcmt()
        self.q = struct.unpack('>12f', buf.read(48))
        self.qd = struct.unpack('>12f', buf.read(48))
        self.p = struct.unpack('>12f', buf.read(48))
        self.v = struct.unpack('>12f', buf.read(48))
        self.tau_est = struct.unpack('>12f', buf.read(48))
        self.timestamp_us, self.id, self.robot_id = struct.unpack(">qqq", buf.read(24))
        return self
    _decode_one = staticmethod(_decode_one)

    def _get_hash_recursive(parents):
        if aliengo_leg_data_lcmt in parents: return 0
        tmphash = (0xa9a928b534bfc487) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if aliengo_leg_data_lcmt._packed_fingerprint is None:
            aliengo_leg_data_lcmt._packed_fingerprint = struct.pack(">Q", aliengo_leg_data_lcmt._get_hash_recursive([]))
        return aliengo_leg_data_lcmt._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", aliengo_leg_data_lcmt._get_packed_fingerprint())[0]

