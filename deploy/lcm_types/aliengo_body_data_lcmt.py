"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class aliengo_body_data_lcmt(object):
    __slots__ = ["p", "vWorld", "vBody", "rpy", "omegaBody", "omegaWorld", "quat", "contact_estimate", "aBody", "aWorld", "timestamp_us", "id", "robot_id"]

    __typenames__ = ["float", "float", "float", "float", "float", "float", "float", "float", "float", "float", "int64_t", "int64_t", "int64_t"]

    __dimensions__ = [[3], [3], [3], [3], [3], [3], [4], [4], [3], [3], None, None, None]

    def __init__(self):
        self.p = [ 0.0 for dim0 in range(3) ]
        self.vWorld = [ 0.0 for dim0 in range(3) ]
        self.vBody = [ 0.0 for dim0 in range(3) ]
        self.rpy = [ 0.0 for dim0 in range(3) ]
        self.omegaBody = [ 0.0 for dim0 in range(3) ]
        self.omegaWorld = [ 0.0 for dim0 in range(3) ]
        self.quat = [ 0.0 for dim0 in range(4) ]
        self.contact_estimate = [ 0.0 for dim0 in range(4) ]
        self.aBody = [ 0.0 for dim0 in range(3) ]
        self.aWorld = [ 0.0 for dim0 in range(3) ]
        self.timestamp_us = 0
        self.id = 0
        self.robot_id = 0

    def encode(self):
        buf = BytesIO()
        buf.write(aliengo_body_data_lcmt._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack('>3f', *self.p[:3]))
        buf.write(struct.pack('>3f', *self.vWorld[:3]))
        buf.write(struct.pack('>3f', *self.vBody[:3]))
        buf.write(struct.pack('>3f', *self.rpy[:3]))
        buf.write(struct.pack('>3f', *self.omegaBody[:3]))
        buf.write(struct.pack('>3f', *self.omegaWorld[:3]))
        buf.write(struct.pack('>4f', *self.quat[:4]))
        buf.write(struct.pack('>4f', *self.contact_estimate[:4]))
        buf.write(struct.pack('>3f', *self.aBody[:3]))
        buf.write(struct.pack('>3f', *self.aWorld[:3]))
        buf.write(struct.pack(">qqq", self.timestamp_us, self.id, self.robot_id))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != aliengo_body_data_lcmt._get_packed_fingerprint():
            raise ValueError("Decode error")
        return aliengo_body_data_lcmt._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = aliengo_body_data_lcmt()
        self.p = struct.unpack('>3f', buf.read(12))
        self.vWorld = struct.unpack('>3f', buf.read(12))
        self.vBody = struct.unpack('>3f', buf.read(12))
        self.rpy = struct.unpack('>3f', buf.read(12))
        self.omegaBody = struct.unpack('>3f', buf.read(12))
        self.omegaWorld = struct.unpack('>3f', buf.read(12))
        self.quat = struct.unpack('>4f', buf.read(16))
        self.contact_estimate = struct.unpack('>4f', buf.read(16))
        self.aBody = struct.unpack('>3f', buf.read(12))
        self.aWorld = struct.unpack('>3f', buf.read(12))
        self.timestamp_us, self.id, self.robot_id = struct.unpack(">qqq", buf.read(24))
        return self
    _decode_one = staticmethod(_decode_one)

    def _get_hash_recursive(parents):
        if aliengo_body_data_lcmt in parents: return 0
        tmphash = (0xea87c8282effe5b6) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if aliengo_body_data_lcmt._packed_fingerprint is None:
            aliengo_body_data_lcmt._packed_fingerprint = struct.pack(">Q", aliengo_body_data_lcmt._get_hash_recursive([]))
        return aliengo_body_data_lcmt._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", aliengo_body_data_lcmt._get_packed_fingerprint())[0]

