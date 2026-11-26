import math
import struct
import unittest

from ihm.protocol import FIELD_POSES, TWIST_CMD, decode_field_poses, encode_twist


class ProtocolTests(unittest.TestCase):
    def test_protocol_sizes_match_packed_c_structs(self):
        self.assertEqual(FIELD_POSES.size, 40)
        self.assertEqual(TWIST_CMD.size, 8)

    def test_decodes_field_packet(self):
        values = [8, -14]
        for robot_id in range(4):
            values.extend(
                (robot_id, robot_id * 10, -robot_id * 5, robot_id * 0.25)
            )
        field = decode_field_poses(FIELD_POSES.pack(*values))
        self.assertEqual(field.ball.x, 8)
        self.assertEqual(field.ball.y, -14)
        self.assertEqual(field.robots[3].robot_id, 3)
        self.assertEqual(field.robots[3].position.x, 30)
        self.assertAlmostEqual(field.robots[3].angle, 0.75)

    def test_rejects_wrong_packet_size(self):
        with self.assertRaises(ValueError):
            decode_field_poses(b"short")

    def test_encodes_stop_command(self):
        self.assertEqual(
            struct.unpack("<ff", encode_twist(0.0, 0.0)), (0.0, 0.0)
        )

    def test_rejects_non_finite_command(self):
        with self.assertRaises(ValueError):
            encode_twist(math.nan, 0.0)


if __name__ == "__main__":
    unittest.main()
