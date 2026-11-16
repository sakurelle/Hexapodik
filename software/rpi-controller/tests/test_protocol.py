import unittest

from hexapod.protocol import (
    MessageType,
    Packet,
    ProtocolError,
    StreamParser,
    decode_packet,
    encode_packet,
    pwm_packet,
)


class ProtocolTests(unittest.TestCase):
    def test_firmware_known_frames(self):
        self.assertEqual(
            encode_packet(Packet(MessageType.DISARM, 1)).hex(" ").upper(),
            "04 01 03 01 01 01 01 03 1E 2D 00",
        )
        self.assertEqual(
            encode_packet(Packet(MessageType.GET_STATUS, 2)).hex(" ").upper(),
            "04 01 04 02 01 01 01 03 8D 0B 00",
        )
        expected_arm = (
            "04 01 01 03 01 01 28 24 "
            + "DC 05 " * 18
            + "83 AE 00"
        ).strip()
        self.assertEqual(encode_packet(pwm_packet(MessageType.ARM, 3, (1500,) * 18)).hex(" ").upper(), expected_arm)

    def test_round_trip_and_crc_rejection(self):
        source = pwm_packet(MessageType.SET_PWM, 0x12345678, tuple(range(1400, 1418)))
        frame = encode_packet(source)
        self.assertEqual(decode_packet(frame), source)
        corrupt = bytearray(frame)
        corrupt[8] ^= 0x40
        with self.assertRaises(ProtocolError):
            decode_packet(bytes(corrupt))

    def test_stream_recovers_after_bad_frame(self):
        parser = StreamParser()
        valid = encode_packet(Packet(MessageType.GET_STATUS, 9))
        bad = bytearray(valid)
        bad[-2] ^= 1
        packets = parser.push(bytes(bad) + valid)
        self.assertEqual(packets, [Packet(MessageType.GET_STATUS, 9)])
        self.assertEqual(parser.dropped_frames, 1)


if __name__ == "__main__":
    unittest.main()
