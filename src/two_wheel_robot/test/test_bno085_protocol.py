import pytest

from two_wheel_robot.bno085_protocol import ImuSample, PacketParser, encode_packet


def make_sample(seq):
    return ImuSample(seq, 1_000_000 + seq * 4000, 0.1, -0.2, 9.81, 0.01, 0.02, -0.03, 3, 2)


def test_decodes_packets_split_across_reads_with_boot_text():
    data = b"BNO085 SPI test\r\nBNO08x Found!\r\n" + b"".join(
        encode_packet(make_sample(i)) for i in range(5))
    parser = PacketParser()
    samples = []
    for i in range(0, len(data), 7):
        samples += parser.feed(data[i:i + 7])
    assert [s.sequence for s in samples] == [0, 1, 2, 3, 4]
    assert samples[2].timestamp_us == 1_008_000
    assert samples[0].az == pytest.approx(9.81, abs=1e-5)
    assert (samples[0].accel_status, samples[0].gyro_status) == (3, 2)


def test_rejects_corrupted_packet_and_resyncs():
    good = encode_packet(make_sample(1))
    bad = bytearray(encode_packet(make_sample(0)))
    bad[20] ^= 0xFF
    parser = PacketParser()
    samples = parser.feed(bytes(bad) + good)
    assert [s.sequence for s in samples] == [1]
    assert parser.crc_errors >= 1
