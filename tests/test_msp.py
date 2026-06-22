import struct

from kenet.msp import MSPCode, MSPConnection, msp_checksum, msp_encode


class FakeSerial:
    def __init__(self, read_data=b""):
        self.read_data = bytearray(read_data)
        self.writes = []
        self.is_open = True

    def read(self, size):
        if not self.read_data:
            return b""
        chunk = self.read_data[:size]
        del self.read_data[:size]
        return bytes(chunk)

    def write(self, data):
        self.writes.append(data)


def response_frame(code, payload=b"", direction=b">"):
    return (
        b"$M" + direction +
        bytes([len(payload), int(code)]) +
        payload +
        bytes([msp_checksum(code, payload)])
    )


def connection_with_response(data):
    conn = MSPConnection()
    conn._serial = FakeSerial(data)
    return conn


def test_msp_encode_request_checksum():
    assert msp_encode(MSPCode.MSP_RC) == bytes.fromhex("244d3c006969")


def test_request_reads_matching_response_payload():
    payload = struct.pack("<4H", 1500, 1501, 1502, 1503)
    conn = connection_with_response(response_frame(MSPCode.MSP_RC, payload))

    assert conn.request(MSPCode.MSP_RC) == payload
    assert conn._serial.writes == [msp_encode(MSPCode.MSP_RC)]


def test_request_ignores_stale_response_code():
    stale = response_frame(MSPCode.MSP_ATTITUDE, b"\x00" * 6)
    payload = struct.pack("<2H", 1000, 2000)
    conn = connection_with_response(stale + response_frame(MSPCode.MSP_RC, payload))

    assert conn.request(MSPCode.MSP_RC) == payload


def test_request_rejects_bad_checksum():
    frame = bytearray(response_frame(MSPCode.MSP_RC, b"\x01\x02"))
    frame[-1] ^= 0xFF
    conn = connection_with_response(bytes(frame))

    assert conn.request(MSPCode.MSP_RC) is None


def test_request_handles_msp_error_frame():
    conn = connection_with_response(response_frame(MSPCode.MSP_RC, direction=b"!"))

    assert conn.request(MSPCode.MSP_RC) is None
