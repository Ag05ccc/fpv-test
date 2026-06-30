import struct

from kenet.msp import (
    MSPCode,
    MSPConnection,
    TCPSocketSerial,
    is_tcp_endpoint,
    msp_checksum,
    msp_encode,
    parse_tcp_endpoint,
)


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


def test_get_api_version_parses_protocol_fields():
    conn = connection_with_response(response_frame(MSPCode.MSP_API_VERSION, bytes([0, 1, 45])))

    assert conn.get_api_version() == {
        "protocol_version": 0,
        "api_major": 1,
        "api_minor": 45,
    }


def test_tcp_endpoint_parsing():
    assert parse_tcp_endpoint("tcp://127.0.0.1:5761") == ("127.0.0.1", 5761)
    assert parse_tcp_endpoint("127.0.0.1:5761") == ("127.0.0.1", 5761)
    assert is_tcp_endpoint("/dev/ttyAMA0") is False
    assert is_tcp_endpoint("127.0.0.1:5761") is True


def test_tcp_socket_serial_behaves_like_serial(monkeypatch):
    calls = []

    class FakeSocket:
        def __init__(self):
            self.timeout = None
            self.sent = []
            self.read_data = bytearray(b"abc")
            self.closed = False

        def settimeout(self, timeout):
            self.timeout = timeout

        def sendall(self, data):
            self.sent.append(data)

        def recv(self, size):
            chunk = self.read_data[:size]
            del self.read_data[:size]
            return bytes(chunk)

        def close(self):
            self.closed = True

    fake_socket = FakeSocket()

    def fake_create_connection(address, timeout):
        calls.append((address, timeout))
        return fake_socket

    monkeypatch.setattr("kenet.msp.socket.create_connection", fake_create_connection)

    transport = TCPSocketSerial("127.0.0.1", 5761, timeout=0.25)
    transport.open()

    assert calls == [(("127.0.0.1", 5761), 0.25)]
    assert transport.is_open is True
    assert fake_socket.timeout == 0.25
    assert transport.write(b"hello") == 5
    assert fake_socket.sent == [b"hello"]
    assert transport.read(2) == b"ab"
    assert transport.read(2) == b"c"
    transport.close()
    assert transport.is_open is False
    assert fake_socket.closed is True


def test_connection_uses_tcp_transport_for_tcp_endpoint(monkeypatch):
    created = []

    class FakeTransport:
        is_open = True

        def __init__(self, host, port, timeout):
            created.append((host, port, timeout))

        def open(self):
            created.append("open")

        def close(self):
            pass

    monkeypatch.setattr("kenet.msp.TCPSocketSerial", FakeTransport)
    conn = MSPConnection(port="tcp://127.0.0.1:5761", timeout=0.4)

    conn.connect()

    assert created == [("127.0.0.1", 5761, 0.4), "open"]
