"""Length-prefixed JSON + NumPy protocol for localhost inference."""

import json
import struct

import numpy as np


VERSION = 1
_LENGTH = struct.Struct("!I")
MAX_HEADER = 1024*1024
MAX_ARRAY = 64*1024*1024


def _receive_exact(sock, size):
    parts = []
    while size:
        part = sock.recv(size)
        if not part:
            raise ConnectionError("inference socket closed")
        parts.append(part)
        size -= len(part)
    return b"".join(parts)


def send_message(sock, payload, arrays=None):
    document = dict(payload)
    document["protocol_version"] = VERSION
    metadata, buffers = [], []
    for name, value in (arrays or {}).items():
        array = np.ascontiguousarray(value)
        if array.nbytes > MAX_ARRAY:
            raise ValueError("array %s is too large" % name)
        metadata.append({"name": str(name), "dtype": array.dtype.str,
                         "shape": list(array.shape), "nbytes": array.nbytes})
        buffers.append(memoryview(array).cast("B"))
    document["arrays"] = metadata
    header = json.dumps(document, separators=(",", ":")).encode("utf-8")
    if len(header) > MAX_HEADER:
        raise ValueError("message header is too large")
    sock.sendall(_LENGTH.pack(len(header)))
    sock.sendall(header)
    for buffer in buffers:
        sock.sendall(buffer)


def receive_message(sock):
    size = _LENGTH.unpack(_receive_exact(sock, _LENGTH.size))[0]
    if size <= 0 or size > MAX_HEADER:
        raise ValueError("invalid inference header size")
    document = json.loads(_receive_exact(sock, size).decode("utf-8"))
    if document.get("protocol_version") != VERSION:
        raise ValueError("inference protocol version mismatch")
    arrays = {}
    for meta in document.pop("arrays", []):
        dtype = np.dtype(meta["dtype"])
        shape = tuple(map(int, meta["shape"]))
        nbytes = int(meta["nbytes"])
        if nbytes != int(np.prod(shape))*dtype.itemsize or nbytes > MAX_ARRAY:
            raise ValueError("invalid array metadata")
        arrays[str(meta["name"])] = np.frombuffer(
            _receive_exact(sock, nbytes), dtype=dtype).copy().reshape(shape)
    return document, arrays
