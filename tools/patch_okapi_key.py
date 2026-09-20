#!/usr/bin/env python3
import argparse
import hashlib
import struct
from pathlib import Path

MARKER = b"__OKAPI_CONSUMER_KEY_PLACEHOLDER_0000000000000000__"

def main():
    ap = argparse.ArgumentParser(description="Patch OKAPI consumer key into T-Embed ESP32 firmware")
    ap.add_argument("input", type=Path)
    ap.add_argument("output", type=Path)
    ap.add_argument("consumer_key")
    args = ap.parse_args()

    key = args.consumer_key.encode("ascii")
    if not key or len(key) > len(MARKER):
        raise SystemExit(f"Consumer key must be 1..{len(MARKER)} ASCII bytes")

    data = bytearray(args.input.read_bytes())
    if data.count(MARKER) != 1:
        raise SystemExit("Expected exactly one OKAPI placeholder in firmware")

    pos = data.find(MARKER)
    data[pos:pos + len(MARKER)] = key + b"\x00" * (len(MARKER) - len(key))

    if len(data) < 24 or data[0] != 0xE9:
        raise SystemExit("Not an ESP32 app image")

    seg_count = data[1]
    off = 24
    segments = []
    for _ in range(seg_count):
        if off + 8 > len(data):
            raise SystemExit("Invalid ESP32 segment table")
        _, length = struct.unpack_from("<II", data, off)
        off += 8
        start = off
        off += length
        if off > len(data):
            raise SystemExit("Invalid ESP32 segment length")
        segments.append((start, length))

    checksum_pos = ((off + 15) // 16) * 16 - 1
    if checksum_pos >= len(data):
        raise SystemExit("Invalid checksum location")

    checksum = 0xEF
    for start, length in segments:
        for b in data[start:start + length]:
            checksum ^= b
    data[checksum_pos] = checksum

    if data[23] == 1:
        digest = hashlib.sha256(bytes(data[:checksum_pos + 1])).digest()
        end = checksum_pos + 1 + len(digest)
        if end > len(data):
            raise SystemExit("Missing appended ESP32 image hash")
        data[checksum_pos + 1:end] = digest

    args.output.write_bytes(data)
    print("Patched:", args.output)
    print("SHA256:", hashlib.sha256(data).hexdigest())

if __name__ == "__main__":
    main()
