#!/usr/bin/env python3
"""Generate a ready-to-paste table-driven CRC-16 lookup table + function.

Produces the byte-wise (256-entry) equivalent of the bit-by-bit CRC-16-CCITT
used in projects/libs/packets/SIGMA2/SIGMA2.hpp. The defaults match that
function (poly=0x1021, init=0xFFFF, MSB-first / non-reflected).

Examples:
    uv run scripts/generate_crc_tables.py
    uv run scripts/generate_crc_tables.py --poly 0x1021 --init 0xFFFF
    uv run scripts/generate_crc_tables.py --poly 0x8005 --reflect
"""

import argparse


def build_table(poly: int, reflect: bool) -> list[int]:
    """Build the 256-entry CRC-16 lookup table for the given polynomial."""
    table = []
    for byte in range(256):
        if reflect:
            crc = byte
            for _ in range(8):
                crc = (crc >> 1) ^ poly if crc & 0x0001 else crc >> 1
        else:
            crc = byte << 8
            for _ in range(8):
                crc = ((crc << 1) ^ poly) if crc & 0x8000 else (crc << 1)
        table.append(crc & 0xFFFF)
    return table


def format_table(table: list[int], per_row: int = 8) -> str:
    """Format the table as a C++ array body, `per_row` entries per line."""
    lines = []
    for i in range(0, len(table), per_row):
        row = ", ".join(f"0x{v:04X}" for v in table[i:i + per_row])
        lines.append(f"    {row},")
    return "\n".join(lines)


def emit(poly: int, init: int, reflect: bool) -> str:
    """Return the full ready-to-paste C++ snippet."""
    table = build_table(poly, reflect)
    body = format_table(table)

    if reflect:
        update = (
            "            crc = (crc >> 8) ^ CRC16_TABLE[(crc ^ data[i]) & 0xFF];"
        )
    else:
        update = (
            "            crc = (crc << 8) ^ "
            "CRC16_TABLE[((crc >> 8) ^ data[i]) & 0xFF];"
        )

    return f"""\
    // CRC-16 lookup table (poly=0x{poly:04X}, init=0x{init:04X}, \
reflected={str(reflect).lower()})
    static constexpr uint16_t CRC16_TABLE[256] = {{
{body}
    }};

    inline uint16_t crc16(const uint8_t* data, size_t len) {{
        uint16_t crc = 0x{init:04X}; // Initial Value
        for (size_t i = 0; i < len; i++) {{
{update}
        }}
        return crc;
    }}"""


def parse_int(value: str) -> int:
    """Parse an int accepting decimal or 0x-prefixed hex."""
    return int(value, 0)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate a table-driven CRC-16 C++ snippet."
    )
    parser.add_argument(
        "--poly", type=parse_int, default=0x1021,
        help="Generator polynomial (default: 0x1021, CCITT)",
    )
    parser.add_argument(
        "--init", type=parse_int, default=0xFFFF,
        help="Initial CRC value (default: 0xFFFF)",
    )
    parser.add_argument(
        "--reflect", action="store_true",
        help="Use reflected (LSB-first) algorithm (default: MSB-first)",
    )
    args = parser.parse_args()

    print(emit(args.poly, args.init, args.reflect))


if __name__ == "__main__":
    main()
