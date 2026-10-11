#!/usr/bin/env python3
"""xwd2png.py — 把 xwd 截图转成 PNG（本地调试用）。

用法: python3 xwd2png.py input.xwd output.png
自动解析 XWD header (big/little-endian, ZPixmap 24/32bpp)。
"""
import struct
import sys

from PIL import Image


def decode_xwd(path):
    d = open(path, "rb").read()
    # detect byte order: header_size sane value (~100+) decides
    for endian in (">", "<"):
        hsize = struct.unpack(endian + "I", d[0:4])[0]
        if 90 <= hsize <= 1000:
            break
    else:
        raise ValueError("cannot detect endian")
    g = lambda o: struct.unpack(endian + "I", d[o : o + 4])[0]
    hsize = g(0)
    fmt = g(8)
    depth = g(12)
    w = g(16)
    h = g(20)
    bpp = g(44)
    bpl = g(48)
    vclass = g(52)
    if fmt != 2:  # ZPixmap
        raise ValueError(f"unsupported pixmap format {fmt}")
    off = hsize
    # Prefer bytes-per-line / width to infer real pixel stride (X server often
    # stores depth-24 as 4 bytes/pixel with bpp field still reporting 24).
    px_bytes_per = 0
    if bpl > 0 and w > 0 and bpl % w == 0:
        px_bytes_per = bpl // w
    if px_bytes_per not in (3, 4):
        px_bytes_per = max(1, (bpp + 7) // 8)
    # reconstruct pixel strides; scanline padded to bpl
    # try common layouts by masks
    rmask = g(56)
    gmask = g(60)
    bmask = g(64)
    ncolors = g(76)
    # skip colormap entries if present (rare for TrueColor)
    # For TrueColor depth 24/32 we assume direct RGB in stored order.
    row_bytes = w * px_bytes_per
    if bpl < row_bytes:
        bpl = row_bytes
    # sample first pixel to decide channel order via masks on a 4-byte word
    # Handle both 3 and 4 bytes per pixel.
    rows = []
    if px_bytes_per == 4:
        # big-endian word layout guess [pad,R,G,B] or [B,G,R,pad]
        # Determine by mask location within the low 24 bits.
        # Use masks: find highest set bit index to know R channel byte
        def chan_off(mask):
            # mask e.g. 0x00ff0000 -> offset 16 bits from LSB
            if mask == 0:
                return None
            return (mask.bit_length() - 1) - (mask & -mask).bit_length() + 1

        ro = chan_off(rmask)
        go = chan_off(gmask)
        bo = chan_off(bmask)

        def byte_of(word, bitshift):
            # word is int of 4 bytes assembled per machine order
            return (word >> bitshift) & 0xFF

        for y in range(h):
            line = d[off + y * bpl : off + y * bpl + row_bytes]
            out = bytearray()
            for x in range(w):
                # assemble pixel word in native order of the file
                word = 0
                if endian == ">":
                    for b in line[x * 4 : x * 4 + 4]:
                        word = (word << 8) | b
                else:
                    for b in reversed(line[x * 4 : x * 4 + 4]):
                        word = (word << 8) | b
                r = (word >> ro) & 0xFF if ro is not None else 0
                gg = (word >> go) & 0xFF if go is not None else 0
                b = (word >> bo) & 0xFF if bo is not None else 0
                out += bytes((r, gg, b))
            rows.append(bytes(out))
    else:
        # 3 bytes/pixel typical RGB or BGR
        for y in range(h):
            line = d[off + y * bpl : off + y * bpl + row_bytes]
            rows.append(line)
    im = Image.frombytes("RGB", (w, h), b"".join(rows))
    return im


if __name__ == "__main__":
    src, dst = sys.argv[1], sys.argv[2]
    img = decode_xwd(src)
    img.save(dst)
    print(f"saved {dst} {img.size}")
