"""Generate 16x16 Chinese font data for SSD1306 OLED (Hzk[] row-major format).

Format (matching existing oledfont.h data):
  32 bytes per character:
    bytes 0-15:  LEFT half  (columns 0-7),  1 byte per row, bits 7-0 = cols 0-7
    bytes 16-31: RIGHT half (columns 8-15), 1 byte per row, bits 7-0 = cols 8-15

Display function must TRANSPOSE row-major font data → SSD1306 column-major page memory.
"""
from PIL import Image, ImageDraw, ImageFont
import sys

FONT_PATH = "C:/Windows/Fonts/simsun.ttc"
CHAR_SIZE = 14
Y_OFFSET = 1

chars = "光照温度湿度空气良好中差巡逻入火灾告警备拆前"

font = ImageFont.truetype(FONT_PATH, CHAR_SIZE)

for idx, ch in enumerate(chars):
    img = Image.new('L', (16, 16), 0)
    draw = ImageDraw.Draw(img)
    draw.text((0, Y_OFFSET), ch, fill=255, font=font)

    left = []
    right = []
    for row in range(16):
        lb = 0
        rb = 0
        for col in range(8):
            if img.getpixel((col, row)) >= 128:
                lb |= (0x80 >> col)
            if img.getpixel((8 + col, row)) >= 128:
                rb |= (0x80 >> col)
        left.append(lb)
        right.append(rb)

    left_hex = ','.join(f'0x{b:02X}' for b in left)
    right_hex = ','.join(f'0x{b:02X}' for b in right)
    print(f'{{{left_hex}}},')
    print(f'{{{right_hex}}},/*"{ch}",{idx}*/')
