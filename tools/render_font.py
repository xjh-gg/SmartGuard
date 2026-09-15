# -*- coding: utf-8 -*-
# 渲染 Hzk[0] ("中") 和 Hzk[7] ("光") 的三种格式

# 从 oledfont.h 提取的原始字节
hzk0 = [0x00,0x00,0xF0,0x10,0x10,0x10,0x10,0xFF,0x10,0x10,0x10,0x10,0xF0,0x00,0x00,0x00,
        0x00,0x00,0x0F,0x04,0x04,0x04,0x04,0xFF,0x04,0x04,0x04,0x04,0x0F,0x00,0x00,0x00]
hzk7 = [0x00,0x00,0x02,0x42,0x22,0x12,0x02,0xFF,0x08,0x08,0x08,0x08,0x10,0x10,0x20,0x40,
        0x00,0x00,0x00,0x10,0x20,0x40,0x00,0xF8,0x80,0x80,0x80,0x80,0x80,0x88,0x88,0x78]

def render_row_major(data):
    """格式 1：行主序 16x16
    每行 2 字节 (data[r*2] + data[r*2+1])
    共 16 行
    每字节 bit7=第 0 列, bit0=第 7 列
    第二字节同理 bit7=第 8 列, bit0=第 15 列
    """
    grid = [['.' for _ in range(16)] for _ in range(16)]
    for row in range(16):
        b1 = data[row * 2]       # 左 8 列
        b2 = data[row * 2 + 1]   # 右 8 列
        for col in range(8):
            if b1 & (0x80 >> col):
                grid[row][col] = '#'
            if b2 & (0x80 >> col):
                grid[row][col + 8] = '#'
    return grid

def render_col_major(data):
    """格式 2：列主序 16x16
    每列 2 字节 (data[c*2] + data[c*2+1])
    共 16 列
    每字节 bit0=第 0 行, bit7=第 7 行  (LSB=顶)
    """
    grid = [['.' for _ in range(16)] for _ in range(16)]
    for col in range(16):
        b1 = data[col * 2]       # 上 8 行
        b2 = data[col * 2 + 1]   # 下 8 行
        for row in range(8):
            if b1 & (1 << row):
                grid[row][col] = '#'
            if b2 & (1 << row):
                grid[row + 8][col] = '#'
    return grid

def render_ssd1306_page(data):
    """格式 3：SSD1306 Page 友好
    data[0..15]  = page 0 的 16 列 (上半页, 行 0-7)
    data[16..31] = page 1 的 16 列 (下半页, 行 8-15)
    每字节 bit0=该列的顶行像素, bit7=底行像素
    """
    grid = [['.' for _ in range(16)] for _ in range(16)]
    for col in range(16):
        b_top = data[col]          # 上半页
        b_bot = data[16 + col]     # 下半页
        for row in range(8):
            if b_top & (1 << row):
                grid[row][col] = '#'
            if b_bot & (1 << row):
                grid[row + 8][col] = '#'
    return grid

def render_ssd1306_page_msb(data):
    """格式 4：SSD1306 Page 但 bit7=顶行 (MSB=顶)
    有些取模工具的字库是这种格式
    """
    grid = [['.' for _ in range(16)] for _ in range(16)]
    for col in range(16):
        b_top = data[col]
        b_bot = data[16 + col]
        for row in range(8):
            if b_top & (0x80 >> row):
                grid[row][col] = '#'
            if b_bot & (0x80 >> row):
                grid[row + 8][col] = '#'
    return grid

def render_firmware_format(data):
    """格式 5：厂商原始格式（无转置直接写）
    厂商字库是 [N*2][16] 结构
    page0 = data[0..15], 直接作为 page0 的 16 列
    page1 = data[16..31], 直接作为 page1 的 16 列
    每字节：bit0=该列顶行, bit7=该列底行 (SSD1306 原生)
    """
    grid = [['.' for _ in range(16)] for _ in range(16)]
    # page 0: data[0..15] 直接每字节 = 一列, bit0=top
    for col in range(16):
        b = data[col]
        for row in range(8):
            if b & (1 << row):
                grid[row][col] = '#'
    # page 1: data[16..31]
    for col in range(16):
        b = data[16 + col]
        for row in range(8):
            if b & (1 << row):
                grid[row + 8][col] = '#'
    return grid

def print_grid(grid, label):
    print(f"\n{'='*20} {label} {'='*20}")
    for row in grid:
        print(''.join(row))

# ====== 渲染 ======
for name, data in [("Hzk[0] (中)", hzk0), ("Hzk[7] (光)", hzk7)]:
    print(f"\n{'#'*60}")
    print(f"# {name}")
    print(f"{'#'*60}")
    print(f"\n原始字节: {' '.join(f'0x{b:02X}' for b in data)}")
    
    print_grid(render_row_major(data), "行主序 16x16 (每行2字节, MSB=左列)")
    print_grid(render_col_major(data), "列主序 16x16 (每列2字节, LSB=顶行)")
    print_grid(render_ssd1306_page(data), "SSD1306 Page 友好 (每字节=一列, bit0=顶行)")
    print_grid(render_ssd1306_page_msb(data), "SSD1306 Page 友好(MSB=顶行)")
    print_grid(render_firmware_format(data), "厂商原生格式 (直接写SSD1306)")
