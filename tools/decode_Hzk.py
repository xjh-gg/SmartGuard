# 从 oledfont.h 手动复制 Hzk[17] 的 32 字节（"中"）
data = [0x00,0x00,0x02,0x02,0x7F,0x42,0x42,0x42,0x42,0x42,0x7F,0x42,0x02,0x02,0x02,0x02,
        0x00,0x00,0x00,0x00,0xF0,0x10,0x10,0x10,0x10,0x10,0xF0,0x10,0x00,0x00,0x00,0x00]

def render(name, grid):
    print(f"\n===== {name} =====")
    for row in grid:
        print("".join("#" if b else "." for b in row))

# 格式 1：横向取模，每行2字节，MSB在左
def fmt1(d):
    g = []
    for row in range(16):
        line = []
        for col in range(16):
            if col < 8:
                b = (d[row*2] >> (7-col)) & 1
            else:
                b = (d[row*2+1] >> (15-col)) & 1
            line.append(b)
        g.append(line)
    return g

# 格式 2：横向取模，每行2字节，LSB在左
def fmt2(d):
    g = []
    for row in range(16):
        line = []
        for col in range(16):
            if col < 8:
                b = (d[row*2] >> col) & 1
            else:
                b = (d[row*2+1] >> (col-8)) & 1
            line.append(b)
        g.append(line)
    return g

# 格式 3：纵向取模，每列2字节，上半在前
def fmt3(d):
    g = [[0]*16 for _ in range(16)]
    for col in range(16):
        for row in range(8):
            g[row][col] = (d[col*2] >> row) & 1
            g[row+8][col] = (d[col*2+1] >> row) & 1
    return g

# 格式 4：纵向取模，每列2字节，上半在后
def fmt4(d):
    g = [[0]*16 for _ in range(16)]
    for col in range(16):
        for row in range(8):
            g[row][col] = (d[col*2+1] >> row) & 1
            g[row+8][col] = (d[col*2] >> row) & 1
    return g

# 格式 5：脚本当前假设的（上半左、下半左、上半右、下半右）
def fmt5(d):
    g = [[0]*16 for _ in range(16)]
    for row in range(8):
        for col in range(8):
            g[row][col]       = (d[row]      >> (7-col)) & 1
            g[row+8][col]     = (d[8+row]    >> (7-col)) & 1
            g[row][col+8]     = (d[16+row]   >> (7-col)) & 1
            g[row+8][col+8]   = (d[24+row]   >> (7-col)) & 1
    return g

# 格式 6：OLED page 模式（每字节一列）
def fmt6(d):
    g = [[0]*16 for _ in range(16)]
    for page in range(2):
        for col in range(16):
            b = d[page*16 + col]
            for k in range(8):
                g[page*8+k][col] = (b >> k) & 1
    return g

# 格式 7：page 模式但 LSB 在上
def fmt7(d):
    g = [[0]*16 for _ in range(16)]
    for page in range(2):
        for col in range(16):
            b = d[page*16 + col]
            for k in range(8):
                g[page*8+k][col] = (b >> (7-k)) & 1
    return g

# 格式 8：横向取模，但字节序反（右字节在前）
def fmt8(d):
    g = []
    for row in range(16):
        line = []
        for col in range(16):
            if col < 8:
                b = (d[row*2+1] >> (7-col)) & 1
            else:
                b = (d[row*2] >> (15-col)) & 1
            line.append(b)
        g.append(line)
    return g

for i, fn in enumerate([fmt1,fmt2,fmt3,fmt4,fmt5,fmt6,fmt7,fmt8], 1):
    render(f"格式{i}", fn(data))