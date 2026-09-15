"""校验 oled.c 里的 HZ_* 宏是否指向 oledfont.h 中正确的字模。

OLED 中文「乱码」的一类根因：Hzk[] 里出现重复字（如「度」占两个槽位），
而 HZ_* 宏按字面顺序推算下标 → 屏幕上显示成别的汉字。本脚本把每个宏
注释里声明的汉字，与宏值指向的 Hzk 字模做位图比对，不一致即报错退出。

用法:
    python tools/check_oled_font.py            # 校验全部 HZ_* 宏
    python tools/check_oled_font.py --dump 13  # 渲染指定 Hzk 下标的字形
"""
import io
import re
import sys
from pathlib import Path

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

ROOT = Path(__file__).resolve().parent.parent
FONT_H = ROOT / 'Hardware' / 'OLED' / 'oledfont.h'
OLED_C = ROOT / 'Hardware' / 'OLED' / 'oled.c'
FONT_PATH = 'C:/Windows/Fonts/simsun.ttc'
CHAR_SIZE = 14
Y_OFFSET = 1
MAX_BITS_DIFF = 24


def read_text(path):
    raw = path.read_bytes()
    for enc in ('utf-8-sig', 'utf-8', 'gbk'):
        try:
            return raw.decode(enc)
        except UnicodeDecodeError:
            continue
    return raw.decode('gbk', errors='replace')


def load_hzk():
    """Hzk 每字 32 字节，源码里写成相邻两个 16 字节初始化块。"""
    text = read_text(FONT_H)
    body = text[text.index('char Hzk'):]
    blocks = []
    for inner in re.findall(r'\{([^{}]*)\}', body):
        vals = re.findall(r'0[xX][0-9a-fA-F]{2}', inner)
        if len(vals) == 16:
            blocks.append([int(v, 16) for v in vals])
    if len(blocks) % 2:
        raise SystemExit('Hzk 初始化块数量为奇数，字库文件可能被截断')
    return [blocks[i] + blocks[i + 1] for i in range(0, len(blocks), 2)]


def load_macros():
    """解析 #define HZ_XXX <下标> /* 汉字 */"""
    text = read_text(OLED_C)
    out = []
    for name, idx, ch in re.findall(
            r'#define\s+(HZ_\w+)\s+(\d+)\s*/\*\s*(\S+)', text):
        out.append((name, int(idx), ch))
    return out


def render_glyph(ch):
    from PIL import Image, ImageDraw, ImageFont
    img = Image.new('L', (16, 16), 0)
    ImageDraw.Draw(img).text((0, Y_OFFSET), ch, fill=255,
                             font=ImageFont.truetype(FONT_PATH, CHAR_SIZE))
    g = []
    for half in range(2):
        for row in range(16):
            b = 0
            for col in range(8):
                if img.getpixel((half * 8 + col, row)) >= 128:
                    b |= 0x80 >> col
            g.append(b)
    return g


def bits_diff(a, b):
    return sum(bin(x ^ y).count('1') for x, y in zip(a, b))


def dump(hzk, idx):
    g = hzk[idx]
    print(f'Hzk[{idx}]')
    for row in range(16):
        line = ''
        for col in range(16):
            byte, bit = (g[row], 7 - col) if col < 8 else (g[16 + row], 15 - col)
            line += '#' if (byte >> bit) & 1 else '.'
        print('   ' + line)


def main():
    hzk = load_hzk()
    macros = load_macros()
    print(f'Hzk 字模数={len(hzk)}  待校验宏数={len(macros)}')

    if '--dump' in sys.argv:
        dump(hzk, int(sys.argv[sys.argv.index('--dump') + 1]))
        return 0

    cache, bad = {}, []
    for name, idx, ch in macros:
        if not (0 <= idx < len(hzk)):
            bad.append(f'{name} -> Hzk[{idx}] 越界（字库只有 {len(hzk)} 字）')
            continue
        if len(ch) != 1:
            bad.append(f'{name} 注释里的 "{ch}" 不是单个汉字，无法校验')
            continue
        if ch not in cache:
            cache[ch] = render_glyph(ch)
        d = bits_diff(hzk[idx], cache[ch])
        flag = 'OK ' if d <= MAX_BITS_DIFF else 'BAD'
        print(f'  [{flag}] {name:9} = {idx:2}  {ch}  位差={d}')
        if d > MAX_BITS_DIFF:
            near = min((x for x in range(len(hzk)) if x != idx),
                       key=lambda x: bits_diff(hzk[x], cache[ch]))
            bad.append(f'{name} 声明「{ch}」但 Hzk[{idx}] 不匹配；'
                       f'最接近的是 Hzk[{near}]（位差={bits_diff(hzk[near], cache[ch])}）')

    if bad:
        print('\n校验失败:')
        for m in bad:
            print('  - ' + m)
        return 1
    print('\n全部通过：HZ_* 宏与 Hzk[] 字模一一对应。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
