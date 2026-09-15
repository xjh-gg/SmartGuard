# build_verify.bat 使用说明

## 用途

一键完成 **全量编译 + 编译真实性校验**，防止 Keil UV4 的"陈旧构建"假阳性：
UV4 在某些情况下会用陈旧的目标文件（.o/.crf）链接，表面显示 `0 Error(s), 0 Warning(s)`，
但源码改动实际并未编入固件（2026-09 实测发生过一次：ZI-data 未随新增静态缓冲增长，
下一次全量 Rebuild 才补入）。本脚本在编译后额外做两层校验，把这类假阳性拦下来。

## 使用方法

双击 `build_verify.bat` 即可，无需管理员权限、无环境变量依赖
（仅要求 Keil 安装在 `D:\Keil_v5`）。可重复运行，幂等。

## 输出各行含义

| 输出 | 含义 |
|---|---|
| `Project directory:` | 自动定位的项目根目录（脚本所在 `User\` 的上一级） |
| `[1/4] Running full Keil rebuild (-r Rebuild)...` | 调用 UV4 全量重建，日志写入项目根目录 `build_log.txt` |
| `UV4 exit code:` | UV4 进程退出码（0 = 正常结束） |
| `[2/4] Build result: 0 Error` | 编译日志含 `- 0 Error(s)`；若为 `errors found` 会附错误行 |
| `[3/4] Full-rebuild marker: YES` | 日志含 `Rebuild target`，确认本次是**全量**重建；`NO` 表示增量构建，有陈旧目标文件风险 |
| `[4/4] Auditing source vs object timestamps...` | 逐一比对：`User/Hardware/Library/src/Core` 下每个 `.c` 与其 `Object\*.crf` 的时间戳，每个 `.h` 与 `Template.axf` 的时间戳 |
| `Not compiled: X (source newer than object)` | 异常：源文件比其目标文件新，说明改动未编入 |
| `Info: X not compiled (no .crf, disabled module)` | 正常参考信息：工程未启用的库文件/参考文件，不参与编译 |
| `Uncompiled source files: none` | 所有参与编译的源文件均已编入 |
| `Program Size: Code=... ZI-data=...` | 最终固件体积，可与历史值对照判断改动是否入链 |
| `Result: PASS / FAIL` | 总结论 |

## 为什么脚本用英文

Windows cmd 按系统代码页（中文系统为 GBK）解析批处理文件。中文内容若以 UTF-8 保存会
解析出乱码、以 GBK 保存则被 UTF-8 编辑器另存后损坏——两种编码下都无法安全编辑。
脚本改为全 ASCII 后，任何编辑器（UTF-8 / UTF-8 BOM / GBK）保存都得到相同字节，
永远不会因编码问题坏掉，因此也不需要 `chcp` 切换代码页。

## 退出码

- `0`：全量重建 + 所有源文件均已编入 + 0 Error，可用于 CI 判断
- `1`：编译报错 / 非全量重建 / 存在未编译源文件 / 环境异常（找不到 UV4、日志或 axf）

## 注意

`build_verify.bat` 必须保持纯 ASCII 内容。如需修改，请只使用 ASCII 字符，
不要再引入中文注释或全角标点。
