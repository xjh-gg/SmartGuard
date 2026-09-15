"""
 * ============================================================================
 *  SmartGuard v1.0 TinyML 训练脚本
 *  ============================================================================
 *
 *  功能：
 *    1. 解析 STM32 串口导出的 CSV 数据
 *    2. 训练 2 层 MLP 二分类模型（10→16→1, sigmoid）
 *    3. 自动评估 + 生成 STM32 部署文件
 *
 *  输入：  sensor_data.csv（STM32 串口复制出来的 ML, 开头行）
 *  输出：
 *    ├── saved_model/          ← TensorFlow SavedModel（喂给 STM32Cube.AI）
 *    ├── scaler_params.h       ← 标准化参数（硬编码进 STM32 C 代码）
 *    ├── training_report.txt   ← 完整评估报告
 *    └── confusion_matrix.png  ← 混淆矩阵图（如果有 matplotlib）
 *
 *  使用方法：
 *    python train.py                       # 默认从 sensor_data.csv 读
 *    python train.py --data my_data.csv    # 指定数据文件
 *    python train.py --epochs 100          # 自定义 epoch 数
 *
 *  依赖：
 *    pip install tensorflow pandas scikit-learn numpy
 *    可选：pip install matplotlib seaborn
 *  ============================================================================
 */

import os
import sys
import argparse
import numpy as np

# ── 1. 解析命令行参数 ──────────────────────────────────────────────────────
parser = argparse.ArgumentParser(description="SmartGuard TinyML 训练脚本")
parser.add_argument("--data",    default="sensor_data.csv", help="CSV 数据文件路径")
parser.add_argument("--epochs",  type=int, default=50,       help="训练 epoch 数")
parser.add_argument("--batch",   type=int, default=32,      help="batch size")
parser.add_argument("--model",   default="saved_model",    help="SavedModel 输出目录")
parser.add_argument("--no-plots", action="store_true",     help="跳过绘图（没有 matplotlib 时）")
args = parser.parse_args()

# ── 2. 环境检测 ────────────────────────────────────────────────────────────
print("=" * 60)
print(" SmartGuard v1.0 TinyML 训练")
print("=" * 60)

import tensorflow as tf
import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import (
    accuracy_score, precision_score, recall_score, f1_score,
    confusion_matrix, classification_report, roc_auc_score
)

# GPU 检测
gpus = tf.config.list_physical_devices('GPU')
if gpus:
    print(f"[OK] GPU 可用: {len(gpus)} 个")
else:
    print("[INFO] 未检测到 GPU，使用 CPU（模型很小，CPU 训练只需几秒）")

print(f"[INFO] TensorFlow {tf.__version__}")

# ── 3. 加载 + 清洗数据 ────────────────────────────────────────────────────
print(f"\n[1/5] 加载数据: {args.data}")

if not os.path.exists(args.data):
    print(f"[ERROR] 找不到数据文件: {args.data}")
    print("  请把 STM32 串口输出中 ML, 开头的行复制到 sensor_data.csv")
    sys.exit(1)

df = pd.read_csv(args.data)

# ── 兼容：去掉串口输出里可能混入的非 CSV 行 ──
# 只保留 label 是 0 或 1 的行
df = df[df["label"].isin([0, 1])].copy()

# ── 去掉 MQ-2 相关列（硬件未接，值全是 0，对训练无贡献）──
# 但保留列结构一致，部署时 STM32 会填 0
drop_cols = []
for col in ["mq2", "mq2_dev"]:
    if col in df.columns and df[col].sum() == 0:
        print(f"  [INFO] {col} 全为 0（MQ-2 未接），保留但不影响训练")
        # 保留列，因为 STM32 部署时输入维度要匹配

print(f"  样本总数: {len(df)}")
print(f"  列数: {len(df.columns)}")
print(f"\n  Label 分布:")
print(f"    正常 (label=0): {(df['label'] == 0).sum()} 条 "
      f"({(df['label'] == 0).mean() * 100:.1f}%)")
print(f"    异常 (label=1): {(df['label'] == 1).sum()} 条 "
      f"({(df['label'] == 1).mean() * 100:.1f}%)")

# ── 4. 准备特征 + 标准化 ──────────────────────────────────────────────────
print("\n[2/5] 特征工程 + 标准化")

# 10 维特征（和 STM32 输出顺序一致）
FEATURE_COLS = [
    "light", "dist", "temp", "humi", "mq2",          # 原始值
    "light_dev", "dist_delta", "temp_dev", "humi_dev", "mq2_dev"  # baseline 偏差
]

# 确保列都存在
for col in FEATURE_COLS:
    if col not in df.columns:
        print(f"  [WARN] 缺少列 {col}，补 0")
        df[col] = 0

X = df[FEATURE_COLS].values.astype(np.float32)
y = df["label"].values.astype(np.float32)

print(f"  X shape: {X.shape}  (样本数 × 10 维特征)")
print(f"  y shape: {y.shape}")

# ── 划分训练集/测试集（分层抽样，保证 label 比例一致）──
X_train, X_test, y_train, y_test = train_test_split(
    X, y, test_size=0.2, random_state=42, stratify=y
)
print(f"  训练集: {len(X_train)} 条  测试集: {len(X_test)} 条")

# ── ⭐ 标准化（训练效果关键！STM32 部署时也要用相同参数）──
scaler = StandardScaler()
X_train_scaled = scaler.fit_transform(X_train)
X_test_scaled  = scaler.transform(X_test)   # 注意：只 transform，不 fit！

print(f"\n  Scaler mean ({len(scaler.mean_)} 维):")
for i, col in enumerate(FEATURE_COLS):
    print(f"    {col:12s}: mean={scaler.mean_[i]:.4f}  "
          f"std={scaler.scale_[i]:.4f}")

# ── 5. 构建模型 ────────────────────────────────────────────────────────────
print(f"\n[3/5] 构建 2 层 MLP (10→16→1)")

model = tf.keras.Sequential([
    tf.keras.layers.Dense(
        16, input_shape=(10,),
        activation='relu',
        kernel_initializer='he_normal'
    ),
    tf.keras.layers.Dense(1, activation='sigmoid')
])

model.compile(
    optimizer=tf.keras.optimizers.Adam(learning_rate=0.001),
    loss='binary_crossentropy',
    metrics=['accuracy', tf.keras.metrics.Precision(), tf.keras.metrics.Recall()]
)

# 打印模型结构
model.summary()

# ── 早停回调（val_loss 连续 10 轮不降就停，防止过拟合）──
early_stop = tf.keras.callbacks.EarlyStopping(
    monitor='val_loss', patience=10, restore_best_weights=True, verbose=1
)

# ── 学习率衰减回调（后期学习率慢慢变小）──
reduce_lr = tf.keras.callbacks.ReduceLROnPlateau(
    monitor='val_loss', factor=0.5, patience=5, min_lr=1e-6, verbose=0
)

# ── 6. 训练 ────────────────────────────────────────────────────────────────
print(f"\n[4/5] 训练模型 (epochs={args.epochs}, batch={args.batch})")

history = model.fit(
    X_train_scaled, y_train,
    epochs=args.epochs,
    batch_size=args.batch,
    validation_data=(X_test_scaled, y_test),
    callbacks=[early_stop, reduce_lr],
    verbose=1
)

# ── 7. 评估 ────────────────────────────────────────────────────────────────
print(f"\n[5/5] 评估模型")

y_pred_proba = model.predict(X_test_scaled, verbose=0)
y_pred = (y_pred_proba > 0.5).astype(int).flatten()

acc  = accuracy_score(y_test, y_pred)
prec = precision_score(y_test, y_pred, zero_division=0)
rec  = recall_score(y_test, y_pred, zero_division=0)
f1   = f1_score(y_test, y_pred, zero_division=0)
cm   = confusion_matrix(y_test, y_pred)

print(f"\n  ┌──────────────────────────────┐")
print(f"  │  Accuracy : {acc:.4f} ({acc*100:.2f}%)   │")
print(f"  │  Precision: {prec:.4f}           │")
print(f"  │  Recall   : {rec:.4f}           │")
print(f"  │  F1 Score : {f1:.4f}           │")
print(f"  └──────────────────────────────┘")

print(f"\n  混淆矩阵 (行=实际, 列=预测):")
print(f"              预测=0(正常)  预测=1(异常)")
print(f"  实际=0(正常)  {cm[0][0]:>10}  {cm[0][1]:>10}")
print(f"  实际=1(异常)  {cm[1][0]:>10}  {cm[1][1]:>10}")

if cm.shape == (2, 2) and cm.sum() > 0:
    specificity = cm[0][0] / (cm[0][0] + cm[0][1]) if (cm[0][0]+cm[0][1]) > 0 else 0
    print(f"  Specificity (真阴性率): {specificity:.4f}")

try:
    auc = roc_auc_score(y_test, y_pred_proba.flatten())
    print(f"  ROC AUC: {auc:.4f}")
except:
    pass

print(f"\n  完整分类报告:")
print(classification_report(y_test, y_pred, target_names=["正常", "异常"],
                              zero_division=0))

# ── 8. 保存训练报告 ────────────────────────────────────────────────────────
report_path = os.path.join(os.path.dirname(args.data), "training_report.txt")
with open(report_path, "w", encoding="utf-8") as f:
    f.write("=" * 60 + "\n")
    f.write(" SmartGuard v1.0 TinyML 训练报告\n")
    f.write("=" * 60 + "\n\n")
    f.write(f"数据文件: {args.data}\n")
    f.write(f"总样本: {len(df)} 条\n")
    f.write(f"  正常: {(df['label']==0).sum()} 条\n")
    f.write(f"  异常: {(df['label']==1).sum()} 条\n\n")
    f.write(f"训练集: {len(X_train)} 条  测试集: {len(X_test)} 条\n\n")
    f.write(f"模型: 2 层 MLP (10→16→1, sigmoid)\n\n")
    f.write(f"┌──────────────────────────────┐\n")
    f.write(f"│  Accuracy : {acc:.4f} ({acc*100:.2f}%)   │\n")
    f.write(f"│  Precision: {prec:.4f}           │\n")
    f.write(f"│  Recall   : {rec:.4f}           │\n")
    f.write(f"│  F1 Score : {f1:.4f}           │\n")
    f.write(f"└──────────────────────────────┘\n\n")
    f.write(f"混淆矩阵:\n")
    f.write(f"              预测=0(正常)  预测=1(异常)\n")
    f.write(f"  实际=0(正常)  {cm[0][0]:>10}  {cm[0][1]:>10}\n")
    f.write(f"  实际=1(异常)  {cm[1][0]:>10}  {cm[1][1]:>10}\n\n")
    f.write(f"Scaler mean/std:\n")
    for i, col in enumerate(FEATURE_COLS):
        f.write(f"  {col:12s}: mean={scaler.mean_[i]:.6f}, std={scaler.scale_[i]:.6f}\n")

print(f"\n  [OK] 训练报告已保存: {report_path}")

# ── 9. 保存 SavedModel（喂给 STM32Cube.AI）────────────────────────────────
save_dir = args.model
if os.path.exists(save_dir):
    import shutil
    shutil.rmtree(save_dir)
model.save(save_dir)
print(f"  [OK] SavedModel 已保存: {save_dir}/")

# ── 10. 生成 STM32 部署用 C 头文件 ──────────────────────────────────────────
print(f"\n  生成 STM32 部署文件...")

header_path = os.path.join(os.path.dirname(args.data), "scaler_params.h")
with open(header_path, "w", encoding="utf-8") as f:
    f.write("/**\n")
    f.write("  * ==========================================================================\n")
    f.write("  * @file    scaler_params.h\n")
    f.write("  * @brief   StandardScaler 参数（v1.0 TinyML 部署用）\n")
    f.write("  *\n")
    f.write("  * 由 train.py 自动生成 — 不要手动修改！\n")
    f.write("  * 和 STM32 端 RunAIInference() 函数里的标准化逻辑一一对应\n")
    f.write("  *\n")
    f.write(f"  * 10 维特征顺序：{' '.join(FEATURE_COLS)}\n")
    f.write("  * ==========================================================================\n")
    f.write("  */\n\n")
    f.write("#ifndef __SCALER_PARAMS_H__\n")
    f.write("#define __SCALER_PARAMS_H__\n\n")
    f.write("#define ML_INPUT_DIM  10\n\n")
    
    f.write("/* Scaler mean（和训练时 StandardScaler.fit_transform 用的一致） */\n")
    f.write("static const float ML_ScalerMean[ML_INPUT_DIM] = {\n")
    for i in range(10):
        comma = "," if i < 9 else ""
        f.write(f"    {scaler.mean_[i]:.6f}{comma}   /* {FEATURE_COLS[i]} */\n")
    f.write("};\n\n")
    
    f.write("/* Scaler std（scale_ 属性，不是 scale_ 的倒数！） */\n")
    f.write("static const float ML_ScalerStd[ML_INPUT_DIM] = {\n")
    for i in range(10):
        comma = "," if i < 9 else ""
        f.write(f"    {scaler.scale_[i]:.6f}{comma}   /* {FEATURE_COLS[i]} */\n")
    f.write("};\n\n")
    
    f.write("#endif /* __SCALER_PARAMS_H__ */\n")

print(f"  [OK] C 头文件已保存: {header_path}")

# ── 11. 可选：混淆矩阵图 ──────────────────────────────────────────────────
if not args.no_plots:
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        
        fig, ax = plt.subplots(figsize=(6, 5))
        im = ax.imshow(cm, interpolation='nearest', cmap=plt.cm.Blues)
        ax.set_title(f"Confusion Matrix (Acc={acc*100:.1f}%)")
        fig.colorbar(im)
        ax.set_xticks([0, 1])
        ax.set_yticks([0, 1])
        ax.set_xticklabels(["正常 (0)", "异常 (1)"])
        ax.set_yticklabels(["正常 (0)", "异常 (1)"])
        ax.set_xlabel("预测")
        ax.set_ylabel("实际")
        
        thresh = cm.max() / 2.
        for i in range(2):
            for j in range(2):
                ax.text(j, i, format(cm[i, j], 'd'),
                        ha="center", va="center",
                        color="white" if cm[i, j] > thresh else "black")
        
        fig.tight_layout()
        plot_path = os.path.join(os.path.dirname(args.data), "confusion_matrix.png")
        plt.savefig(plot_path, dpi=100)
        plt.close()
        print(f"  [OK] 混淆矩阵图: {plot_path}")
    except ImportError:
        print("  [INFO] 未安装 matplotlib，跳过绘图 (pip install matplotlib)")
    except Exception as e:
        print(f"  [WARN] 绘图失败: {e}")

# ── 12. 完成 + 下一步指引 ──────────────────────────────────────────────────
print("\n" + "=" * 60)
print(" ✅ 训练完成！")
print("=" * 60)
print(f"""
 📁 输出文件：
   {os.path.abspath(save_dir)}/      ← SavedModel（喂 STM32Cube.AI）
   {os.path.abspath(header_path)}    ← C 头文件（部署进 STM32）
   {os.path.abspath(report_path)}    ← 训练报告
   {os.path.abspath('confusion_matrix.png')}  ← 混淆矩阵图（如果有）

 🚀 下一步（STM32Cube.AI 转换）：
   1. 下载 STM32CubeMX: https://www.st.com/en/development-tools/stm32cubemx.html
   2. 新建工程 → 选 STM32F407ZE
   3.顶部菜单 Stm32Cube.AI → Generate Application
   4. Add Model → 选 saved_model 文件夹
   5. Optimize: Balanced, Quantization: INT8
   6. Analyze → 看 Flash/RAM 报告
   7. Generate → 复制生成的文件到 SmartGuard/Hardware/ML/
   8. 把 scaler_params.h 也复制过去
   9. 在 decision_engine.c 里跑推理

 🎯 部署时别忘了：
   用 scaler_params.h 里的 ML_ScalerMean / ML_ScalerStd
   和训练时 StandardScaler 的参数完全一致！
""")
