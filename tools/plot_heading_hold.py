"""对比有/无航向保持时的轨迹与朝向，输出 docs/heading_hold.png。
用法：python tools/plot_heading_hold.py
"""
import pandas as pd
import matplotlib
matplotlib.use("Agg")  # 无图形界面也能保存图片（WSL 里常见）
import matplotlib.pyplot as plt

hold = pd.read_csv("docs/trace_hold.csv")
nohold = pd.read_csv("docs/trace_nohold.csv")

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.5))

# 左图：xy 轨迹
ax1.plot(nohold.x, nohold.y, label="no heading hold", color="tab:red")
ax1.plot(hold.x, hold.y, label="with heading hold", color="tab:green")
ax1.set_xlabel("x (m)"); ax1.set_ylabel("y (m)")
ax1.set_title("Trajectory under constant yaw disturbance")
ax1.set_aspect("equal", adjustable="datalim")
ax1.grid(True); ax1.legend()

# 右图：朝向随时间变化
ax2.plot(nohold.t, nohold.heading, label="no heading hold", color="tab:red")
ax2.plot(hold.t, hold.heading, label="with heading hold", color="tab:green")
ax2.set_xlabel("time (s)"); ax2.set_ylabel("heading (rad)")
ax2.set_title("Heading error")
ax2.grid(True); ax2.legend()

fig.tight_layout()
fig.savefig("docs/heading_hold.png", dpi=150)
print("saved docs/heading_hold.png")
