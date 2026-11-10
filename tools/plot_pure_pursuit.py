"""画 Pure Pursuit 轨迹与路径对比，输出 docs/pure_pursuit.png。
用法（在项目根目录）：python tools/plot_pure_pursuit.py
"""
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# 必须与 apps/path_trace.cpp 里的路径一致
PATH = [(0, 0), (4, 0), (4, 3), (0, 3), (0, 0.6)]

df = pd.read_csv("docs/trace_path.csv")
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.8))

# 左图：路径与实际轨迹
ax1.plot(*zip(*PATH), "k--", marker="o", label="planned path")
ax1.plot(df.x, df.y, color="tab:green", label="vehicle trajectory")
ax1.plot(df.x.iloc[0], df.y.iloc[0], "go", label="start")
ax1.plot(df.x.iloc[-1], df.y.iloc[-1], "rs", label="end")
ax1.set_aspect("equal", adjustable="datalim")
ax1.set_xlabel("x (m)"); ax1.set_ylabel("y (m)")
ax1.set_title("Pure pursuit path following (lookahead = 0.8 m)")
ax1.grid(True); ax1.legend(fontsize=8)

# 右图：偏离路径的距离随时间变化（转角处会出现峰值，这是切弯造成的）
ax2.plot(df.t, df.cross_track, color="tab:blue")
ax2.set_xlabel("time (s)"); ax2.set_ylabel("cross-track error (m)")
ax2.set_title("Distance from path")
ax2.grid(True)

fig.tight_layout()
fig.savefig("docs/pure_pursuit.png", dpi=150)
print(f"saved docs/pure_pursuit.png, max cross-track = {df.cross_track.max():.3f} m")
