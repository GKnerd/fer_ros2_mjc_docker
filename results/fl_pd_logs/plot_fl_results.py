import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("fl_full_log.csv")
time = df["time"]

for i in [2, 4]:
    plt.figure()
    plt.plot(time, df[f"q{i}"], label=f"q{i} actual")
    plt.plot(time, df[f"qdes{i}"], label=f"q{i} desired", linestyle="--")
    plt.xlabel("Time [s]")
    plt.ylabel("Position [rad]")
    plt.title(f"Joint {i} Position Tracking")
    plt.legend()
    plt.grid(True)

    plt.figure()
    plt.plot(time, df[f"dq{i}"], label=f"dq{i}")
    plt.xlabel("Time [s]")
    plt.ylabel("Velocity [rad/s]")
    plt.title(f"Joint {i} Velocity")
    plt.legend()
    plt.grid(True)

    plt.figure()
    plt.plot(time, df[f"tau{i}"], label=f"tau{i}")
    plt.xlabel("Time [s]")
    plt.ylabel("Torque [Nm]")
    plt.title(f"Joint {i} Torque Command")
    plt.legend()
    plt.grid(True)

plt.show()