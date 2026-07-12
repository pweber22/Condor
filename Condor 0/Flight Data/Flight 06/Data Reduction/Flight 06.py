import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
import os
# import datetime

# Read the log file
log_path = os.path.join(os.path.dirname(__file__), "..\\Onboard\\00000008.log")
with open(log_path, "r") as f:
    
    lines = f.readlines()

# Parse lines and group by message type (first column)
dataframes = {}

for line in lines:
    line = line.strip()
    if not line:
        continue
    
    # Split by comma and get the message type (first column)
    parts = [p.strip() for p in line.split(',')]
    msg_type = parts[0]
    
    # Create or append to dataframe for this message type
    if msg_type not in dataframes:
        dataframes[msg_type] = []
    
    dataframes[msg_type].append(parts)

# Convert lists to dataframes
for msg_type in dataframes:
    if dataframes[msg_type]:
        # First row might be headers for FMT/FMTU entries, use all rows as data
        dataframes[msg_type] = pd.DataFrame(dataframes[msg_type])

# Display summary of message types found
# print("Message types found in log file:")
# for msg_type in sorted(dataframes.keys()):
#     print(f"  {msg_type}: {len(dataframes[msg_type])} rows")

dataframes["AHR2"].columns = ["MessageType", "Timestamp", "Roll", "Pitch", "Yaw", "Alt", "Lat", "Lng",
                              "Q1", "Q2", "Q3", "Q4"]

dataframes["AHR2"]["Timestamp"] = pd.to_numeric(dataframes["AHR2"]["Timestamp"], errors='coerce')/1000000 # Convert to seconds



# ************* IMU Plots ***************

dataframes["IMU"].columns = ["MessageType", "Timestamp", "I", "GyroX", "GyroY", "GyroZ", "AccelX", "AccelY",
                              "AccelZ", "EG", "EA", "T", "GH", "AH", "GHz", "AHz"]
dataframes["IMU"]["Timestamp"] = pd.to_numeric(dataframes["IMU"]["Timestamp"], errors='coerce')/1000000 # Convert to seconds
dataframes["IMU"]["AccelZ"] = pd.to_numeric(dataframes["IMU"]["AccelZ"], errors='coerce')/-9.80665  # Convert AccelZ to numeric g
#test = (dataframes["IMU"].head(300))  # Display first 30 rows of IMU data for verification
plt.figure(figsize=(16, 9))
plt.plot(dataframes["IMU"]["Timestamp"].astype(int), dataframes["IMU"]["AccelZ"].astype(float))  # Full plot
plt.xlabel("Time Since Power On (s)")
plt.ylabel("AccelZ (g)")
plt.title("Z Acceleration vs Time")
plt.savefig("Condor 0\\Flight Data\\Flight 06\\Data Reduction\\Z_Acceleration_Full.png", dpi=300, bbox_inches='tight')  # Save the plot as a PNG file

plt.close()  # Close the full plot

imu_takeoff = dataframes["IMU"][(dataframes["IMU"]["Timestamp"] >= 365) & (dataframes["IMU"]["Timestamp"] <= 370)]
plt.figure(figsize=(16, 9))
plt.plot(imu_takeoff["Timestamp"], imu_takeoff["AccelZ"].astype(float))  # Zoomed-in plot
plt.xlabel("Time Since Power On (s)")
plt.ylabel("AccelZ (g)")
plt.title("Z Acceleration vs Time (Takeoff)")
plt.savefig("Condor 0\\Flight Data\\Flight 06\\Data Reduction\\Z_Acceleration_Takeoff.png", dpi=300, bbox_inches='tight')  # Save the zoomed-in plot as a PNG file
plt.close()



# ************* XKF1 Plots ***************

dataframes["XKF1"].columns = ["type", "timestamp", "c", "roll", "pitch", "yaw", "VN", "VE", "VD", "dPD",
                               "PN", "PE", "PD", "GX", "GY", "GZ", "OH"]
# print(dataframes["XKF1"].head(3))
dataframes["XKF1"]["VN"]=dataframes["XKF1"]["VN"].astype(float)
dataframes["XKF1"]["VE"]=dataframes["XKF1"]["VE"].astype(float)
# print(dataframes["XKF1"].head(3))
dataframes["XKF1"]["groundspeed"] = np.sqrt(dataframes["XKF1"]["VN"]**2 + dataframes["XKF1"]["VE"]**2)
dataframes["XKF1"]["timestamp"] = pd.to_numeric(dataframes["XKF1"]["timestamp"], errors='coerce')/1000000 # Convert to seconds
print("max speed", dataframes["XKF1"]["groundspeed"].max())

plt.figure(figsize=(16, 9))
plt.plot(dataframes["XKF1"]["timestamp"], dataframes["XKF1"]["groundspeed"])
plt.xlabel("Time Since Power On (s)")
plt.ylabel("groundspeed (m/s)")
plt.title("Condor 0 Flight 06 Groundspeed")
plt.savefig("Condor 0\\Flight Data\\Flight 06\\Data Reduction\\groundspeed_full.png", dpi=300, bbox_inches='tight')  # Save the plot
plt.close()

# ************* Attitude Plots ***************

dataframes["ATT"].columns = ["type", "timestamp", "DesRoll", "roll", "DesPitch", "pitch", "DesYaw", "yaw", "AEKF"]
#print(dataframes["ATT"].head(3))
dataframes["ATT"]["DesRoll"]=dataframes["ATT"]["DesRoll"].astype(float)
dataframes["ATT"]["roll"]=dataframes["ATT"]["roll"].astype(float)
dataframes["ATT"]["RollDev"]=dataframes["ATT"]["DesRoll"]-dataframes["ATT"]["roll"]
dataframes["ATT"]["DesPitch"]=dataframes["ATT"]["DesPitch"].astype(float)
dataframes["ATT"]["pitch"]=dataframes["ATT"]["pitch"].astype(float)
dataframes["ATT"]["PitchDev"]=dataframes["ATT"]["DesPitch"]-dataframes["ATT"]["pitch"]
dataframes["ATT"]["DesYaw"]=dataframes["ATT"]["DesYaw"].astype(float)
dataframes["ATT"]["yaw"]=dataframes["ATT"]["yaw"].astype(float)
dataframes["ATT"]["YawDev"]=dataframes["ATT"]["DesYaw"]-dataframes["ATT"]["yaw"]
#print(dataframes["ATT"].head(3))
dataframes["ATT"]["timestamp"] = pd.to_numeric(dataframes["ATT"]["timestamp"], errors='coerce')/1000000 # Convert to seconds

plt.figure(figsize=(16, 9))
plt.plot(dataframes["ATT"]["timestamp"], dataframes["ATT"]["RollDev"])
plt.xlabel("Time Since Power On (s)")
plt.ylabel("Roll Deviation (deg)")
plt.title("Condor 0 Flight 06 Roll Deviation")
plt.savefig("Condor 0\\Flight Data\\Flight 06\\Data Reduction\\Roll Dev.png", dpi=300, bbox_inches='tight')  # Save the plot
plt.close()

plt.figure(figsize=(16, 9))
plt.plot(dataframes["ATT"]["timestamp"], dataframes["ATT"]["PitchDev"])
plt.xlabel("Time Since Power On (s)")
plt.ylabel("Pitch Deviation (deg)")
plt.title("Condor 0 Flight 06 Pitch Deviation")
plt.savefig("Condor 0\\Flight Data\\Flight 06\\Data Reduction\\Pitch Dev.png", dpi=300, bbox_inches='tight')  # Save the plot
plt.close()


# ************* AOA Plots ***************

dataframes["AOA"].columns = ["type", "timestamp", "AOA", "SSA"]
# print(dataframes["AOA"].head(3))
dataframes["AOA"]["AOA"]=dataframes["AOA"]["AOA"].astype(float)
dataframes["AOA"]["SSA"]=dataframes["AOA"]["SSA"].astype(float)
# print(dataframes["AOA"].head(3))
dataframes["AOA"]["timestamp"] = pd.to_numeric(dataframes["AOA"]["timestamp"], errors='coerce')/1000000 # Convert to seconds


AOA_Flight = dataframes["AOA"][(dataframes["AOA"]["timestamp"] >= 76) & (dataframes["AOA"]["timestamp"] <= 625)]

plt.figure(figsize=(16, 9))
plt.plot(AOA_Flight["AOA"], AOA_Flight["SSA"])
plt.xlim(-22,22)
plt.ylim(-22,32)
plt.xlabel("Beta (deg)")
plt.ylabel("Alpha (deg)")
plt.title("Condor 0 Flight 06 \u03b1/\u03b2")
plt.savefig("Condor 0\\Flight Data\\Flight 06\\Data Reduction\\AlphaBeta.png", dpi=300, bbox_inches='tight')  # Save the plot
plt.close()

# ************* Control Tuning Plots ***************

dataframes["CTUN"].columns = ["type", "timestamp", "NavRoll", "Roll", "NavPitch", "Pitch", "ThO", "RdO", "ThD", "As", "AsT",
                                "E2T", "GU"]
# print(dataframes["CTUN"].head(3))
dataframes["CTUN"]["As"]=dataframes["CTUN"]["As"].astype(float)
# print(dataframes["CTUN"].head(3))
dataframes["CTUN"]["timestamp"] = pd.to_numeric(dataframes["CTUN"]["timestamp"], errors='coerce')/1000000 # Convert to seconds

plt.figure(figsize=(16, 9))
plt.plot(dataframes["CTUN"]["timestamp"], dataframes["CTUN"]["As"])
plt.xlabel("Time Since Power On (s)")
plt.ylabel("Airspeed (m/s)")
plt.title("Condor 0 Flight 06 Airspeed")
plt.savefig("Condor 0\\Flight Data\\Flight 06\\Data Reduction\\Airspeed_Full.png", dpi=300, bbox_inches='tight')  # Save the plot
plt.close()

landing_start = 600
landing_end = 640

CTUN_Landing = dataframes["CTUN"][(dataframes["CTUN"]["timestamp"] >= landing_start) & (dataframes["CTUN"]["timestamp"] <= landing_end)]
XKF1_Landing = dataframes["XKF1"][(dataframes["XKF1"]["timestamp"] >= landing_start) & (dataframes["XKF1"]["timestamp"] <= landing_end)]
AOA_Landing = dataframes["AOA"][(dataframes["AOA"]["timestamp"] >= landing_start) & (dataframes["AOA"]["timestamp"] <= landing_end)]
plt.figure(figsize=(16, 9))
plt.plot(CTUN_Landing["timestamp"], CTUN_Landing["As"], label = "Airspeed (m/s)")
plt.plot(XKF1_Landing["timestamp"], -(XKF1_Landing["PD"].astype(float)), label="Height AGL(m)")
plt.plot(AOA_Landing["timestamp"], AOA_Landing["AOA"], label="AOA (deg)")
plt.xlabel("Time Since Power On (s)")
# plt.ylabel("Airspeed (m/s)")
plt.title("Condor 0 Flight 06 Airspeed (Landing)")
plt.legend()
plt.text(600,-50, "Will caught the plane so this data doesn't make any sense")
plt.savefig("Condor 0\\Flight Data\\Flight 06\\Data Reduction\\Landing.png", dpi=300, bbox_inches='tight')  # Save the plot


# plt.show()
plt.close()