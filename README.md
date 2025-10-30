# **iso-version-cmd_vel_mux**
A modified version of Yujin’s **cmd_vel_mux**:https://wiki.ros.org/cmd_vel_mux.

This version adds **key mode** to meet AMR certification requirements and introduces an **emergency velocity** that can bypass all validation checks and publish velocity commands immediately when critical conditions occur.

##  🧭 **Overview**
**iso-version-cmd_vel_mux** acts as a velocity command multiplexer that selectively publishes velocity commands to the robot based(like MCU) on their priority and validity.
It adds new safety features and operational states required for **AMR safety certification**, ensuring that only valid and authorized velocity commands are published.

## ⚙️ **Features**
- **Key Mode Verification** — validates operation mode before accepting commands (Manual / Navigation).

- **Emergency Velocity Control** — limits or blocks unsafe velocity outputs under abnormal conditions.

- **Priority-based Command Selection** — only the highest-priority active command is published.

- **Timeout Detection** — stops publishing when input velocity commands are not updated in time.

Diagnostic Integration — reports system status through ROS diagnostics for monitoring.


| Parameter | Type | Description |
|------------|------|-------------|
| key_mode | int | The operating mode key (manual / navigation). |
| priority_level | int | Determines which velocity source takes control. |