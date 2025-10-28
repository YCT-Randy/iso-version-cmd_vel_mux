# **semi-cmd_vel_mux**
A modified version of Yujin’s **cmd_vel_mux**:https://wiki.ros.org/cmd_vel_mux.

This version adds **Key Mode** to meet **AMR safety certification** requirements and introduces an **Emergency Velocity** that can bypass all validation checks and publish velocity commands immediately when critical conditions occur.

## 🎬 **Demo VOD**
Link: https://www.youtube.com/watch?v=0KtuuE5DR4E
##  🧭 **Overview**
**semi-cmd_vel_mux** acts as a velocity command multiplexer that selectively publishes velocity commands to the robot based on their priority and validity.It introduces new safety features and operational states required for **AMR safety certification**, ensuring that only valid and authorized velocity commands are published.

## ⚙️ **Features**
- **Key Mode State**:  
In AMR safety certifications, manual operation is required. Therefore, we divide the AMR’s motion modes into two main types: **Automatic** and **Manual**, and use a ROS topic to receive mode-switching signals.

- **Single Velocity Output**:  
All system-generated velocity commands are sent to **semi-cmd_vel_mux**, which outputs only the velocity command that matches the **Key Mode** and has the highest **Priority Level**.

- **Velocity Priority Level**:  
The command with the highest **Priority Level** takes precedence and is published.

- **Select Current Velocity**:  
**semi-cmd_vel_mux** identifies the velocity command that matches the **Key Mode** and has the highest **Priority Level** as the **Current Velocity**, rejecting all other velocity outputs.  
The **Current Velocity** will only be updated when the **Key Mode** changes, a high **Priority Level** command with the same **Key Mode** is received, a timeout occurs, or an **Emergency Velocity** command is received.

- **Timeout Detection**:  
Publish zero velocity when **Current Velocity** commands are not updated in time.
When the **Current Velocity** is not updated within the timeout period, **semi-cmd_vel_mux** automatically publishes a zero velocity to stop AMR.

- **Emergency Velocity**:  
This velocity can bypass all conditions, forcing **semi-cmd_vel_mux** to output this command and update the **Current Velocity**.

## 🧾 **Config**

### **Subscribers:**
| Parameter | Type | Description |
|------------|------|-------------|
| name | string | Velocity topic name. |
| key_mode | int | Automatic:0 / Manual:1 / Emergency Velocity:-1 |
| priority_level | int | Set each velocity Priority Level. |
| timeout | float | Unit:seconds. |

### **Publisher:**
| Parameter | Type | Description |
|------------|------|-------------|
| name | string | Output velocity topic name. |

## 📡 **Subscribe Topic**
- **key_switch_mode**(std_msgs::Int8):  
Switch **Key Mode**(Automatic:0 / Manual:1)
