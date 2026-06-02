# JetArm 连接异常排查与恢复指南

## 背景

在 Sprint 6.1 开发期间，JetArm 出现了以下现象：

* NoMachine 登录后显示：

```text
Oh no! Something has gone wrong.
A problem has occurred and the system can't recover.
Please log out and try again.
```

* ROS2 节点无法正常查看
* 图形桌面无法进入
* 怀疑 JetArm 宕机或网络异常

最终确认：

* JetArm 未关机
* 网络正常
* SSH 正常
* ROS2 环境正常
* 问题来自 GNOME / NoMachine 会话异常
* 系统时间错误（1970 年）导致桌面会话异常

---

# 一、JetArm 无法连接时的排查顺序

## Step 1：确认网络连接

在 PC 上执行：

```bash
ping 192.168.100.1
```

正常结果：

```text
64 bytes from 192.168.100.1
0% packet loss
```

判断：

| 结果      | 结论             |
| ------- | -------------- |
| Ping 通  | JetArm 网络正常    |
| Ping 不通 | 检查网线、IP、交换机、电源 |

---

## Step 2：确认 SSH 是否正常

```bash
ssh ubuntu@192.168.100.1
```

如果能够登录：

```text
Welcome to Ubuntu ...
```

说明：

* JetArm 正常运行
* Linux 系统正常
* 网络正常

问题仅存在于图形界面。

---

## Step 3：检查时间是否异常

登录 JetArm 后：

```bash
date
timedatectl
```

异常案例：

```text
Thu 1970-01-01
```

或者：

```text
System clock synchronized: no
```

说明：

* 系统时间丢失
* NTP 未同步
* RTC 时间失效

常见影响：

* GNOME 崩溃
* GDM 登录异常
* NoMachine 无法创建会话

---

## Step 4：检查 Display Manager

```bash
systemctl status display-manager --no-pager
```

正常结果：

```text
gdm.service
Active: active (running)
```

说明：

* GNOME 服务正常启动

如果服务未运行：

```bash
sudo systemctl restart gdm3
```

或：

```bash
sudo systemctl restart display-manager
```

---

## Step 5：检查 NoMachine 服务

```bash
sudo systemctl status nxserver --no-pager
```

正常结果：

```text
Active: active (running)
```

重启 NoMachine：

```bash
sudo /usr/NX/bin/nxserver --restart
```

检查状态：

```bash
sudo /usr/NX/bin/nxserver --status
```

---

# 二、本次问题根因

发现：

```text
系统时间回到 1970 年
```

检查结果：

```text
Local time: Thu 1970-01-01
RTC time: Thu 1970-01-01
System clock synchronized: no
```

导致：

```text
pam_unix:
account ubuntu has password changed in future
```

GNOME 与 NoMachine 无法正确创建用户会话。

---

# 三、本次恢复过程

## 1. 修正系统时间

```bash
sudo timedatectl set-ntp false

sudo date -s "2026-05-31 23:42:00"
```

验证：

```bash
date
```

输出：

```text
Sun May 31 2026
```

---

## 2. 重启图形界面

```bash
sudo systemctl restart gdm3
```

---

## 3. 重启 NoMachine

```bash
sudo /usr/NX/bin/nxserver --restart
```

---

## 4. 重新连接 NoMachine

恢复成功后出现：

```text
User ubuntu connected
Your desktop is currently viewed
```

桌面恢复正常。

---

# 四、后续建议

## 检查 RTC

执行：

```bash
sudo hwclock -r
```

确认硬件时钟是否工作。

---

## 检查 NTP

执行：

```bash
timedatectl
systemctl status systemd-timesyncd
```

确保：

```text
System clock synchronized: yes
```

---

## 建议保留的快速排查命令

### 网络

```bash
ping 192.168.100.1
```

### SSH

```bash
ssh ubuntu@192.168.100.1
```

### 时间

```bash
date
timedatectl
```

### GDM

```bash
systemctl status display-manager
```

### NoMachine

```bash
sudo /usr/NX/bin/nxserver --status
sudo /usr/NX/bin/nxserver --restart
```

---

# 最终结论

本次故障并非：

* ROS2 问题
* 相机问题
* 机械臂问题
* 网络问题

根因是：

```text
系统时间异常（1970）
↓
GNOME 会话异常
↓
NoMachine 无法正常显示桌面
```

恢复时间并重启 GDM + NoMachine 后，JetArm 恢复正常。
