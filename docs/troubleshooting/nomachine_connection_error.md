# JetArm 连接异常排查与恢复指南（2026修订版）

## 最终确认的故障模式

现象：

```text
NoMachine 登录后显示：

Oh no! Something has gone wrong.
A problem has occurred and the system can't recover.
Please log out and try again.
```

同时：

* SSH 正常
* ROS2 正常
* 网络正常
* JetArm 未死机

---

## 推荐排查顺序

### Step 1：确认网络

```bash
ping 192.168.100.1
```

---

### Step 2：确认 SSH

```bash
ssh ubuntu@192.168.100.1
```

---

### Step 3：检查系统时间

```bash
date
timedatectl
```

重点关注：

```text
Local time: Thu 1970-01-01
System clock synchronized: no
```

如果出现 1970：

```bash
sudo timedatectl set-ntp false
sudo date -s "2026-06-14 21:00:00"
```

---

### Step 4：检查 GDM

```bash
systemctl status gdm3 --no-pager
```

正常状态：

```text
Active: active (running)
```

重启：

```bash
sudo systemctl restart gdm3
```

---

### Step 5：检查 NoMachine（最关键）

查看状态：

```bash
sudo /usr/NX/bin/nxserver --status
```

正常：

```text
Enabled service: nxserver
Enabled service: nxnode
Enabled service: nxd
```

异常案例：

```text
Enabled service: nxserver
Disabled service: nxnode
Enabled service: nxd
```

如果发现 nxnode 被禁用：

```bash
sudo /usr/NX/bin/nxserver --restart
```

再次检查：

```bash
sudo /usr/NX/bin/nxserver --status
```

确认：

```text
Enabled service: nxnode
```

---

## 本次真实根因（2026-06）

本次问题并非：

* ROS2 问题
* 相机问题
* 机械臂问题
* 网络问题

实际链路：

```text
系统时间回到1970
↓
GNOME会话异常
↓
NoMachine服务异常
↓
nxnode未正常工作
↓
出现 Oh no! Something has gone wrong
```

恢复步骤：

```bash
sudo timedatectl set-ntp false
sudo date -s "2026-06-14 21:00:00"

sudo systemctl restart gdm3

sudo /usr/NX/bin/nxserver --restart
```

验证：

```bash
sudo /usr/NX/bin/nxserver --status
```

确认：

```text
Enabled service: nxserver
Enabled service: nxnode
Enabled service: nxd
```

然后重新连接 NoMachine。

---

## 快速恢复命令（推荐收藏）

```bash
date
timedatectl

sudo systemctl restart gdm3

sudo /usr/NX/bin/nxserver --restart

sudo /usr/NX/bin/nxserver --status
```
