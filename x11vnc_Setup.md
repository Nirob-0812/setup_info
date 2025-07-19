# Raspberry Pi 5 Ubuntu 24.04 – x11vnc Remote Desktop Setup for Windows 11

This guide shows how to set up `x11vnc` on a Raspberry Pi 5 running Ubuntu 24.04, so you can view the actual desktop from Windows 11 using VNC.

---

## 1. **Install Dependencies**

```bash
sudo apt update
sudo apt install lightdm x11vnc
```

---

## 2. **Enable Auto-Login (Recommended)**
Make sure your Pi auto-logs in to the desktop on boot.

**For LightDM** (`/etc/lightdm/lightdm.conf`):

```bash
sudo nano /etc/lightdm/lightdm.conf
```

Add at the end (replace `nirob0812` with your username):

```
[Seat:*]
autologin-user=nirob0812
autologin-user-timeout=0
user-session=ubuntu
greeter-session=lightdm-gtk-greeter
```

---

## 3. **Set a Secure x11vnc Password**

```bash
x11vnc -storepasswd
```
- Enter your desired password (will be saved at `~/.vnc/passwd`).

---

## 4. **Create the x11vnc systemd Service**

```bash
sudo nano /lib/systemd/system/x11vnc.service
```

Paste the following (change `nirob0812` to your user if needed):

```ini
[Unit]
Description=x11vnc service
After=display-manager.service network.target syslog.target

[Service]
Type=simple
ExecStart=/usr/bin/x11vnc -forever -display :0 -auth guess -rfbauth /home/nirob0812/.vnc/passwd
ExecStop=/usr/bin/killall x11vnc
Restart=on-failure
User=nirob0812

[Install]
WantedBy=multi-user.target
```

---

## 5. **Enable and Start the Service**

```bash
sudo systemctl daemon-reload
sudo systemctl enable x11vnc.service
sudo systemctl start x11vnc.service
sudo systemctl status x11vnc.service
```

---

## 6. **(Optional, but Recommended for Headless Use) – Force HDMI Mode**

If you want to VNC into your Pi with **no monitor attached**:

```bash
sudo nano /boot/firmware/config.txt
```
Add at the end:

```
hdmi_force_hotplug=1
hdmi_group=2
hdmi_mode=82
```

---
## Headless Setup Configuration (if needed)

If you find that the X server stops or behaves differently when the monitor is disconnected, you may want to configure the system to run in a headless mode. Here's how to handle it:

Prevent X from closing on disconnect (if necessary):

You can install a dummy display driver (like xserver-xorg-video-dummy) to ensure that a virtual display is always running, even without a monitor.

```bash
sudo apt install xserver-xorg-video-dummy
```
Then, configure a dummy display by modifying /etc/X11/xorg.conf to add the Dummy driver:
```bash
Section "Device"
Identifier "Dummy Device"
Driver "dummy"
EndSection

Section "Monitor"
Identifier "Dummy Monitor"
HorizSync 30-70
VertRefresh 50-75
EndSection

Section "Screen"
Identifier "Dummy Screen"
Device "Dummy Device"
Monitor "Dummy Monitor"
DefaultDepth 24
SubSection "Display"
Depth 24
Modes "1920x1080"
EndSubSection
EndSection
```

Keep the X session active: Ensure that the X server stays active even without a monitor attached by using xorg.conf settings or checking if there is a virtual display active.



## 7. **Connect from Windows 11**

- Download and install [VNC Viewer (RealVNC)](https://www.realvnc.com/en/connect/download/viewer/).
- Find your Pi’s IP with:  
  ```bash
  hostname -I
  ```
- In VNC Viewer, connect to:  
  ```
  [Pi_IP]:5900
  ```
  Example: `192.168.0.112:5900`
- Enter the password you set earlier.

---

## 8. **Sudo Without Password (Optional/Security Tradeoff)**

If you want to run `sudo` without being prompted for a password (for scripting or convenience):

```bash
sudo visudo
```
Add at the end (replace username):

```
nirob0812 ALL=(ALL) NOPASSWD:ALL
```

---

## 9. **References**

- [Linux Mint Forums](https://forums.linuxmint.com/viewtopic.php?t=440131)
- [x11vnc Official Site](https://www.karlrunge.com/x11vnc/)
- [RealVNC Viewer](https://www.realvnc.com/en/connect/download/viewer/)

- 

---

## **Done!**

You now have full remote desktop control from Windows 11 to your Raspberry Pi 5 running Ubuntu, with auto-login and persistent access after reboot.
