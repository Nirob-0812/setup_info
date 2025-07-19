# Connect Raspberry Pi 5 (Ubuntu 24.04 LTS) with Windows 11 WSL via SSH

This guide clearly explains how to establish a simple and reliable SSH connection between your **Raspberry Pi 5 (running Ubuntu 24.04 LTS)** and your **Windows 11 WSL environment**.

---

## ✅ Step 1: Prepare your Raspberry Pi

Connect to your Raspberry Pi directly (keyboard + monitor) or via another method to execute initial setup commands.

### Update and install SSH:
```bash
sudo apt update
sudo apt install openssh-server
```

### Enable SSH service (so it starts automatically):
```bash
sudo systemctl enable ssh
sudo systemctl start ssh
```

### Find Raspberry Pi's IP address:
```bash
hostname -I
```

Example output:
```
192.168.0.112
```

---

## ✅ Step 2: Establish SSH connection from Windows 11 WSL

Open your WSL (Ubuntu terminal) and run:

```bash
ssh your_username@192.168.0.112
```

Replace `your_username` with your Raspberry Pi username and `192.168.0.112` with your Pi's actual IP address.

When prompted, enter your Raspberry Pi password.

---

## ✅ Step 3: (Recommended) Configure passwordless SSH login

In WSL terminal, generate SSH keys if you don't have them yet:

```bash
ssh-keygen -t ed25519 -C "wsl_pi_key"
```

Copy your public key to the Pi:

```bash
ssh-copy-id your_username@192.168.0.112
```

Now you can connect without entering your password:

```bash
ssh your_username@192.168.0.112
```

---

## ⚙️ Useful commands

- Transfer files **WSL → Pi**:

```bash
scp ~/localfile.txt your_username@192.168.0.112:/home/your_username/
```

- Transfer files **Pi → WSL**:

```bash
scp your_username@192.168.0.112:/home/your_username/remotefile.txt ~/localfolder/
```

---

