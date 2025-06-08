# Grotifer
## Build Instructions
```bash
cmake -S . -B build         # -S: source dir (current folder), -B: build dir
cmake --build build -j$(nproc)   # Compile using all CPU cores
```

## Run the Executable
```bash
# After building
./build/mainExe
```

## Windows Development
WSL is needed for windows development.
```bash
wsl --install
```
When you open the repository in vscode or any platform, make sure to build in WSL terminal.

### Give your Windows an ssh key if not already
```bash
ssh-keygen -t ed25519 -C "your_email@example.com"
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519
cat ~/.ssh/id_ed25519.pub #copy entire line starting with ssh-ed25519
git config --global user.name "Your Full Name"
git config --global user.email "your_email@example.com"
```

Go to GitHub → Settings > SSH and GPG keys

Click "New SSH Key"

Paste the public key into the box


Download libusb (used my labjack.so in the hardware sol lib)
```bash
sudo apt update
sudo apt install libusb-1.0-0
```

## Deploying On The Pi
On your windows machine, download the **sftp extension** by **Natizyskunk** in vs code.

Create a new folder called Pi and open it with vs code. Inside the folder, do
ctrl + shift + p -> SFTP:config

This opens up sftp.json. Copy and paste the following into it:
```bash
{
  "name": "My Pi via SFTP",
  "host": "192.168.8.170",
  "protocol": "sftp",
  "port": 22,
  "username": "pi",
  "password": "grotifer_2024",
  "remotePath": "/home/pi/Desktop",

  "uploadOnSave": true,
  "ignore": [
    "**/.vscode/**",
    "**/.git/**",
    "**/.DS_Store"
  ]
}
```

Then, do ctrl + shift + p -> SFTP:Download Project. Make sure you are connected to the Grotifer Local Network.

Open a new terminal in vscode. SSH into the pi:
```bash
ssh pi@192.168.8.170
```

Once in, you will be in /home/pi/Desktop. The Grotifer-main folder has the cmake project.
In Grotifer-main, run 
```bash
cmake -S . -B build
cmake --build build -j$(nproc)
./build/mainExe
```

to run the program. **THIS IS ALL DONE IN VS CODE ON YOUR REMOTE MACHINE**.

### Cmake build clock skew error
Run:
```bash
find . -exec touch {} +
```

### Updating your local Pi folder with changes from the actual pi:
** ALWAYS RUN ON START** ctrl + shift + p SFTP: Download Project.
WARNING this rewrites everything in your local directory.
Ideally, we just use the Pi folder for live experiments and use this repository for actual code.
