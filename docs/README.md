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

## Dependencies

Basic ones:

```bash
# Libmodbus
sudo apt update
sudo apt install libmodbus-dev
# Libusb
sudo apt install libusb-1.0-0-dev
sudo apt install libeigen3-dev
```

For labjack:

```bash
git clone https://github.com/labjack/exodriver.git
cd exodriver/liblabjackusb
sudo make clean

sudo make
sudo make install

sudo ldconfig

ls /usr/local/lib | grep labjackusb
# → should show: liblabjackusb.so

ls /usr/local/include | grep labjackusb
# → should show: labjackusb.h
```

For Maxon stuff, download the Epos and ftd2xx libraries [here](https://drive.google.com/drive/folders/1fGORTzJLLv8IFwaYweX2cKQKk2AuIADm?usp=sharing)

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
