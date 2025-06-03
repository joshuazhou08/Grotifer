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

Download libusb (used my labjack.so in the hardware sol lib)
```bash
sudo apt update
sudo apt install libusb-1.0-0
```
