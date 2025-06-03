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
