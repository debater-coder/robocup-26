# Raspberry Pi Software for RoboCup

This is a custom NixOS image for the software on our Raspberry Pis.
This image declares the complete OS configuration for the Pis,
including setting up the control software as a system service.

This image is based on these instructions:
- [janissary](https://blog.janissary.xyz/posts/nixos-install-custom-image)


## Bringup instructions
(based on article)

For this to work, ensure binfmt emulation is enabled. On NixOS:
```nix
boot.binfmt.emulatedSystems = [ "aarch64-linux" ];
```


1. Ensure all changes to the flake are staged to git.
2. Run the following command to generate the SD card image
```bash
nix run nixpkgs#nixos-generators -- -f sd-aarch64 --flake .#pi --system aarch64-linux -o ./pi.sd
```
3. Decompress the SD image:
```bash
cd pi.sd/sd-image
cp nixos-sd-image-24.05.20231124.5a09cb4-aarch64-linux.img.zst ~/ && cd ~
unzstd -d nixos-sd-image-24.05.20231124.5a09cb4-aarch64-linux.img.zst -o nixos-sd-image.img
```
4. Flash the image:
```bash
sudo dd if=~/nixos-sd-image.img of=/dev/DEVICEGOESHERE bs=1M status=progress
```
