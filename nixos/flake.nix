{
  description = "High Scorers 2026 RoboCup Software for Raspberry Pi";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixos-unstable";
    nixos-hardware.url = "github:NixOS/nixos-hardware/master";
  };

  outputs = { self, nixpkgs, nixos-hardware, ... }@inputs: rec {

    # here goes the other flake outputs, if you have any

    nixosConfigurations."pi" = nixpkgs.lib.nixosSystem {
      system = "aarch64-linux";
      modules = [
        ./configuration.nix
        nixos-hardware.nixosModules.raspberry-pi-4
      ];
    };
  };
}
