{
  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixos-unstable";
  };

  outputs =
    { nixpkgs, ... }:
    let
      inherit (nixpkgs) lib;
      forAllSystems = lib.genAttrs lib.systems.flakeExposed;
    in
    {
      devShells = forAllSystems (
        system:
        let
          pkgs = nixpkgs.legacyPackages.${system};
        in
        {
          default = pkgs.mkShell {
            packages = with pkgs; [
              picotool
              flip-link
              minicom
              rustup
              gcc-arm-embedded
            ];
          };

          shellHook = ''
            export ARM_SYSROOT="${pkgs.gcc-arm-embedded}/arm-none-eabi"
          '';
        }
      );
    };
}
