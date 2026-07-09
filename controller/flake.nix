{
  description = "Robot Soccer Controller for Raspberry Pi";

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
            packages = [
              pkgs.uv
              pkgs.cmake
              pkgs.python314
              pkgs.stdenv.cc.cc
              pkgs.zlib
              pkgs.glibc
              pkgs.swig
              pkgs.lgpio
            ];
            shellHook = ''
              	    export LD_LIBRARY_PATH="${pkgs.zlib}/lib:${pkgs.stdenv.cc.cc.lib}/lib:$LD_LIBRARY_PATH"
              			export GPIOZERO_PIN_FACTORY=lgpio
                    export CFLAGS="-I${pkgs.lgpio}/include $CFLAGS"
                    export LDFLAGS="-L${pkgs.lgpio}/lib $LDFLAGS"
              	  '';
          };
        }
      );
    };
}
