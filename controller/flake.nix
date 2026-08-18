{
  description = "GPIO Zero uv dev shell";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
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
              pkgs.pkg-config
              pkgs.swig

              pkgs.stdenv.cc
              pkgs.stdenv.cc.cc.lib

              pkgs.zlib
              pkgs.glibc.dev
            ];

            shellHook = ''
                  export CC=cc
                  export CXX=c++

              		export LD_LIBRARY_PATH="${pkgs.stdenv.cc.cc.lib}/lib:${pkgs.zlib}/lib:${pkgs.lgpio}/lib:$LD_LIBRARY_PATH"
            '';
          };
        }
      );
    };
}
