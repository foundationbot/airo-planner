{
  description = "Python project dev shell with poetry2nix";
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    poetry2nix.url = "github:nix-community/poetry2nix";
    flake-utils.url = "github:numtide/flake-utils";
  };
  outputs = { self, nixpkgs, poetry2nix, flake-utils, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = nixpkgs.legacyPackages.${system};
        poetry2nixLib = poetry2nix.lib.mkPoetry2Nix { inherit pkgs; };
        pythonEnv = poetry2nixLib.mkPoetryEnv {
          python = pkgs.python311;
          projectDir = ./.;
        };
      in {
        devShells.default = pkgs.mkShell {
          packages = [
            pkgs.poetry
            pythonEnv
            # add other tools like ruff, pyright, etc. if needed
          ];
        };
      }
    );
}

