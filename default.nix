{ pkgs ? import <nixpkgs> {} }:
 let
     pypkgs = import (fetchTarball "https://github.com/cachix/nixpkgs-python/archive/refs/heads/main.zip");
in
pkgs.mkShell {

    buildInputs = with pkgs; [
        (python311.withPackages(ps: with ps;
          [virtualenvwrapper pip requests pyperclip]))
        stdenv.cc.cc

        # For Numpy
        zlib

        # For rendering gym environments
        libGL
        libGLU
        xorg.libX11

        xorg.libX11.dev
        xorg.libxcb.dev
        glib.dev
        glib.out
        gobject-introspection
        gtk3
        gtk3-x11
        gtk3.dev
        libffi.dev
        gcc
        pkg-config
        cairo
        cairo.dev
        mesa.osmesa
        cmake
        boost
    ];

    shellHook = ''
        # for Numpy
        export LD_LIBRARY_PATH=${pkgs.zlib}/lib:$LD_LIBRARY_PATH
        export LD_LIBRARY_PATH=${pkgs.mesa.osmesa}/lib:$LD_LIBRARY_PATH

        # GL libraries (for gym environment rendering)
        export LD_LIBRARY_PATH=${pkgs.libGL}/lib:$LD_LIBRARY_PATH
        export LD_LIBRARY_PATH=${pkgs.libGLU}/lib:$LD_LIBRARY_PATH
        export LD_LIBRARY_PATH=${pkgs.glib.out}/lib:$LD_LIBRARY_PATH
    '';
}