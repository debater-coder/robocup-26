{ pkgs, ... }:

{
  imports = [
    ./hardware-configuration.nix
  ];

  # Use the extlinux boot loader. (NixOS wants to enable GRUB by default)
  boot.loader.grub.enable = false;
  # Enables the generation of /boot/extlinux/extlinux.conf
  boot.loader.generic-extlinux-compatible.enable = true;


  # the user account on the machine
  users.users.robocup = {
    isNormalUser = true;
    extraGroups = [ "wheel" ]; # Enable ‘sudo’ for the user.
    hashedPassword = "$y$j9T$AswYQmV3RnIe.flXcPa6D0$5H8KZ2I6xy1TZsdjbVOvBv6kiqyW6xCPDHPbywzvC91"; # generate with `mkpasswd`
  };

  # Enable the OpenSSH daemon.
  services.openssh.enable = true;

  # I use neovim as my text editor, replace with whatever you like
  environment.systemPackages = with pkgs; [
    neovim
    wget
  ];

  # allows the use of flakes
  nix.extraOptions = ''
    keep-outputs = true
    keep-derivations = true
    experimental-features = nix-command flakes
  '';

  # this allows you to run `nixos-rebuild --target-host robocup@this-machine` from
  # a different host. not used in this tutorial, but handy later.
  nix.settings.trusted-users = [ "robocup" ];

  # ergonomics, just in case I need to ssh into
  programs.zsh.enable = true;
  environment.variables = {
    SHELL = "zsh";
    EDITOR = "neovim";
  };
}
