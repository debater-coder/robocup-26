{ pkgs, ... }:

{
  imports = [
    ./hardware-configuration.nix
  ];

  # Use the extlinux boot loader. (NixOS wants to enable GRUB by default)
  boot.loader.grub.enable = false;
  # Enables the generation of /boot/extlinux/extlinux.conf
  boot.loader.generic-extlinux-compatible.enable = true;

  # https://github.com/NixOS/nixpkgs/issues/154163
  nixpkgs.overlays = [
    (final: super: {
      makeModulesClosure = x:
        super.makeModulesClosure (x // { allowMissing = true; });
    })
  ];

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
    gh
    git
    libraspberrypi
    raspberrypi-eeprom
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

  hardware = {
    raspberry-pi."4".apply-overlays-dtmerge.enable = true;
    raspberry-pi."4".fkms-3d.enable = true;
    deviceTree = {
      enable = true;
    };
  };

  # desktop
  services.xserver = {
    enable = true;
    displayManager.lightdm.enable = true;
  };

  services.desktopManager.gnome.enable = true;

  system.stateVersion = "25.11";
}
