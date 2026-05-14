#set text(
  font: "Rubik",
)
#set heading(numbering: "1.")
#show link: underline
#set page(header: context [
  High Scorers 2026 #h(1fr) #counter(page).display("1")
])
#align(center, text(17pt)[
  *High Scorers 2026 \
  Technical Description Paper*
])

#align(center)[Hamzah Ahmed, Jasper Zheng, Ray Chen, Joshua Campbell]\

#outline()
#pagebreak()

This document contains a description of our team's progress in building a soccer robot for RCJA 2025 Open Soccer. The first section in this document gives a brief description of our robot and process as prescribed by the Technical Description Paper template. The rest of the document covers the technical details of this project.

= Overview
== Team Information
/ Challenge/Division: Open Soccer
/ Team Name: High Scorers
/ School: Sydney Boys High School
/ State: NSW

#table(
  columns: 2,
  [*Team Member*], [*Role*],
  [Hamzah Ahmed], [Software, Team Lead],
  [Jasper Zheng], [Mechanical Design],
  [Ray Chen], [Electronics],
  [Joshua Campbell], [Strategy],
)

== Robot Properties
The star ratings follow the format prescribed by the TDP template.
A detailed technical description of these components will be contained in the
later sections.
=== Software
_Generative AI (for example LLMs), were not used to generate any software for this project._

The following table lists external libraries/tools used in our software.
#table(
  columns: (2fr, 3fr, auto),
  [*Component*], [*Software/Library*], [*Star Rating (1-5)*],
  [Pico Firmware], [MicroPython], [5],
  [Debug UI], [React (TypeScript)], [5],
  [Soccer controller (running on Raspberry Pi) language], [Python], [5],
  [Vector library], [scipy/numpy], [5],
  [Behaviour tree library], [py-trees], [5],
  [Serial interface library], [pyserial], [2],
  [Debug UI server], [aiohttp], [5],
  [Bluetooth Low Energy Communication (not yet integrated at time of writing)],
  [Bless (server) / Bleak (client)],
  [N/A],
)

=== Hardware
This table contains the major hardware components used in this project.
#table(
  columns: (1fr, 2fr, auto),
  [*Component*], [*Part Number (hyperlinked)*], [*Star Rating (1-5)*],
  [Microcontroller],
  [#link("https://core-electronics.com.au/raspberry-pi-pico-h-with-headers.html")[Raspberry Pi Pico]],
  [5],

  [5V Regulator], [#link("https://core-electronics.com.au/dc-dc-power-module-25w.html")[DC-DC Power Module 25W]], [5],
  [Mecanum Wheels],
  [#link("https://core-electronics.com.au/mecanum-wheel-kit-48mm-4-wheels.html")[Mecanum Wheel Kit]],
  [3],

  [Motor (for wheels and dribbler)],
  [#link(
    "https://core-electronics.com.au/n20-dc-gear-motor-magnetic-hall-encoder-all-metal-gearbox-high-precision-reduction-motor-with-l-shaped-6pin-connector.html",
  )[N20 DC Gear Motor with Magnetic Hall Encoder]],
  [3],

  [Battery],
  [#link(
    "https://hobbyking.com/en_us/turnigy-battery-5000mah-3s-25c-lipo-pack-xt-90.html?___store=en_us",
  )[Turnigy 5000mAh 3S 25C Lipo Pack W/XT-90]],
  [4],

  [Motor driver],
  [#link("https://core-electronics.com.au/makerverse-motor-driver-2-channel.html")[Makerverse Motor Driver 2 Channel]],
  [4],

  [Raspberry Pi 4], [#link("https://www.raspberrypi.com/products/raspberry-pi-4-model-b/")[Raspberry Pi 4]], [5],
  [Servo (for kicker)], [#link("https://www.makerstore.com.au/product/mg996r-servo-90-rotation/")[MG996R]], [2],
  [Custom PCB], [#link("https://www.pcbway.com/")[PCBWay]], [5],
)


== Collaboration

For CAD, *Onshape* served useful through its collaboration features. CAD documents
are stored in the cloud with version history, and its component features allow
for complex assemblies to be broken down into smaller components which could be
assigned to each person.

For code, we used *GitHub* to store our code in a centralised place, where it can
be accessed by all team members. GitHub's pull request features provide a useful
way for team members to peer review our code.

Our team's collaborative process involved breaking down the system into hardware and software *subassemblies* (the specifics of these assemblies are included in the System Architecture section).

== Key Achievement & Area for improvement

Our software uses a single forward facing camera and wheel odometry to obtain precise position and velocity data. We use closed-loop control (PD) to keep the robot on its computed trajectory. Our use of behaviour trees allowed us to implement strategies involving priorities which interrupt each
other, allowing for more reactive gameplay.

Our key areas for improvement is the kicker. Due to space and time limitations we used a servo connected to a kicking lever to kick the ball. The servo is jittery and only provides angle input, and its low speed makes it ineffective at kicking the ball. In general, space limitations on the robot led us to make many compromises.

== Photos and Design Documentation
