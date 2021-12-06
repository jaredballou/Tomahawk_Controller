# Tomahawk Controller

This project is a first attempt for me to convert a Dart Zone Tomahawk 60 into
fully automatic operation. This is more complex than most magazine/flywheel
blasters, because the 60 dart drum which must be indexed as part of the
operating cycle is physically large and heavy. Interference between the dart
pusher and the feed pawl can cause major jams, and there doesn't seem to be
an easy way to run both mechanical operations off of a single electromechanical
element.

To address that, I've put in a pair of solenoids, and set the timing very slow
as a starting point to give myself the best chance of success. Currently, I'm
using a Narfduino as the microcontroller, with an I2C .96" OLED display, LiPo
3S1P 11.1v batteries, and all internal connections equipped with either JST
2-pin connectors or a 4-cable DuPont connection for the display.

The code is absolutely not good, this is my first real Arduino project and I
already have a lot of things I want to change. The delays and way that the
logic controls the blaster is very clunky, and wouldn't be good for a final
product given all the performance and style problems.

That said, it does work, so I'm using it to help figure out the hardware
problems (mostly, the feed advance solenoid isn't strong enough). I will do
a refactor and optimization pass when/if I am able to get the physical blaster
to operate reliably.

There are a number of photos and videos of the progress at 
[Google Photos](https://photos.app.goo.gl/SdEb1wXE3UN2jBCg6)

## TODO

- [X] Initial POC code to activate elements in sequence.
- [X] Display support with fire mode, battery level, ammo count.
- [ ] Configurable elements to add into display for element status, voltage,
and other metrics.
- [ ] Rework delays and timings to use timer elements.
- [ ] Change pin readings from individual reads with their own check/set
functions to the JC_Buttons or direct pin access.
- [ ] Get feed advance working reliably.
- [ ] Add "debug" mode to output real-time operational status to display.
- [ ] Create persistent storage to maintain preferences/settings across
microcontroller reboots.
- [ ] Add on-blaster buttons to allow setting things like magazine size,
flywheel speed, element timings, etc.
- [ ] Look into adding in-blaster charging, using extensions to balance plug
and battery terminals.

![Current Spaghetti Wiring](https://lh3.googleusercontent.com/pw/AM-JKLVGqBzzdoMFDMm0hFDGryKAcebPCSU7-E1aJwL93kdH1HKX_m_adok4yg8bi0f-uJ5BlUx2utB5g9NcjFVVyaBxZw3ZK2jcTb3zQSWJmKacy3XuAXmutaujosGXkxwfBeCiI6whJLJOfaE89qYOaJuGew=w1059-h794-no?authuser=0)
