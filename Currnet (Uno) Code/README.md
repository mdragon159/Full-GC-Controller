# Current (Uno) Code
Something that works so-so

# Very large thanks to...
Andrew Brown and his very amazing [Gamecube-N64-Controller repo](https://github.com/brownan/Gamecube-N64-Controller)!

Saved me a LOT of work. I don't know how to stress how much time this repo saved me! And it worked absolutely perfectly in every respect as expected for reading from a GC controller (and for writing to the data line pretty much perfectly)!

Note: Took a very large chunk of Brown's (brownan) repo, ripped off the N64 portions, and am now in the process of modifying to act as an extensive GC solution (so can act as a console, as a controller, as well as simply an audience member viewing the transactions of a separate console and controller).

# Current Status
Summary: Can read from a controller but that's it.

Currently only acts as a GC console, according to Andrew Brown's repo. Essentially, ripped out all the N64 portions and carefully tested on a brand new (R3) Arduino Uno. Absolutely works and the timing is spot on.

The file called "GC As-Console Reader Waveform (part of initial reading command sequence).BMP" (TODO: figure out how to link images and other files within repo) contains a picture of my oscilliscope with the current as-console GC controller reader waveform (and only contains the beginning portion of the read-from-controller command).

Note that an "as-console" state consists of simply the Arduino and the controller, which necessitates that a) there is a pull-up resistor between the data line and the 3.3V line and b) the power line

# Next Steps
(Current status and this area will be completed once these steps are reliably completed)
 * Have the Arduino read transactions between a controller and a console as a third-party (pure read mode, doesn't write at all)
 * Have the Arduino comprehend commands from a console
 * Have the Arduino respond as a controller to a console's commands
 
Other necessary steps:
 * Fix licensing for Andrew Brown's repo and material!
