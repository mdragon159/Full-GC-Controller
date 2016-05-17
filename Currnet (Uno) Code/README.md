# Current (Uno) Code
Can act as a host/console as well as an independent third party observing transactions (albeit doesn't yet )

# Very large thanks to...
Andrew Brown and his very amazing [Gamecube-N64-Controller repo](https://github.com/brownan/Gamecube-N64-Controller)!

Saved me a LOT of work. I don't know how to stress how much time this repo saved me! And it worked absolutely perfectly in every respect as expected for reading from a GC controller (and for writing to the data line pretty much perfectly)!

Note: Took a very large chunk of Brown's (brownan) repo, ripped off the N64 portions, and am now in the process of modifying to act as an extensive GC solution (so can act as a console, as a controller, as well as simply an audience member viewing the transactions of a separate console and controller).

# Current Status
Summary: As_Console and As_ThirdParty modes reasonably work
 * As_Console = Acting as a host/console and reading data from a gamecube controller
 * As_ThirdParty = Reading transactions as an independent third party between a separate console and controller

The file called "GC As-Console Reader Waveform (part of initial reading command sequence).BMP" (TODO: figure out how to link images and other files within repo) contains a picture of my oscilliscope with the current as-console GC controller reader waveform (and only contains the beginning portion of the read-from-controller command).

Note that an As_Console mode consists of simply the Arduino and the controller, which necessitates that a) there is a pull-up resistor between the data line and the 3.3V line and b) the power line (as described in browman's repo linked above). The As_ThirdParty (and As_Controller) setup, on the other hand, does NOT need a pull-up resistor, as the separate, independent console will be handling that.

# Next Steps
(Current status and this area will be completed once these steps are reliably completed)
 * Have the Arduino comprehend commands from a console
 * Separate data read on the data line into sections (commands, GC controller data, etc), useful for both As_Controller and As_ThirdParty
 * Have the Arduino respond as a controller to a console's commands (As_Controller mode)
 
Once above are done and work reasonably well with different conditions, I'll work on expanding it into a flexible library and creating a full application with more advanced features (such as reading GC controller sequences on the data line then repeating them at the touch of pushbuttons).
 
Other necessary steps:
 * Fix licensing for Andrew Brown's repo and material!