# Melvin

This has been superceeded by a higher level project, encompassing the work done with Melvin, as well as my work related to a DIY fuel injection ECU, retrofitted automatic climate controls, data logging, and more. Find that repository here: [Retromod-In-Car-Systems](https://github.com/grobertson/Retromod-In-Car-Systems)
-----

Extending my Volvo with micro-controllers interconnected via serial network. This is a very early WIP, at this stage it's more like a thought experiment taken a little too far and an excuse to dive deeper into circuit design. 

#Melvin_
The master controller (000). Sends messages to a slave device controlling light relays.
The very very basic message format between nodes is:

    001:1:Arbitrary Data;
    
Where fields are seperated by colon, messages terminated with semi-colon. Fields are <Device Class>:<Device Number>:<Data>;

Currently implements an LCD display with barometer/thermostat and status indicators for four (4) banks of two relays. A lighting controller (or relay controller, we don't care what's being switched) is device class 001, and this is the first device, number 1. The relay controller messages are very simple, with each relay being represented by a 1 or 0 in a string of 8 chars. This is then used on the remote end like any array of chars, we inspect each in the series and set the relay's state on or off.

The point of this? Experimentation with several disciplines, with a goal of eventually implementing a host of modern and post-modern automotive convenience features into an antique/retro car with deeply integrated driver controls. 
