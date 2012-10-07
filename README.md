#FixBot
http://www.thingiverse.com/thing:29745
A Servo controlled FixBot with a 3D printed chassis.  The repo contains various bits of source code for the FixBot.

##States
An arduino based state machine that controls the FixBot.  There are several states it can enter, with each state having its own types of action.  This file is meant for FixBots that are mounted on a wall.  There is also logic in place for a LuxMeter to be plugged into Digital i/o pin 2 on the arduino board; if the lux sensed by a lux meter is recorded as less than 10000, the bot will go to sleep.

##Learning
Designed to interface with a genetic algorithm to evolve gaits for a FixBot.  Each servo has a start angle and an end angle, which are alternated between.  VIA Serial communication, a host can send new angles for the Servos to move at.  
