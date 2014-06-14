#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 12 10:17:55 2014

Simple N-body simulator game written with Python 2.7 and pygame

Rouge earth is at center of system
LMB hold, drag, and release to launch new body into orbit
MMB hold for orbit persistance
RMB to quit
Game exits when earth leaves system

@author: madengr
"""

import numpy as np
import pygame
import sys

# Create Body class
class Body:
    def __init__(self, mass, velocity, position, color, radius):
        """Initialize body
        
        Keyword arguments:
        m -- mass (kg)
        v -- initial velocity (m/s) numpy array
        p -- initial position (m/s) numpy array
        c -- color tuple (r, g, b)
        r -- radius (m)
        """               
        self.m = mass
        self.v = velocity
        self.p = position
        self.c = color
        self.r = radius
       
        self.px = np.vdot(self.p, [1, 0])
        self.py = np.vdot(self.p, [0, 1])
        
        # Adaptive step time parameters
        # Tweak by hand for max speed with low error
        self.hmax = 1000 # Maximum time step
        self.h = self.hmax
        self.lte_tolerance = 10000 # Local truncation error tolerance
        
    def force(self, targets):
        """Compute total force on body from list of target bodies
        
        Keyword arguments:
        targets -- the list of target bodies to compute force
        """
        G = 6.67E-11 # Gravitational constant (m^3*kg^-1*s^-2)        
        force = 0        
        sum_force = 0
        for target in targets:        
            if target == self:
                pass # Don't compute force on self
            else:
                force = G * self.m * target.m * (target.p - self.p)
                force = force / np.linalg.norm(target.p - self.p) ** 3 
            sum_force = sum_force + force
        return sum_force
                
    def update(self, targets):
        """Update body's position and velocity
        using Huen's method with adaptive time step from 
        Udacity Differential Equations in Action, Lesson 2
        
        Keyword arguments:
        targets -- the list of target bodies to compute force
        """        
        # Euler's method        
        fe = self.force(targets)       
        pe = self.p + self.h * self.v      
        ve = self.v + self.h *  fe / self.m
        
        # Huen's method
        self.p = self.p + self.h * (self.v + ve) / 2
        self.v = self.v + self.h * (fe + self.force(targets)) / (2 * self.m)
        
        # Local truncation error
        lte = np.linalg.norm(pe - self.p) + self.h*np.linalg.norm(ve - self.v)        
        hnew = self.h * np.sqrt(self.lte_tolerance / lte )
        
        # Limit maximum timestep for large masses
        if hnew > self.hmax:
            self.h = self.hmax
        else:
            self.h = hnew
        
        # Update instance variables for position plotting
        self.px = np.vdot(self.p, [1, 0])
        self.py = np.vdot(self.p, [0, 1])
    
    def collision(self, targets):
        """Check for collision from list of target bodies
        
        Keyword arguments:
        targets -- the list of target bodies
        """
        
        for target in targets:        
            if target == self:
                # Check if self position is outside the space                
                if np.linalg.norm(self.p) > spacemax:
                    return True
                    
            elif np.linalg.norm(target.p - self.p) < (target.r + self.r):
                return True
        
        return False
        
def rndcolor():
    r = np.random.randint(0, 255)
    g = np.random.randint(0, 255)
    b = np.random.randint(0, 255)
    return (r, g, b)

black = (0, 0, 0)
blue = (0, 0, 255)
white = (255, 255, 255)

# Setup screen
screen_width = 1000
screen_height = 1000
pygame.init()
screen = pygame.display.set_mode( (screen_width, screen_height) )
pygame.display.flip()

# Misc constants
spacemax = 170E6
vmax = 10E3

# Earth at center
bodies = [Body(5.97E24, np.array([0, 0]), np.array([0, 0]), blue, 32E5)]

# Spacecraft in GEO orbit
bodies.append( Body(1E20, np.array([4E3, 0]), np.array([0, 42E6]), rndcolor()\
, 16E5) )


# Update body orbits
def update_orbits():    
    # Send each body the list of bodies for force calculations    
    for body in bodies:
        body.update(bodies)
        # Map position to screen pixels and draw dot       
        x = int( ( body.px/spacemax +1 ) * screen_width / 2)
        y = int( (-body.py/spacemax +1 ) * screen_height / 2)
        r = int( ( body.r / spacemax )  * screen_width / 2 )
        pygame.draw.circle(screen, body.c, (x, y), r)
    
    # Send each body, but the first, list of bodies for collision check
    for body in bodies[1:]:    
        if body.collision(bodies):
            bodies.remove(body)
    
    # Flip display to update after all bodies drawn
    pygame.draw.circle(screen, white, (screen_width/2, screen_height/2)\
    , screen_height/2, 1)    
    pygame.display.flip()
        
        
while 1:   
    
    update_orbits()
    
     # Exit if Earth drifts outside space
    if np.linalg.norm(bodies[0].p) > spacemax:
        sys.exit(0)

    # Check for user events    
    pygame.event.pump() 
       
    # If MMB press then don't black screen to erase orbits   
    if pygame.mouse.get_pressed() [1]:
        pass
    else:
        screen.fill(black)
    
    # If RMB then exit
    if pygame.mouse.get_pressed() [2]:
        sys.exit(0)
    
    # If LMB then insert new body
    if pygame.mouse.get_pressed() [0]:
        # Get mouse position for sling shot start point        
        sling_start = pygame.mouse.get_pos()
        while pygame.mouse.get_pressed() [0]:
            pygame.event.pump()
            sling_stop = pygame.mouse.get_pos()
            pygame.draw.line(screen, white, sling_start, sling_stop, 1)
            pygame.draw.circle(screen, white, sling_start, 5)
            update_orbits()
            screen.fill(black)

        px = spacemax * (2.0 * sling_start[0]/screen_width - 1)
        py = -spacemax * (2.0 * sling_start[1]/screen_height - 1)
        vx = vmax * (sling_start[0] - sling_stop[0]) / screen_width
        vy = -vmax * (sling_start[1] - sling_stop[1]) / screen_height
        body = Body(1E20, np.array([vx, vy]), np.array([px, py]), rndcolor()\
        , 16E5)
        bodies.append(body)        
        






        
