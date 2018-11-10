# Overview
Kalman filters are a class of algorithms used to filter noisy signals, generate non-observable states, and predict future states in robotics. They allow roboticists to take sensors that measure position and derive velocity measurements. They can also filter multiple noisy inputs to produce a stable output. This project is built to allow the students to learn in depth about kalman filters, implement one from the ground up, and build a simple comparison of various forms of odometry using a Neato botvac with an upward facing camera, encoders, and an IMU.

# Learning Goals
##### Connor
- Pre-plan and iteratively rescope over the course of the project - Be deliberate about process and timescale
- Understand a new algorithm to the point of teaching others
- Create hands-on learning materials useable by other Oliners

##### Charlie
- Understand a new algorithm to the point of teaching others
- Well scaffolded project
- Focus on core conceptual understanding

# Deliverables
- Learning materials designed to show students why kalman filters are useful, how they work, and when to implement them, as well as provide further resources in an accessible manner. Some example code for basic implementation OR simple example. 
  - Google docs ipython notebook with separate steps for each aspect of the filter
  - Matlab notebook with separate steps for each aspect of the filter
  - Build a problem set about kalman filters with a solutions file
- Simple robot localizing: Drive square, straight line, spin in place and determine error. Compare with ceiling localization as ground truth. Compare with odometry localizing and encoders (separately) as baseline comparison.

# Project Schedule
## Week 1
- Class 20 (F 11/9)
  - Write Project Proposal
  - Find Reference Texts
  - For next time: Choose one text, take notes, and understand how that text describes filters
  - On radar: Read about linear vs. nonlinear
  
## Week 2
- Class 21 (T 11/13)
  - Synthesize information from texts
  - Begin basic KF: Define context of example (scaffold)
  - Aside: Ask Paul about IMU implementation, covariance matrices
  - For next class: Finish writing out scaffolding and take a stab at stuff
- Class 22 (F 11/16)
  - Write more sections of example and corner Paul
  - Update schedule
  - For next time: Get some sweet R&R
  
## Thanksgiving Break

## Week 3
- Class 23 (T 11/27)
  - Start teaching notebook: Rough draft, have someone look, revise
  - Alongside notebook: Introductory scaffolding for Kalmin Filter
- Class 24 (F 11/30)
  - Take dataset of IMU, encoder, ceiling
  
## Week 4
- Class 25 (T 12/4)
  - Continue teaching notebook
- Class 26 (F 12/7)

## Week 5
- Finals week
  - Slop time wooo
- Final Demo
- Make teaching notebook: Rough draft, have someone look, revise
- Take dataset of IMU, encoder, ceiling

# Resources
1. Hall, David L. _Mathematical Techniques in Multisensor Data Fusion_ Boston: Artech House 2004. Print
2. RLabbe Textbook ([link](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python))
3. Mathworks.com ([link](https://www.mathworks.com/videos/series/understanding-kalman-filters.html))
4. Bzarg.com ([link](http://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/))
5. Probabilistic Robotics ([link](https://docs.ufpr.br/~danielsantos/ProbabilisticRobotics.pdf))
6. Robotsforrobiticists.com ([link](http://robotsforroboticists.com/kalman-filtering/))
