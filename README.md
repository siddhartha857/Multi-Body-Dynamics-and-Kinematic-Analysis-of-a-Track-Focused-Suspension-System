# Multi-Body-Dynamics-and-Kinematic-Analysis-of-a-Track-Focused-Suspension-System
Project Overview

This project focuses on the development, simulation, and analysis of a track-focused race car suspension system. The objective is to study vehicle ride and handling characteristics through mathematical modeling, suspension geometry design, and dynamic simulations.

The project includes:

Quarter Car Model
Full Car Model
Bicycle Model
Curb-Hit Analysis
High-Speed Cornering Analysis
Tire Load Transfer Analysis
Double Wishbone Suspension Design
Suspension Hardpoint Development
AutoCAD Geometry Modeling
Objectives
Develop mathematical models for ride and handling analysis.
Study vertical, roll, and pitch dynamics of the vehicle.
Analyze vehicle response during curb-hit disturbances.
Investigate handling behavior during high-speed cornering maneuvers.
Perform tire load transfer analysis under 1.2g cornering conditions.
Design a double wishbone suspension geometry using 3D hardpoint coordinates.
Create a CAD model of the suspension system.
Prepare the suspension design for future structural analysis and optimization.
Methodology
Literature Survey
        ↓
Quarter Car Model
        ↓
Full Car Model
        ↓
Curb-Hit Analysis
        ↓
Bicycle Model
        ↓
High-Speed Cornering Analysis
        ↓
Tire Load Transfer Analysis
        ↓
Suspension Hardpoint Design
        ↓
AutoCAD Geometry Development
        ↓
Abaqus FEA (Future Work)
        ↓
Optimization and Validation
Models Developed
1. Quarter Car Model

Purpose:

Analyze ride comfort and vertical vehicle dynamics.
Study sprung and unsprung mass behavior.
Evaluate suspension response to road disturbances.

Outputs:

Body displacement
Suspension deflection
Wheel displacement
2. Full Car Model

Purpose:

Analyze complete vehicle body dynamics.
Study heave, roll, and pitch motions.
Evaluate vehicle response during curb-hit events.

Degrees of Freedom:

Heave
Roll
Pitch
Four wheel vertical motions

Outputs:

Body displacement
Roll angle
Pitch angle
3. Bicycle Model

Purpose:

Analyze vehicle handling and stability.
Simulate steering maneuvers.
Evaluate cornering performance.

Inputs:

Steering angle

Outputs:

Side-slip angle
Yaw rate
Lateral acceleration
High-Speed Cornering Analysis

A 2-DOF Bicycle Model was used to simulate a high-speed cornering maneuver.

Vehicle Parameters
Parameter	Value
Vehicle Mass	300 kg
Yaw Inertia	120 kg·m²
Distance CG → Front Axle	0.8 m
Distance CG → Rear Axle	0.8 m
Vehicle Speed	15 m/s
Front Cornering Stiffness	45000 N/rad
Rear Cornering Stiffness	35000 N/rad
Steering Input	5° (0.0873 rad)
Results
Lateral Acceleration ≈ 1.2g
Stable Yaw Rate Response
Dynamic Tire Load Transfer
Tire Load Transfer Analysis

The load transfer during a 1.2g cornering maneuver was calculated using:

ΔW=
t
ma
y
	​

h
	​


Where:

m = Vehicle mass
ay = Lateral acceleration
h = CG height
t = Track width

Outputs:

Front Left Wheel Load
Front Right Wheel Load
Rear Left Wheel Load
Rear Right Wheel Load
Suspension Design
Suspension Type

Double Wishbone Suspension

Alignment Targets
Parameter	Value
Camber	-2°
Caster	6°
KPI	8°
Toe	-0.5°
Hardpoints Developed
Upper Control Arm Front
Upper Control Arm Rear
Upper Ball Joint
Lower Control Arm Front
Lower Control Arm Rear
Lower Ball Joint
Wheel Center
Tie Rod Inner
Tie Rod Outer
Software Used
MATLAB
Simulink
AutoCAD
Abaqus (planned)
Current Status (Mid-Evaluation)
Completed
Literature Survey
Quarter Car Model
Full Car Model
Curb-Hit Analysis
Bicycle Model
High-Speed Cornering Analysis
Tire Load Transfer Analysis
Suspension Hardpoint Design
AutoCAD Suspension Geometry
Ongoing / Future Work
Camber Gain Analysis
Roll Center Migration
Track Width Variation Analysis
Abaqus Structural Analysis
Suspension Optimization
Final Validation
Repository Structure
Project/
│
├── Quarter_Car_Model/
├── Full_Car_Model/
├── Bicycle_Model/
├── Tire_Load_Transfer/
├── Hardpoint_Design/
├── AutoCAD_Models/
├── Results/
├── Documentation/
└── README.md
Author

Raavi Chandra Siddhartha
Mechanical Engineering
Indian Institute of Technology Indore

License

This project is developed for academic and research purposes.
