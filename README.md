# Multi-Body Dynamics and Kinematic Analysis of a Track-Focused Suspension System

## Overview

This project focuses on the development, simulation, and analysis of a track-focused race car suspension system. The objective is to study vehicle ride and handling characteristics using mathematical models, suspension geometry design, and dynamic simulations.

The project includes:

- Quarter Car Model
- Full Car Model
- Bicycle Model
- Curb-Hit Analysis
- High-Speed Cornering Analysis
- Tire Load Transfer Analysis
- Double Wishbone Suspension Design
- Hardpoint Coordinate Development
- AutoCAD Geometry Modeling

---

## Project Objectives

- Develop mathematical models for ride and handling analysis.
- Study vertical, roll, and pitch dynamics of the vehicle.
- Analyze transient response during curb-hit disturbances.
- Investigate handling behavior during high-speed cornering.
- Perform tire load transfer analysis under 1.2g cornering conditions.
- Design a double wishbone suspension geometry using 3D hardpoints.
- Create a CAD model of the suspension system.
- Prepare the design for structural analysis and optimization.

---

## Methodology

```text
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
```

---

## Models Developed

### Quarter Car Model

Purpose:

- Analyze ride comfort and vertical vehicle dynamics.
- Study sprung and unsprung mass behavior.
- Evaluate suspension response to road disturbances.

Outputs:

- Body displacement
- Suspension deflection
- Wheel displacement

---

### Full Car Model

Purpose:

- Analyze complete vehicle body dynamics.
- Study heave, roll, and pitch motions.
- Evaluate vehicle response during curb-hit events.

Degrees of Freedom:

- Heave
- Roll
- Pitch
- Four wheel vertical motions

Outputs:

- Body displacement
- Roll angle
- Pitch angle

---

### Bicycle Model

Purpose:

- Analyze vehicle handling and stability.
- Simulate steering maneuvers.
- Evaluate cornering performance.

Inputs:

- Steering angle

Outputs:

- Side-slip angle
- Yaw rate
- Lateral acceleration

---

## High-Speed Cornering Analysis

A 2-DOF Bicycle Model was used to simulate a high-speed cornering maneuver.

### Vehicle Parameters

| Parameter | Value |
|------------|--------|
| Vehicle Mass | 300 kg |
| Yaw Inertia | 120 kg·m² |
| Distance CG → Front Axle | 0.8 m |
| Distance CG → Rear Axle | 0.8 m |
| Vehicle Speed | 15 m/s |
| Front Cornering Stiffness | 45000 N/rad |
| Rear Cornering Stiffness | 35000 N/rad |
| Steering Input | 5° (0.0873 rad) |

### Results

- Achieved approximately 1.2g lateral acceleration.
- Stable yaw rate response observed.
- Dynamic tire load transfer evaluated.

---

## Tire Load Transfer Analysis

Load transfer during cornering was calculated using:

ΔW = (m × ay × h) / t

Where:

- m = Vehicle mass
- ay = Lateral acceleration
- h = CG height
- t = Track width

Outputs:

- Front Left Wheel Load
- Front Right Wheel Load
- Rear Left Wheel Load
- Rear Right Wheel Load

---

## Suspension Design

### Suspension Type

Double Wishbone Suspension

### Alignment Targets

| Parameter | Value |
|------------|--------|
| Camber | -2° |
| Caster | 6° |
| KPI | 8° |
| Toe | -0.5° |

### Hardpoints Developed

- Upper Control Arm Front
- Upper Control Arm Rear
- Upper Ball Joint
- Lower Control Arm Front
- Lower Control Arm Rear
- Lower Ball Joint
- Wheel Center
- Tie Rod Inner
- Tie Rod Outer

---

## Software Used

- MATLAB
- Simulink
- AutoCAD
- Abaqus (Planned)

---

## Current Progress (Mid-Evaluation)

### Completed

- Literature Survey
- Quarter Car Model Development
- Full Car Model Development
- Curb-Hit Analysis
- Bicycle Model Development
- 1.2g High-Speed Cornering Analysis
- Tire Load Transfer Analysis
- Suspension Hardpoint Design
- AutoCAD Suspension Geometry

### Future Work

- Camber Gain Analysis
- Roll Center Migration
- Track Width Variation Analysis
- Abaqus Structural Analysis
- Suspension Optimization
- Final Validation

---

## Repository Structure

```text
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
```

---

## Author

**Raavi Chandra Siddhartha** 
**Bhairam Hasini Mourya**
Indian Institute of Technology Indore

---

Upcoming phase:
- Structural analysis and optimization using Abaqus
