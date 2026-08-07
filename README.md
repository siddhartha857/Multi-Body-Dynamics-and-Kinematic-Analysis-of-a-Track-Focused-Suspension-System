# Multi-Body Dynamics and Kinematic Analysis of a Track-Focused Suspension System

# Multi-Body Dynamics and Kinematic Analysis of a Track-Focused Suspension System

## Overview

This repository presents the mathematical modeling, simulation workflows, suspension kinematic analysis, and dynamic performance analysis of a track-focused race car suspension system.

The project covers vehicle ride dynamics, full vehicle body dynamics, handling analysis, transient curb-hit response, tire load transfer, and the development of a double wishbone suspension geometry using three-dimensional hardpoints and a SolidWorks wireframe model.

---

## Technical Methodology

```text
Literature Survey & Target Definition
                 ↓
Quarter Car Model
                 ↓
Full Car Model
                 ↓
Curb-Hit Transient Response Analysis
                 ↓
Bicycle Model
                 ↓
High-Speed Cornering Analysis
                 ↓
Tire Load Transfer Analysis
                 ↓
Suspension Hardpoint Development
                 ↓
SolidWorks Wireframe Model
                 ↓
Suspension Kinematic Analysis
                 ↓
Roll Centre Migration Analysis
                 ↓
Motion Ratio Analysis
                 ↓
Final Validation
```

---

## Mathematical Models Developed

### 1. Quarter Car Model

The Quarter Car Model is developed to study the vertical dynamics of the vehicle.

**Purpose:**

* Study sprung and unsprung mass behavior.
* Analyze suspension response to road disturbances.
* Study wheel and body vertical motion.
* Evaluate suspension deflection.

**Outputs:**

* Sprung mass displacement
* Unsprung mass displacement
* Suspension deflection
* Wheel displacement
* Body vertical response

---

### 2. Full Car Model

The Full Car Model is developed to study the complete vehicle body dynamics.

**Purpose:**

* Study vehicle heave motion.
* Study roll motion.
* Study pitch motion.
* Analyze individual wheel responses.
* Investigate vehicle response to asymmetric road disturbances.

**Outputs:**

* Heave displacement
* Roll angle
* Pitch angle
* Wheel vertical responses
* Suspension responses

---

### 3. Bicycle Model

The Bicycle Model is developed to study the lateral dynamics and handling behavior of the vehicle.

**Purpose:**

* Study steering response.
* Analyze yaw dynamics.
* Evaluate vehicle side-slip behavior.
* Study lateral vehicle response during cornering.

**Outputs:**

* Yaw rate
* Side-slip angle
* Lateral acceleration

---

## Dynamic Analysis

### Curb-Hit Analysis

A transient curb-hit analysis is performed to study the response of the vehicle when subjected to a sudden track disturbance.

The analysis focuses on:

* Vertical vehicle response
* Roll response
* Pitch response
* Wheel response
* Suspension response

---

### High-Speed Cornering Analysis

The Bicycle Model is used to study vehicle behavior during high-speed cornering.

The analysis focuses on:

* Yaw behavior
* Side-slip behavior
* Lateral acceleration
* Vehicle handling response

---

### Tire Load Transfer Analysis

Tire load transfer is analyzed to study the redistribution of vertical loads between the wheels during cornering.

The analysis focuses on:

* Inner and outer wheel loading
* Front and rear load distribution
* Dynamic tire loading
* Effect of lateral acceleration on wheel loads

---

## Suspension Geometry

### Double Wishbone Suspension

A double wishbone suspension geometry is developed using three-dimensional hardpoint coordinates.

The hardpoint layout defines the suspension arms, steering linkage, pushrod mechanism, rocker, and damper mounting locations.

### Hardpoints Developed

* Wheel Center
* LCA Outboard
* UCA Outboard
* LCA Inboard Front
* LCA Inboard Rear
* UCA Inboard Front
* UCA Inboard Rear
* Pushrod–LCA
* Pushrod–Rocker
* Rocker Pivot
* Damper–Rocker
* Damper–Chassis
* Tie Rod Inner
* Tie Rod Outer

These hardpoints are used to define the suspension geometry and perform the subsequent kinematic analysis.

---

## Suspension Kinematic Analysis

The developed suspension geometry is analyzed throughout suspension travel to study its kinematic behavior.

The analysis includes:

* Camber variation
* Roll centre location
* Roll centre migration
* Motion ratio

---

### Roll Centre Migration

Roll centre migration is analyzed to determine how the suspension roll centre changes with suspension movement.

This helps evaluate the geometric behavior of the suspension during wheel travel.

---

### Motion Ratio Analysis

Motion ratio is evaluated to determine the relationship between wheel movement and suspension movement.

This helps understand the effect of suspension geometry on the effective suspension behavior.

---

## SolidWorks Wireframe Development

The developed suspension hardpoints were used to create a **wireframe representation of the suspension geometry in SolidWorks**.

The wireframe model is used to represent:

* Suspension hardpoints
* Control arm geometry
* Steering linkage
* Pushrod and rocker geometry
* Damper mounting locations
* Spatial relationship between suspension components

The SolidWorks work in this project is limited to **wireframe geometry development** and does not include a complete 3D component or suspension assembly.

---

## Software & Tools Used

* **MATLAB:** Mathematical modeling, numerical calculations, and vehicle dynamics simulation.
* **Simulink:** Dynamic system modeling and transient vehicle simulations.
* **SolidWorks:** Suspension hardpoint visualization and wireframe geometry development.

---

## Current Progress

### Completed

* Literature Survey
* Quarter Car Model
* Full Car Model
* Curb-Hit Analysis
* Bicycle Model
* High-Speed Cornering Analysis
* Tire Load Transfer Analysis
* Suspension Hardpoint Development
* Double Wishbone Suspension Geometry
* SolidWorks Wireframe Model
* Suspension Kinematic Analysis
* Roll Centre Migration Analysis
* Motion Ratio Analysis

---

## Repository Structure

```text
Project/
│
├── Quarter_Car_Model/         # Vertical ride dynamics
├── Full_Car_Model/            # Heave, roll, pitch & wheel dynamics
├── Bicycle_Model/             # Vehicle handling & yaw dynamics
├── Curb_Hit_Analysis/         # Transient curb-hit analysis
├── Tire_Load_Transfer/        # Dynamic tire load transfer
├── Hardpoint_Design/           # Suspension hardpoints & coordinates
├── Kinematic_Analysis/         # Suspension kinematic analysis
├── Roll_Centre_Migration/      # Roll centre analysis
├── Motion_Ratio/               # Motion ratio analysis
├── Results/                    # Simulation and analysis outputs
└── README.md                   # Project documentation summary
```

---

## Project Scope

The project establishes a workflow from **vehicle dynamics modeling to suspension hardpoint development and kinematic analysis**.

The developed models are used to study vehicle ride, handling, transient response, and tire load transfer, while the suspension hardpoints and SolidWorks wireframe geometry are used to investigate suspension kinematics, roll centre migration, and motion ratio.

---

## Authors

* **Raavi Chandra Siddhartha** — Indian Institute of Technology Indore
* **Bhairam Hasini Mourya** — Indian Institute of Technology Indore

---
Indian Institute of Technology Indore


