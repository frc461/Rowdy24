# Development Guide

This documentation provides a general scaffold for developers in the process of developing the software of an FRC robot.

## Step 1: Primitive
- [ ] Codebase (`Robot.java`, `RobotContainer.java`, `Constants.java`)
  - [ ] Utility files
  - [ ] Imported vendor dependencies
- [ ] Framework — Subsystems
  - [ ] Motor configurations
  - [ ] Subsystem scaffold (class declaration, getter functions)
- [ ] Framework — Vision
  - [ ] Test camera data & brainstorm desired metrics for measurement
  - [ ] Optimize localization
- [ ] Exception: Drivetrain, Camera data
  - [ ] Feedforward & PID optimization for effective driving

## Step 2: Operational
- [ ] Basic control
  - [ ] Teleop manual control for all subsystems with joysticks
  - [ ] Drivetrain-only autonomous paths
- [ ] Framework — Commands
  - [ ] Brainstorm primitive commands
  - [ ] Brainstorm secondary/more complex commands
  - [ ] Brainstorm interaction between subsystems

## Step 3: Optimization
- [ ] Advanced control
  - [ ] Optimize PID values for clarified positional or velocity identification, i.e., presets
  - [ ] Button bindings for certain, more sophisticated/automated actions
- [ ] Advanced Vision
  - [ ] Integrate camera data/machine learning to implement automated tasks

## Step 4: Autonomous
- [ ] Optimize auto paths with automated commands
- [ ] Vary paths for strategy
- [ ] Integrate camera data/machine learning to dynamically configure paths