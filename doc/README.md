# Getting Started

This guide will help you get started with an in-depth analysis of 461's Robot Software. The goal of this documentation is help build understanding into what robot software development actually entails and how to develop robot software. Below will include helpful advice and best practices when contributing to and deploying our code.

*NOTE: The software and imported libraries for this project are likely outdated. Using this project as a template for a modern project might not work.*

## Deployment

Before anything, make sure:

- [WPILib](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html) is installed
- [Git for Windows](https://git-scm.com/download/win) or [Git (Linux/macOS)](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git) is installed
- Network on device is connected to your robot

Now we are going to retrieve the code from this repository to deploy it.

1. Open your terminal and enter the command: `git clone https://github.com/frc461/Rowdy24.git`
2. Once it finishes cloning, open WPILib VSCode Client and open the cloned folder as a project.
3. Check our [vendor dependencies](lib/VENDOR%20LIBRARIES.md) to confirm that they are installed.
4. Press `ctrl`+`shift`+`P` to open up a VSCode menu, then type in 'Deploy'. Press `enter`.

## Contributing (Version Control)

On any issue/feature that doesn't have a branch yet, create an issue and create a respective branch to work on it. This will help keep the master branch and working tree clean and organized. If a branch already exists for an issue/feature, commit changes relevant to that issue/feature to that branch, and request write access if needed. After any issue/feature is solved/created, generate a pull request (PR) to the main branch.

## Event Best Practices

#### In the pits:
- Keep a programmer on pit crew to manage robot programming and to speak with judges.
- Make as few changes as possible to the code. If a crucial change is needed, *tune/revise* existing code and avoid writing new code as possible, as malfunctions occur more in newly written software.
- Confine to one laptop if possible. FRC events don't have Wi-Fi so Git is inaccessible.
- Write a preflight checklist to *thoroughly evaluate* the robot with before each match. Test every subsystem thoroughly. catching a problem early often decides the result of a match.
- Help alliance partners! Offer assistance before playoffs - this is especially important during earlier events, when software may be less stable.

#### Before/during a match:
- MAKE SURE TO SELECT AN AUTO.
- Check the driver station to ensure that everything including the robot, laptop, and controllers, is connected.
- Record auto in all matches. It's better to base auto revisions/tweaks on a recording instead of recall.

## Explore Documentation

The following links are to documentation extensively explaining the software implementation of Rowdy24. The documentation directory structure is similar to that of the actual software implementation. Hence, the following items:

- [Libraries](lib)
- [Robot](robot)
   - [Commands](robot/commands)
   - [Subsystems](robot/subsystems)

As a to-do list to help beginner programmers, the following pages are recommended to read in order:
- [ ] Follow the [Getting Started](README.md) guide to deploy the code.
- [ ] [Robot](robot) - Readme
- [ ] [Commands](robot/commands) - Readme
- [ ] [Subsystems](robot/subsystems) - Readme
- [ ] [RobotContainer](robot/ROBOT_CONTAINER.md) - Functions and Commands
- [ ] [Commands](robot/commands) - All commands
- [ ] [Libraries](lib) - Readme & All imported classes/vendor dependencies
- [ ] [Subsystems](robot/subsystems) - All subsystems
- [ ] [Development Guide](dev) - The guide to developing everything listed above *over time*