# FRC 461 Rowdy24 Robot Code

## Notes

  1. Programming is focusing on organizing different groups to work on different subsystems/ideas via branches. Our latest code is always developed in seperate dev branches before being
  tested and merged back into master. Explore different specialized branches for your interest. If a certain subsystem is not being developed in a specific branch, it has either been worked on or not yet.

  2. If you are interested in deploying our Robot/Auto code, we advise you to use the master branch instead of others, as they are experimental and may (will) cause issues. Use other branches at your own risk.
  
  3. Our drivetrain is made up of 4 SDS-l3 swerve drives with NEO brushless motors for rotation and propulsion and CTRE CanCoders to measure wheel rotation.
  
  4. This project uses a command-based structure. You can read more about WPILib command-based programming in the [WPILib Docs](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html).

## Deploying Our Code

  Before anything, make sure:
  
  - [WPILib](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html) is installed
  - [Git for Windows](https://git-scm.com/download/win) or [Git (Linux/macOS)](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git) is installed
  - Network on device is connected to your robot

  Now we are going to retrieve the code from this repository to deploy it.
  
  1. Open your terminal and enter the command: `git clone https://github.com/frc461/Rowdy24.git`
  2. Once it finishes cloning, open WPILib VSCode Client and open the cloned folder as a project.
  3. Check our [vendor dependencies](https://github.com/frc461/Rowdy24/edit/master/README.md#vendor-dependencies-install-before-deploying-our-code) to confirm that they are installed.
  4. Press `ctrl`+`shift`+`P` to open up a VSCode menu, then type in 'Deploy'. Press `enter`.

## Vendor Dependencies (Install before deploying our code)

  - Phoenix v6: https://pro.docs.ctr-electronics.com/en/stable/docs/installation/installation-frc.html  
  - RevLib 2024: https://software-metadata.revrobotics.com/REVLib-2024.json
  - Pathplanner 2024: https://3015rangerrobotics.github.io/pathplannerlib/PathplannerLib.json
  - WPIlib New Commands 1.0: Pre-installed with WPIlib

  To install (or check installation):
  
  1. Open WPILib VSCode Client and open the cloned folder as a project.
  2. Press `ctrl`+`shift`+`P` to open up a VSCode menu, then type in 'Manage Vendor Libraries'. Press `enter`.
  3. Indicate the button 'Install new library (online)', and use each link to install/check installation.

  To update, follow above steps until Step 3, and then indicate the button 'Check for updates (online)'.

  If you do use another code editor than VSCode, follow instructions by the WPILib extension on that code editor via Gradle.
