# Vendor Dependencies (install before deployment)

The following dependencies are imported into the project:
- Phoenix v6 - CTRE Phoenix API for CAN Spark motor controllers
- RevLib 2024 - REV Robotics API for brushless motor controllers
- Pathplanner 2024 - Path generation for autonomous gamemode
- Choreo - Efficient motion profiling tool for path generation
- WPILib New Commands 1.0 - WPILib's command-based project structure

## JSON Links
- Phoenix v6: https://pro.docs.ctr-electronics.com/en/stable/docs/installation/installation-frc.html
- RevLib 2024: https://software-metadata.revrobotics.com/REVLib-2024.json
- Pathplanner 2024: https://3015rangerrobotics.github.io/pathplannerlib/PathplannerLib.json
- Choreo: https://sleipnirgroup.github.io/ChoreoLib/dep/ChoreoLib.json
- WPILib New Commands 1.0: Pre-installed with WPILib

## To install (or check installation):

1. Open WPILib VSCode Client and open the cloned folder as a project.
2. Press `ctrl`+`shift`+`P` to open up a VSCode menu, then type in 'Manage Vendor Libraries'. Press `enter`.
3. Indicate the button 'Install new library (online)', and use each link to install/check installation.

To update, follow above steps until Step 3, and then indicate the button 'Check for updates (online)'.

If you use another code editor than VSCode, follow instructions by the WPILib extension on that code editor via Gradle.

## Third-Party Library Documentation

- [Phoenix v6 API Reference](https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/index.html)
  - [Phoenix v6 Java Docs](https://api.ctr-electronics.com/phoenix6/release/java/)
- [RevLib 2024 Documentation](https://docs.revrobotics.com/brushless/revlib/revlib-overview)
  - [RevLib 2024 Java Docs](https://codedocs.revrobotics.com/java/com/revrobotics/package-summary.html)
- [Pathplanner 2024 Documentation](https://pathplanner.dev/home.html)
- [Choreo Documentation](https://sleipnirgroup.github.io/Choreo/)
- [WPILib Command Structure Docs](https://docs.wpilib.org/en/stable/docs/software/commandbased/commands.html)