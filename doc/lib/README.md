# Library Documentation

> [!NOTE]
> This part of the documentation does not include documentation of the vendor dependencies themselves. For reference to these vendor dependencies, click the links to their respective documentations in the [Vendor Dependencies specifications](VENDOR%20LIBRARIES.md#third-party-library-documentation).


This section of the documentation is about the modules that are added to the robot code to simplify and organize the codebase. The modules are created by third-party developers and are very useful for robot functionality through supplementary classes. The two types of modules that are used by the robot software are:

- Miscellaneous Utility Classes - smaller in volume, directly imported into the codebase as a Java class (in the `lib` directory)
- Vendor Dependencies - larger in volume, imported as a JSON file (in the `vendordeps` directory) and installed through Gradle