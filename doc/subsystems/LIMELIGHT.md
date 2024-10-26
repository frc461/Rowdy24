# Limelight Documentation

Software implementation of the Limelight subsystem on the Rowdy 24 robot. The function of the Limelight is to detect the AprilTags on the field and figure out how far away the robot is from them. We use a [Limelight 3G](https://limelightvision.io/collections/products/products/limelight-3g) on the robot. It is used as a static class in our code. 

## Implementation

### Methods

There are many methods to get the robot's position on the field using the data published by the limelight into network tables. We use the [LimelightHelpers.java](../../src/main/java/frc/lib/util/LimelightHelpers.java) to facilitate the extraction of data. 