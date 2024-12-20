# SwerveBase-CTRE
[![CI](https://github.com/Team334/SwerveBase-CTRE/actions/workflows/main.yml/badge.svg)](https://github.com/Team334/SwerveBase-CTRE/actions/workflows/main.yml)

A base project for future robots that has CTRE generated swerve drive code and PhotonVision AprilTag processing.

# Features
- Swerve drive code using CTRE's swerve code generator.
- Device logging with SignalLogger and data logging with Epilogue and DogLog.
- Device Fault Logging as telemetry for at-home testing. The faults are also logged with DogLog for post-match review.
- Pre-match self check with custom self-check commands.

# Todo
- Add photon vision code (LOOK AT THE 2025 BETA UPDATES FIRST).
- Add more unit tests.
- Add proper licenses.
- 6328 wheel radius characterization.

## 2025 Latest Beta Known Issues
- To run sim, do `./gradlew simulateJava` instead of using the WPILib extension (for Epilogue to work).
- Epilogue can't handle annotated inner classes (see [this](https://github.com/wpilibsuite/2025Beta/issues/54) issue) which has been fixed in [this](https://github.com/wpilibsuite/allwpilib/pull/7439) pull request for the next wpilib release.
