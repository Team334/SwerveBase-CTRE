# Graph Report - SwerveBase-CTRE  (2026-11-11)

## Corpus Check
- Corpus is ~17,270 words - fits in a single context window. You may not need a graph.

## Summary
- 450 nodes · 1051 edges · 16 communities (7 shown, 9 thin omitted)
- Extraction: 89% EXTRACTED · 11% INFERRED · 0% AMBIGUOUS · INFERRED: 115 edges (avg confidence: 0.85)
- Token cost: 93,865 input · 0 output

## Community Hubs (Navigation)
- Fault Logger & NT Tables
- Robot Lifecycle & Autos
- Vision Pose Estimation
- Holonomic Control & Trajectories
- Swerve Commands & Self-Check
- Swerve Modules & Field
- Tuner Constants Config
- Input Streams & SysId
- CTRE Device Faults
- Docs & CI Pipeline
- Unit Test Harness
- Main Entry
- Superstructure

## God Nodes (most connected - your core abstractions)
1. `Swerve` - 62 edges
2. `Fault` - 26 edges
3. `TunerConstants` - 24 edges
4. `HolonomicController` - 23 edges
5. `VisionPoseEstimator` - 21 edges
6. `InputStream` - 20 edges
7. `FaultLogger` - 20 edges
8. `FaultType` - 20 edges
9. `SwerveConstants` - 19 edges
10. `VisionPoseEstimatorTest` - 19 edges

## Surprising Connections (you probably didn't know these)
- `Device Fault Logging` --references--> `FaultLogger`  [INFERRED]
  README.md → src/main/java/frc/lib/fault/FaultLogger.java
- `Pre-match Self-Check` --references--> `SelfChecked`  [INFERRED]
  README.md → src/main/java/frc/lib/subsystem/SelfChecked.java
- `VisionPoseEstimator (README feature)` --references--> `VisionPoseEstimator`  [INFERRED]
  README.md → src/main/java/frc/robot/utils/VisionPoseEstimator.java
- `wpilib/roborio-cross-ubuntu:2025-24.04 Container` --conceptually_related_to--> `RoboRIO Deploy Directory`  [INFERRED]
  .github/workflows/main.yml → src/main/deploy/example.txt
- `SwerveBase-CTRE Project` --references--> `CI Workflow (main.yml)`  [EXTRACTED]
  README.md → .github/workflows/main.yml

## Import Cycles
- None detected.

## Hyperedges (group relationships)
- **Logging and Telemetry Stack** — readme_signallogger, readme_epilogue, readme_doglog, readme_device_fault_logging [INFERRED 0.85]
- **Vision Pose Estimation Pipeline** — readme_photon_vision, readme_apriltag_processing, readme_visionposeestimator, readme_ctre_swerve_generator [INFERRED 0.85]
- **CI Pipeline** — _github_workflows_main_ci_workflow, _github_workflows_main_spotless_job, _github_workflows_main_build_job, _github_workflows_main_wpilib_roborio_cross_container [EXTRACTED 1.00]

## Communities (16 total, 9 thin omitted)

### Community 0 - "Fault Logger & NT Tables"
Cohesion: 0.07
Nodes (10): FaultLogger, FaultReporter, Fault, FaultsTable, FaultType, ERROR, INFO, WARNING (+2 more)

### Community 1 - "Robot Lifecycle & Autos"
Cohesion: 0.05
Nodes (4): GcStatsCollector, Autos, Robot, RobotTest

### Community 2 - "Vision Pose Estimation"
Cohesion: 0.08
Nodes (4): VisionPoseEstimate, VisionPoseEstimator, VisionPoseEstimatorConstants, VisionPoseEstimatorTest

### Community 4 - "Swerve Commands & Self-Check"
Cohesion: 0.09
Nodes (3): SelfChecked, Swerve, WheelRadiusCharacterizationState

### Community 5 - "Swerve Modules & Field"
Cohesion: 0.08
Nodes (6): Constants, FieldConstants, MotorConstants, Ports, SwerveConstants, VisionConstants

### Community 7 - "Input Streams & SysId"
Cohesion: 0.08
Nodes (3): InputStream, TimeoutLinearFilter, SysId

### Community 9 - "Docs & CI Pipeline"
Cohesion: 0.10
Nodes (18): build Job (gradlew build), CI Workflow (main.yml), spotless Job (gradlew spotlessCheck), wpilib/roborio-cross-ubuntu:2025-24.04 Container, AprilTag Vision Processing, Choreo, CTRE Swerve Generator, DogLog (data logging) (+10 more)

## Knowledge Gaps
- **12 isolated node(s):** `INFO`, `WARNING`, `ERROR`, `spotless Job (gradlew spotlessCheck)`, `CTRE Swerve Generator` (+7 more)
  These have ≤1 connection - possible missing edges or undocumented components. (Counts symbols only; 118 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **9 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `Swerve` connect `Swerve Commands & Self-Check` to `Fault Logger & NT Tables`, `Robot Lifecycle & Autos`, `Vision Pose Estimation`, `Holonomic Control & Trajectories`, `Swerve Modules & Field`, `Tuner Constants Config`, `Input Streams & SysId`?**
  _High betweenness centrality (0.195) - this node is a cross-community bridge._
- **Are the 2 inferred relationships involving `Fault` (e.g. with `.faultEquality()` and `.attempt()`) actually correct?**
  _`Fault` has 2 INFERRED edges - model-reasoned connections that need verification._
- **What connects `INFO`, `WARNING`, `ERROR` to the rest of the system?**
  _12 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `Fault Logger & NT Tables` be split into smaller, more focused modules?**
  _Cohesion score 0.07017543859649122 - nodes in this community are weakly interconnected._
- **Why does `VisionPoseEstimator` connect `Vision Pose Estimation` to `CTRE Device Faults`, `Docs & CI Pipeline`, `Swerve Commands & Self-Check`, `Swerve Modules & Field`?**
  _High betweenness centrality (0.088) - this node is a cross-community bridge._
- **Should `Robot Lifecycle & Autos` be split into smaller, more focused modules?**
  _Cohesion score 0.05075187969924812 - nodes in this community are weakly interconnected._
- **Why does `FaultLogger` connect `Fault Logger & NT Tables` to `Robot Lifecycle & Autos`, `Vision Pose Estimation`, `Swerve Modules & Field`, `CTRE Device Faults`, `Docs & CI Pipeline`, `Unit Test Harness`?**
  _High betweenness centrality (0.080) - this node is a cross-community bridge._
