package frc.robot.utils;

import edu.wpi.first.math.controller.PIDController;

public class HolonomicController {
    private final PIDController _xController = new PIDController(0, 0, 0);
    private final PIDController _yController = new PIDController(0, 0, 0);
    private final PIDController _headingController = new PIDController(0, 0, 0);

    
}
