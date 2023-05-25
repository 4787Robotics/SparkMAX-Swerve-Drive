package frc.robot.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.TestSwerveModule;
import static frc.robot.Constants.*;

/** A command that will turn the robot to the specified angle. */
public class TurnToAngle extends PIDCommand {
  /**
   * Turns to robot to the specified angle.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive The drive subsystem to use
   */
  public TurnToAngle(double targetAngleDegrees, TestSwerveModule swerveTurnMotor) {
    super(
        new PIDController(TURN_P, TURN_I, TURN_D),
        // Close loop on heading
        swerveTurnMotor::getTurnEncoder,
        // Set reference to target
        targetAngleDegrees,
        // Pipe output to turn robot
        output -> swerveTurnMotor.setTurnMotor(output),
        // Require the drive
        swerveTurnMotor);

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(K_TURN_TOLERANCE, K_TURN_TOLERANCE_PER_SECOND);
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atSetpoint();
  }
}