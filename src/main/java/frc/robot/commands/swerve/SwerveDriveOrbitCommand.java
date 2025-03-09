package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveDrive;
import org.dyn4j.UnitConversion;
import org.dyn4j.geometry.Vector2;

import java.util.Vector;
import java.util.function.DoubleSupplier;

public class SwerveDriveOrbitCommand extends Command {
  private SwerveDrive drive;

  private double kP = 20;
  private double kI = 3;
  private double kD = 1;

  private DoubleSupplier forward;
  private DoubleSupplier side;
  //  TODO: Add way to make PID go in optimal direction
  private PIDController pidController = new PIDController(kP, kI, kD);

  private Translation2d targetPosition;

  public double closetAngle(double current, double target) {
    double direction = Math.signum(target - current);
    double changes = Math.abs(target - current);

    if (Math.abs(current - target) > 180) {
      changes = Math.abs(360 - changes);
      direction *= -1;
    }
    return changes * direction;
  }

  private double modifyInputs(double val) {
    if (Math.abs(val) < Constants.DriveConstants.kTanDeadband) {
      val = 0;
    }
    return val * 6;
  }

  public SwerveDriveOrbitCommand(DoubleSupplier ly, DoubleSupplier lx, SwerveDrive drive,
      Translation2d targetPosition) {
    this.forward = ly;
    this.side = lx;

    this.drive = drive;

    this.targetPosition = targetPosition;
    pidController.setSetpoint(0);

    addRequirements(drive);
  }


  @Override
  public void execute() {
    this.drive.drive(new Translation2d(modifyInputs(-this.forward.getAsDouble()),
        modifyInputs(-this.side.getAsDouble())), pidController.calculate(
        -this.closetAngle(this.drive.getHeading().getDegrees(),
            this.getTargetHeading() / Math.PI * 180) / 180 * Math.PI), true, false);
  }

  @Override
  public void end(boolean interrupted) {
    this.drive.drive(new Translation2d(0, 0), 0, false, false);
  }

  private double getTargetHeading() {
    Translation2d robotPosition = this.drive.getPose().getTranslation();

    return MathUtil.angleModulus(Math.atan2(this.targetPosition.getY() - robotPosition.getY(),
        this.targetPosition.getX() - robotPosition.getX()));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
