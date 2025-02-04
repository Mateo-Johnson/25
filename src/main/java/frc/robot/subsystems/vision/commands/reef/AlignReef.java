package frc.robot.subsystems.vision.commands.reef;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.ComboDetector;
import frc.robot.utils.LimelightLib;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.commands.MoveToSetpoint;

public class AlignReef extends Command {
  // Subsystems and utilities
  private final ComboDetector comboDetector;
  private final Drivetrain driveSubsystem;
  private final Elevator elevator;

  // Alignment variables
  private String elevatorSetpoint;
  private boolean alignToLeft = true;
  
  // Limelight and alignment tracking
  private double tX; // Horizontal offset
  private double tY; // Vertical offset
  private double tID; // Fiducial ID
  
  // PID Controllers for precise alignment
  private final PIDController horizontalPID;
  private final PIDController verticalPID;
  
  // Alignment constants
  private static final double HORIZONTAL_TOLERANCE = 1.0; // Degrees
  private static final double VERTICAL_TOLERANCE = 1.0; // Degrees
  private static final double MAX_DRIVE_SPEED = 0.5; // Adjust based on your robot

  public AlignReef(
    ComboDetector comboDetector, 
    Drivetrain driveSubsystem, 
    Elevator elevator
  ) {
    this.comboDetector = comboDetector;
    this.driveSubsystem = driveSubsystem;
    this.elevator = elevator;
    
    // Configure PID Controllers
    this.horizontalPID = new PIDController(0.1, 0.0, 0.0); // Tune these values
    this.horizontalPID.setTolerance(HORIZONTAL_TOLERANCE);
    
    this.verticalPID = new PIDController(0.1, 0.0, 0.0); // Tune these values
    this.verticalPID.setTolerance(VERTICAL_TOLERANCE);
    
    // Require subsystems
    addRequirements(driveSubsystem, elevator);
  }

  @Override
  public void initialize() {
    // Determine elevator level based on D-Pad angle
    int dpadAngle = comboDetector.getDPadAngle();
    
    switch (dpadAngle) {
      case 0:   // D-Pad Up
        elevatorSetpoint = "L4";
        break;
      case 90:  // D-Pad Right
        elevatorSetpoint = "L3";
        break;
      case 180: // D-Pad Down
        elevatorSetpoint = "L2";
        break;
      case 270: // D-Pad Left
        elevatorSetpoint = "HP";
        break;
      default:
        elevatorSetpoint = "L1"; // Default or error state
    }

    // Move elevator to setpoint using MoveToSetpoint command
    new MoveToSetpoint(elevator, elevatorSetpoint).initialize();

    // Determine alignment direction based on combo
    int comboResult = comboDetector.detectCombo();
    alignToLeft = comboResult < 0;

    // Reset PID controllers
    horizontalPID.reset();
    verticalPID.reset();
  }

  @Override
  public void execute() {
    // Get Limelight data
    tX = LimelightLib.getTX("");
    tY = LimelightLib.getTY("");    
    tID = LimelightLib.getFiducialID("");

    // Ensure we have a valid target
    if (tID != 0) {
      // Calculate drive adjustments using PID
      double horizontalAdjustment = horizontalPID.calculate(tX, 0);
      double verticalAdjustment = verticalPID.calculate(tY, 0);

      // Determine side-to-side adjustment based on alignment preference
      double sideAdjustment = alignToLeft ? -0.1 : 0.1; // Adjust magnitude as needed

      // Drive adjustments
      driveSubsystem.drive(
        Math.min(horizontalAdjustment, MAX_DRIVE_SPEED),
        sideAdjustment,
        0, //Rotation
        true 
      );
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Stop drive and reset
    driveSubsystem.drive(0,0, 0, true);
  }

  @Override
  public boolean isFinished() {
    // Finish when:
    // 1. Horizontal and vertical alignments are within tolerance
    // 2. Optionally, add a timeout or additional conditions
    return horizontalPID.atSetpoint() && 
           verticalPID.atSetpoint();
  }
}