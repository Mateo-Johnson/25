package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.ElevatorConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

public class Elevator extends SubsystemBase {
    // Hardware
    private final SparkMax leaderMotor;
    private final SparkMax followMotor;
    private final RelativeEncoder encoder;
    private final DigitalInput limitSwitch;

    // Control
    private final ProfiledPIDController profiledPIDController;
    @SuppressWarnings("unused")
    private final PIDController maintainPositionPID;
    private boolean isManualControl = false;
    private double currentSetpoint = 0.0;

    // Constants
    private static final double gearRatio = ElevatorConstants.gearRatio;
    private static final double SPD = ElevatorConstants.SPD; // 22T #25 sprocket in inches
    private static final double IPR = Math.PI * SPD;
    private static final double PCF = IPR / gearRatio;
    
    // Setpoints in inches
    private static final double L1 = ElevatorConstants.L1;
    private static final double intake = ElevatorConstants.intake;
    private static final double L2 = ElevatorConstants.L2;
    private static final double L3 = ElevatorConstants.L3;
    private static final double L4 = ElevatorConstants.L4;

    
    // Motion Profile Constraints
    private static final double maxV = ElevatorConstants.maxV; // inches per second
    private static final double maxA = ElevatorConstants.maxA; // inches per second squared
    
    // PID Constants
    private static final double prof_kP = ElevatorConstants.prof_kP;
    private static final double prof_kI = ElevatorConstants.prof_kI;
    private static final double prof_kD = ElevatorConstants.prof_kD;
    
    private static final double stay_kP = ElevatorConstants.stay_kP;
    private static final double stay_kI = ElevatorConstants.stay_kI;
    private static final double stay_kD = ElevatorConstants.stay_kD;
    
    // Manual Control
    private static final double max = ElevatorConstants.max; // max height | inches - slightly above highest setpoint

    public Elevator() {
        // Initialize hardware
        leaderMotor = new SparkMax(ElevatorConstants.leadID, MotorType.kBrushless);
        followMotor = new SparkMax(ElevatorConstants.followID, MotorType.kBrushless);
        encoder = leaderMotor.getEncoder();
        limitSwitch = new DigitalInput(ElevatorConstants.switchPort); // Update port as needed

        // Configure second motor as follower
        SparkMaxConfig followConfig = new SparkMaxConfig();
        followConfig.follow(leaderMotor, false);
        followMotor.configure(followConfig, null, null);

        SparkMaxConfig encoderConfig = new SparkMaxConfig();
        encoderConfig.encoder.positionConversionFactor(PCF);
        encoderConfig.encoder.velocityConversionFactor(PCF / 60.0);

        
        // Configure motion profiling
        TrapezoidProfile.Constraints constraints = 
            new TrapezoidProfile.Constraints(maxV, maxA);
            
        profiledPIDController = new ProfiledPIDController(
            prof_kP, prof_kI, prof_kD, constraints);
            
        maintainPositionPID = new PIDController(
            stay_kP, stay_kI, stay_kD);
            
        // Set initial position
        resetEncoders();
    }
    
    @Override
    public void periodic() {
        // Check limit switch
        if (isAtLowerLimit()) {
            resetEncoders();
        }
        
        // Only apply PID control if not in manual mode
        if (!isManualControl) {
            double output = profiledPIDController.calculate(
                getPosition(), currentSetpoint);
            setMotor(output);
        }
    }
    
    // D-Pad Control Methods
    public void L1() {
        setTargetPosition(L1);
    }
    
    public void intake() {
        setTargetPosition(intake);
    }
    
    public void L2() {
        setTargetPosition(L2);
    }
    
    public void L3() {
        setTargetPosition(L3);
    }

    public void L4() {
        setTargetPosition(L4);
    }
    
    // Bumper Control Methods
    public void manualUp(double speed) {
        isManualControl = true;
        if (getPosition() < max) {
            setMotor(speed);
        } else {
            setMotor(0);
        }
    }
    
    public void manualDown(double speed) {
        isManualControl = true;
        if (!isAtLowerLimit()) {
            setMotor(-speed);
        } else {
            setMotor(0);
        }
    }
    
    public void stopManual() {
        isManualControl = false;
        currentSetpoint = getPosition();
        profiledPIDController.reset(currentSetpoint);
    }
    
    // Helper Methods
    public void setTargetPosition(double position) {
        isManualControl = false;
        currentSetpoint = position;
    }
    
    public void setMotor(double output) {
        // Prevent movement past limits
        if (isAtLowerLimit() && output < 0) {
            leaderMotor.set(0);
        } else if (getPosition() >= max && output > 0) {
            leaderMotor.set(0);
        } else {
            leaderMotor.set(output);
        }
    }
    
    public boolean isAtLowerLimit() {
        return !limitSwitch.get(); // Return true if switch is closed (reads 0)
    }
    
    public double getPosition() {
        return encoder.getPosition();
    }
    
    public void resetEncoders() {
        encoder.setPosition(0.1);
        profiledPIDController.reset(0.1);
        currentSetpoint = 0.1;
    }
}