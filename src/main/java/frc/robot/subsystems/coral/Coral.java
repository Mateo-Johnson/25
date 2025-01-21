package frc.robot.subsystems.coral;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.CoralConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Coral extends SubsystemBase {
    // Hardware
    private final SparkMax motor;
    private final RelativeEncoder encoder;

    // Control
    private final PIDController velocityPID;
    private double targetVelocity = CoralConstants.targetVelocity;
    private IntakeMode currentMode = IntakeMode.STOPPED;

    // Constants
    private static final int MOTOR_ID = CoralConstants.leftCoralID; // Update this to match CAN ID
    private static final double GEAR_RATIO = CoralConstants.gearRatio; // Update based on gearing

    // Speed Constants (in RPM at the roller)
    private static final double intake_speed = CoralConstants.intake_speed;
    private static final double eject_speed = CoralConstants.eject_speed;
    private static final double hold_speed = CoralConstants.hold_speed;
    
    // PID Constants for velocity control
    private static final double kP = CoralConstants.kP;
    private static final double kI = CoralConstants.kI;
    private static final double kD = CoralConstants.kD;
    private static final double kF = CoralConstants.kF;

    // Current limiting
    private static final int SCL = CoralConstants.SCL; // amps
    private static final int FCL = CoralConstants.FCL; // amps
    
    public enum IntakeMode {
        INTAKING,
        EJECTING,
        HOLDING,
        STOPPED
    }

    public Coral() {
        // Initialize hardware
        motor = new SparkMax(MOTOR_ID, MotorType.kBrushless);
        encoder = motor.getEncoder();
        
        SparkMaxConfig intake = new SparkMaxConfig();
        intake.smartCurrentLimit(SCL, FCL);
        intake.inverted(false);
        intake.idleMode(IdleMode.kBrake);
        // Configure encoder conversion factors
        intake.encoder.velocityConversionFactor(1.0 / GEAR_RATIO); // Convert to roller RPM

        // Initialize PID Controller
        velocityPID = new PIDController(kP, kI, kD);
        
        motor.configure(intake, ResetMode.kResetSafeParameters, null);
    }
    
    @Override
    public void periodic() {
        // Update motor based on current mode
        double currentVelocity = encoder.getVelocity();
        double output = velocityPID.calculate(currentVelocity, targetVelocity) + 
                       (targetVelocity * kF); // Add feedforward
        
        motor.set(output);
        
        // Log data to SmartDashboard
        SmartDashboard.putString("Intake Mode", currentMode.toString());
        SmartDashboard.putNumber("Intake Velocity", currentVelocity);
        SmartDashboard.putNumber("Intake Target Velocity", targetVelocity);
        SmartDashboard.putNumber("Intake Current", motor.getOutputCurrent());
    }
    
    // Control Methods
    public void intake() {
        setMode(IntakeMode.INTAKING);
    }
    
    public void eject() {
        setMode(IntakeMode.EJECTING);
    }
    
    public void hold() {
        setMode(IntakeMode.HOLDING);
    }
    
    public void stop() {
        setMode(IntakeMode.STOPPED);
    }
    
    private void setMode(IntakeMode mode) {
        currentMode = mode;
        switch (mode) {
            case INTAKING:
                targetVelocity = intake_speed;
                break;
            case EJECTING:
                targetVelocity = eject_speed;
                break;
            case HOLDING:
                targetVelocity = hold_speed;
                break;
            case STOPPED:
            default:
                targetVelocity = 0.0;
                break;
        }
    }
    
    // Getter Methods
    public IntakeMode getCurrentMode() {
        return currentMode;
    }
    
    public double getCurrentVelocity() {
        return encoder.getVelocity();
    }
    
    public double getCurrentDraw() {
        return motor.getOutputCurrent();
    }
    
    public boolean isAtTargetVelocity() {
        return Math.abs(encoder.getVelocity() - targetVelocity) < 50.0; // Within 50 RPM
    }
}