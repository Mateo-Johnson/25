package frc.robot.subsystems.elevator.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class HomeElevator extends Command {
    private final Elevator elevator;
    private static final double homeSpeed = -0.1; // Slow speed for finding limit switch
    
    public HomeElevator(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }
    
    @Override
    public void initialize() {
        // Start moving down slowly
        elevator.manualDown(homeSpeed);
    }
    
    @Override
    public boolean isFinished() {
        // Command finishes when limit switch is triggered
        return elevator.isAtLowerLimit();
    }
    
    @Override
    public void end(boolean interrupted) {
        elevator.stopManual();
        if (!interrupted) {
            // Only reset encoders if we actually hit the limit switch
            elevator.resetEncoders();
        }
    }
}