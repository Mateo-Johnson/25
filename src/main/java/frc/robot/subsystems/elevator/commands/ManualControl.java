package frc.robot.subsystems.elevator.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class ManualControl extends Command {
    private final Elevator elevator;
    private final boolean isUpward;
    private final double speed;
    
    public ManualControl(Elevator elevator, boolean isUpward, double speed) {
        this.elevator = elevator;
        this.isUpward = isUpward;
        this.speed = speed;
        addRequirements(elevator);
    }
    
    @Override
    public void execute() {
        if (isUpward) {
            elevator.manualUp(speed);
        } else {
            elevator.manualDown(speed);
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        elevator.stopManual();
    }
    
    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted
    }
}