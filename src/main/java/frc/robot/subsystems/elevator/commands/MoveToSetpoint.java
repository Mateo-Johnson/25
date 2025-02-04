package frc.robot.subsystems.elevator.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class MoveToSetpoint extends Command {
    private final Elevator elevator;
    private final String setpoint;
    
    public MoveToSetpoint(Elevator elevator, String setpoint) {
        this.elevator = elevator;
        this.setpoint = setpoint;
        addRequirements(elevator);
    }
    
    @Override
    public void initialize() {
        if (setpoint == "L1") {
            elevator.L1();
        } else if (setpoint == "HP") {
            elevator.intake();
        } else if (setpoint == "L2") {
            elevator.L2();
        } else if (setpoint == "L3") {
            elevator.L3();
        } else if (setpoint == "L4") {
            elevator.L4();
        }

    }
    
    @Override
    public boolean isFinished() {
        // Command never finishes on its own - requires interruption
        // This allows the PID controller to maintain position
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        // No specific end behavior needed as the subsystem will maintain position
    }
}