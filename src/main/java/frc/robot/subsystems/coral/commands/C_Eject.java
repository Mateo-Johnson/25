package frc.robot.subsystems.coral.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.Coral;

public class C_Eject extends Command {
    private final Coral intake;
    
    public C_Eject(Coral intake) {
        this.intake = intake;
        addRequirements(intake);
    }
    
    @Override
    public void initialize() {
        intake.eject();
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop completely when done ejecting
        intake.stop();
    }
    
    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted
    }
}