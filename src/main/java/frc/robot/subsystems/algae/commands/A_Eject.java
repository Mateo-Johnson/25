package frc.robot.subsystems.algae.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algae.Algae;

public class A_Eject extends Command {
    private final Algae intake;
    
    public A_Eject(Algae intake) {
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