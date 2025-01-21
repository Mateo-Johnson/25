package frc.robot.subsystems.algae.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algae.Algae;
import frc.robot.subsystems.lights.Lights;

public class A_Hold extends Command {
    private final Algae intake;
    private final Lights lights;
    
    public A_Hold(Algae intake, Lights lights) {
        this.intake = intake;
        this.lights = lights;
        addRequirements(intake);
    }
    
    @Override
    public void initialize() {
        intake.hold();
        lights.setColor("SeaGreen");
    }
    
    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop if interrupted
        intake.stop();
    }
}