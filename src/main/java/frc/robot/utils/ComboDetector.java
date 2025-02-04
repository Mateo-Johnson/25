package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ComboDetector {
    private final CommandXboxController controller;
    private final Timer comboTimer = new Timer();
    private boolean isDpadPressed = false;
    private final double COMBO_TIMEOUT = 0.5; // Time window in seconds
    private final double STICK_THRESHOLD = 0.5; // Minimum stick movement to count as a flick
    
    public ComboDetector(int controllerPort) {
        controller = new CommandXboxController(controllerPort);
    }
    
    /**
     * Detects if a D-Pad button is pressed followed by a right stick flick
     * within the combo timeout window.
     * @return -1 for left flick combo, 1 for right flick combo, 0 for no combo
     */
    public int detectCombo() {
        // Check if any D-Pad button is newly pressed
        boolean dpadPressed = controller.getHID().getPOV() != -1;
        if (dpadPressed && !isDpadPressed) {
            isDpadPressed = true;
            comboTimer.reset();
            comboTimer.start();
        }
        
        // Reset if D-Pad is released
        if (!dpadPressed) {
            isDpadPressed = false;
            comboTimer.stop();
            return 0;
        }
        
        // Check for stick flick within time window
        if (isDpadPressed && comboTimer.get() <= COMBO_TIMEOUT) {
            double stickX = controller.getRightX();
            
            // Check for left flick
            if (stickX < -STICK_THRESHOLD) {
                isDpadPressed = false;
                comboTimer.stop();
                return -1;
            }
            // Check for right flick
            else if (stickX > STICK_THRESHOLD) {
                isDpadPressed = false;
                comboTimer.stop();
                return 1;
            }
        }
        
        // Reset if combo timeout expires
        if (comboTimer.get() > COMBO_TIMEOUT) {
            isDpadPressed = false;
            comboTimer.stop();
        }
        
        return 0;
    }
    
    // Alternative method using CommandXboxController's trigger bindings
    public void bindComboTriggers(Runnable onLeftFlick, Runnable onRightFlick) {
        // Create a trigger that activates when the combo is detected
        controller.povCenter()
            .whileTrue(new edu.wpi.first.wpilibj2.command.RunCommand(() -> {
                int result = detectCombo();
                if (result < 0) {
                    onLeftFlick.run();
                } else if (result > 0) {
                    onRightFlick.run();
                }
            }));
    }
}