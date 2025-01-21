package frc.robot.subsystems.lights;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.LightConstants;
import edu.wpi.first.wpilibj.util.Color;

public class Lights extends SubsystemBase {
    private final AddressableLED ledStrip;
    private final AddressableLEDBuffer ledBuffer;
    private int animationCounter = 0;
    private double breathingValue = 0;
    private int pulseValue = 0;
    private int flashCounter = 0;
    private boolean flashState = false;
    private int snakeIndex = 0;
    private int wipeIndex = 0;
    private int rainbowCycleHue = 0;

    /**
     * Helper class to store RGB values
     */
    private static class RGB {
        public final int r, g, b;
        
        public RGB(int r, int g, int b) {
            this.r = r;
            this.g = g;
            this.b = b;
        }
    }

    /**
     * Converts a color name to its RGB values
     * @param colorName The name of the color to convert
     * @return RGB object containing the color values, or null if color name not found
     */
    private RGB getColorFromName(String colorName) {
        for (String[] color : LightsIndex.COLORS) {
            if (color[0].equalsIgnoreCase(colorName)) {
                String[] rgb = color[1].split(",");
                return new RGB(
                    Integer.parseInt(rgb[0].trim()),
                    Integer.parseInt(rgb[1].trim()),
                    Integer.parseInt(rgb[2].trim())
                );
            }
        }
        return null;
    }

    /**
     * Creates a new LED subsystem to control an addressable LED strip.
     * @param pwmPort The PWM port on the RoboRIO that the LED strip is connected to
     * @param length The number of LEDs in the strip
     */
    public Lights() {
        ledStrip = new AddressableLED(LightConstants.port);
        ledBuffer = new AddressableLEDBuffer(LightConstants.length);
        ledStrip.setLength(ledBuffer.getLength());
        ledStrip.start();
    }

    /**
     * Sets all LEDs in the strip to a single solid color using a color name.
     * @param colorName The name of the color from the COLORS array
     */
    public void setColor(String colorName) {
        RGB color = getColorFromName(colorName);
        if (color != null) {
            setColor(color.r, color.g, color.b);
        }
    }
    
    /**
     * Sets all LEDs in the strip to a single solid color using RGB values.
     * @param r Red value (0-255)
     * @param g Green value (0-255)
     * @param b Blue value (0-255)
     */
    public void setColor(int r, int g, int b) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, r, g, b);
        }
        ledStrip.setData(ledBuffer);
    }

    /**
     * Creates a breathing effect where the LEDs smoothly fade in and out.
     * @param colorName The name of the color from the COLORS array
     */
    public void breathing(String colorName) {
        RGB color = getColorFromName(colorName);
        if (color != null) {
            breathing(color.r, color.g, color.b);
        }
    }

    public void breathing(int r, int g, int b) {
        double intensity = (Math.sin(breathingValue) + 1.0) / 2.0;
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 
                (int)(r * intensity),
                (int)(g * intensity),
                (int)(b * intensity));
        }
        breathingValue += 0.05;
        ledStrip.setData(ledBuffer);
    }

    /**
     * Creates a strobe effect that rapidly alternates between the specified color and off.
     * @param colorName The name of the color from the COLORS array
     * @param rate The rate of strobing (lower number = faster strobe)
     */
    private boolean strobeOn = false;
    public void strobe(String colorName, int rate) {
        RGB color = getColorFromName(colorName);
        if (color != null) {
            strobe(color.r, color.g, color.b, rate);
        }
    }

    public void strobe(int r, int g, int b, int rate) {
        if (animationCounter % rate == 0) {
            strobeOn = !strobeOn;
        }
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            if (strobeOn) {
                ledBuffer.setRGB(i, r, g, b);
            } else {
                ledBuffer.setRGB(i, 0, 0, 0);
            }
        }
        animationCounter++;
        ledStrip.setData(ledBuffer);
    }

    /**
     * Creates a wave pattern that smoothly transitions the brightness of LEDs.
     * @param colorName The name of the color from the COLORS array
     */
    public void wave(String colorName) {
        RGB color = getColorFromName(colorName);
        if (color != null) {
            wave(color.r, color.g, color.b);
        }
    }

    public void wave(int r, int g, int b) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            double waveVal = Math.sin(animationCounter * 0.1 + i * 0.2);
            double intensity = (waveVal + 1.0) / 2.0;
            ledBuffer.setRGB(i,
                (int)(r * intensity),
                (int)(g * intensity),
                (int)(b * intensity));
        }
        animationCounter++;
        ledStrip.setData(ledBuffer);
    }

    /**
     * Creates a color wipe effect where the color gradually fills the strip.
     * @param colorName The name of the color from the COLORS array
     */
    public void colorWipe(String colorName) {
        RGB color = getColorFromName(colorName);
        if (color != null) {
            colorWipe(color.r, color.g, color.b);
        }
    }

    public void colorWipe(int r, int g, int b) {
        if (wipeIndex < ledBuffer.getLength()) {
            ledBuffer.setRGB(wipeIndex, r, g, b);
            wipeIndex++;
        } else {
            wipeIndex = 0;
        }
        ledStrip.setData(ledBuffer);
    }

    /**
     * Creates a theater chase effect, similar to marquee lights.
     * @param colorName The name of the color from the COLORS array
     */
    public void theaterChase(String colorName) {
        RGB color = getColorFromName(colorName);
        if (color != null) {
            theaterChase(color.r, color.g, color.b);
        }
    }

    public void theaterChase(int r, int g, int b) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            if ((i + animationCounter) % 3 == 0) {
                ledBuffer.setRGB(i, r, g, b);
            } else {
                ledBuffer.setRGB(i, 0, 0, 0);
            }
        }
        animationCounter++;
        ledStrip.setData(ledBuffer);
    }

    /**
     * Creates an alternating pattern between two colors.
     * @param color1Name The name of the first color from the COLORS array
     * @param color2Name The name of the second color from the COLORS array
     * @param segmentLength Number of LEDs in each color segment
     */
    public void alternate(String color1Name, String color2Name, int segmentLength) {
        RGB color1 = getColorFromName(color1Name);
        RGB color2 = getColorFromName(color2Name);
        if (color1 != null && color2 != null) {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                boolean isColor1 = ((i / segmentLength) % 2) == 0;
                RGB color = isColor1 ? color1 : color2;
                ledBuffer.setRGB(i, color.r, color.g, color.b);
            }
            ledStrip.setData(ledBuffer);
        }
    }

    // *** New Effects Below ***

    /**
     * Creates a pulsing effect where the LEDs gradually increase and decrease in brightness.
     * @param colorName The name of the color from the COLORS array
     */
    public void pulse(String colorName) {
        RGB color = getColorFromName(colorName);
        if (color != null) {
            pulse(color.r, color.g, color.b);
        }
    }

    public void pulse(int r, int g, int b) {
        double intensity = Math.abs(Math.sin(pulseValue)); // Sinusoidal pulse
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 
                (int)(r * intensity),
                (int)(g * intensity),
                (int)(b * intensity));
        }
        pulseValue += 0.1;
        ledStrip.setData(ledBuffer);
    }

    /**
     * Creates a flash effect where all LEDs flash on and off rapidly.
     * @param colorName The name of the color from the COLORS array
     * @param rate The rate at which the flash happens (lower value = faster flash)
     */
    public void flash(String colorName, int rate) {
        RGB color = getColorFromName(colorName);
        if (color != null) {
            flash(color.r, color.g, color.b, rate);
        }
    }

    public void flash(int r, int g, int b, int rate) {
        if (flashCounter % rate == 0) {
            flashState = !flashState;
        }
        if (flashState) {
            setColor(r, g, b);
        } else {
            setColor(0, 0, 0);
        }
        flashCounter++;
    }

    /**
     * Creates a snake effect where LEDs light up one by one and "snake" down the strip.
     * @param colorName The name of the color from the COLORS array
     */
    public void snake(String colorName) {
        RGB color = getColorFromName(colorName);
        if (color != null) {
            snake(color.r, color.g, color.b);
        }
    }

    public void snake(int r, int g, int b) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            if (i == snakeIndex) {
                ledBuffer.setRGB(i, r, g, b);
            } else {
                ledBuffer.setRGB(i, 0, 0, 0);
            }
        }
        snakeIndex++;
        if (snakeIndex >= ledBuffer.getLength()) {
            snakeIndex = 0;
        }
        ledStrip.setData(ledBuffer);
    }

    /**
     * Creates a rainbow effect that cycles through colors across the LED strip.
     */
    public void rainbowCycle() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            int hue = (rainbowCycleHue + (i * 180 / ledBuffer.getLength())) % 360;
            Color color = Color.fromHSV(hue, 255, 255);
            // IT MIGHT NOT WORK IF THESE ARE CAST TO INTS IDK
            ledBuffer.setRGB(i, (int) color.red, (int) color.green, (int) color.blue);
        }
        rainbowCycleHue += 3;
        if (rainbowCycleHue >= 360) {
            rainbowCycleHue = 0;
        }
        ledStrip.setData(ledBuffer);
    }

    public void turnOff() {
        setColor(0, 0, 0);  // Turns off the lights
    }
    
}
