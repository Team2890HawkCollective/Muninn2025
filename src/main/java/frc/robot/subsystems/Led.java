package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import org.dyn4j.dynamics.joint.RevoluteJoint;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;

public class Led {
    static final AddressableLED signalLights = new AddressableLED(Constants.LED.SIGNAL_LIGHTS_PORT);
    static final AddressableLEDBuffer signalLightsBuffer = new AddressableLEDBuffer(
            Constants.LED.SIGNAL_LIGHTS_LENGTH);
    static final AddressableLEDBufferView alignmentLEDS = signalLightsBuffer.createView(200, 225);
    
    public static void initLED(){
        signalLights.setLength(signalLightsBuffer.getLength());
        signalLights.setData(signalLightsBuffer);
        signalLights.start();
    }

    public static int getBufferLength(AddressableLEDBuffer buffer) {
        return buffer.getLength();
    }

    public static void setColorRainbow() {
        LEDPattern rainbow = LEDPattern.rainbow(255, 125);        
        Distance ledSpacing = Meters.of(1 / 120.0);
        LEDPattern scrollingRainbow = rainbow.scrollAtAbsoluteSpeed(InchesPerSecond.of(1), ledSpacing);
        scrollingRainbow.applyTo(signalLightsBuffer);
        signalLights.setData(signalLightsBuffer);
    }

    public static void setColorBreathe(Color firstColor, Color secondColor)
    {
        LEDPattern gradiant = LEDPattern.gradient(GradientType.kContinuous, firstColor, secondColor);
        LEDPattern breathe = gradiant.breathe(Seconds.of(2));
        breathe.applyTo(signalLightsBuffer);
    }
    /**
     * Set Color (With Blink)
     * @param color
     */
    public static void setColorBlink(Color color) {
        LEDPattern blinker = LEDPattern.solid(color);
        blinker.applyTo(signalLightsBuffer);
        for(int i = 0; i < 10; i++)
        {
            signalLights.stop();
            signalLights.start();
        }
    }

    /**
     *  Set Color (No Blink)
     * @param color
     */
    public static void setColor(Color color) {
        LEDPattern solid = LEDPattern.solid(color);
        solid.applyTo(signalLightsBuffer);
    }

    /**
     * Set Aligmnent Light Chunk (Solid)
     * @param color
     */
    public static void setColorAlignment(Color color){
        LEDPattern blinker = LEDPattern.solid(color);
        //blinker.blink(Seconds.of(1.5), Seconds.of(1.5));
        blinker.applyTo(alignmentLEDS);
        signalLights.setData(signalLightsBuffer);
    }

    /**
     * Set Aligmnent Light Chunk (Blink)
     * @param color
     */
    public static void setColorAlignmentBlink(Color color){
        LEDPattern blinker = LEDPattern.solid(color);
        blinker.blink(Seconds.of(1.5), Seconds.of(1.5));
        blinker.applyTo(alignmentLEDS);
        signalLights.setData(signalLightsBuffer);
    }

    /**
     * Turns Off Alignment LEDs
     * @param color
     */
    public static void turnOffAlignmentLights(){
        LEDPattern blinker = LEDPattern.kOff;
        //blinker.blink(Seconds.of(1.5), Seconds.of(1.5));
        blinker.applyTo(alignmentLEDS);
        signalLights.setData(signalLightsBuffer);
    }

    public static double getMatchTime() {
        return DriverStation.getMatchTime();
    }

    public static void updatePeriodically() {
        // Periodically send the latest LED color data to the LED strip for it to display
        signalLights.setData(signalLightsBuffer);
    }
}
