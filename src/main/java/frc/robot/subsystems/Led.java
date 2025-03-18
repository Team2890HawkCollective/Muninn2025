package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import org.dyn4j.dynamics.joint.RevoluteJoint;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;

public class Led {
    static final AddressableLED signalLights = new AddressableLED(Constants.LED.SIGNAL_LIGHTS_PORT);
    static final AddressableLEDBuffer signalLightsBuffer = new AddressableLEDBuffer(
            Constants.LED.SIGNAL_LIGHTS_LENGTH);
    
    public static void initLED(){
        signalLights.setLength(signalLightsBuffer.getLength());
        signalLights.setData(signalLightsBuffer);
        signalLights.start();
    }

    public static int getBufferLength(AddressableLEDBuffer buffer) {
        return buffer.getLength();
    }

    public Command setColorCommand(Color color) {
        return Commands.runOnce(() -> setColor(color));
    }

    /**
     * Set Color (With Blink)
     * @param color
     */
    public static void setColorBlink(Color color) {
        LEDPattern blinker = LEDPattern.solid(color);
        blinker.blink(Seconds.of(1.5), Seconds.of(1.5));
        blinker.applyTo(signalLightsBuffer);
        signalLights.setData(signalLightsBuffer);
    }

    /**
     *  Set Color (No Blink)
     * @param color
     */
    public static void setColor(Color color) {
        LEDPattern blinker = LEDPattern.solid(color);
        //blinker.blink(Seconds.of(1.5), Seconds.of(1.5));
        blinker.applyTo(signalLightsBuffer);
        signalLights.setData(signalLightsBuffer);
    }

    /**
     * Set Aligmnent Light Chunk (Solid)
     * @param color
     */
    //public static void setColorAlignment(Color color){
    //    LEDPattern blinker = LEDPattern.solid(color);
    //    //blinker.blink(Seconds.of(1.5), Seconds.of(1.5));
    //    blinker.applyTo(alignmentLEDS);
    //    signalLights.setData(signalLightsBuffer);
    //}

    /**
     * Set Aligmnent Light Chunk (Blink)
     * @param color
     */
    //public static void setColorAlignmentBlink(Color color){
    //    LEDPattern blinker = LEDPattern.solid(color);
    //    blinker.blink(Seconds.of(1.5), Seconds.of(1.5));
    //    blinker.applyTo(alignmentLEDS);
    //    signalLights.setData(signalLightsBuffer);
    //}
}
