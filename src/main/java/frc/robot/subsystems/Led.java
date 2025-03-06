package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import org.dyn4j.dynamics.joint.RevoluteJoint;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Constants;

public class Led {
    static final AddressableLED signalLights = new AddressableLED(Constants.LED.SIGNAL_LIGHTS_PORT);
    static final AddressableLEDBuffer signalLightsBuffer = new AddressableLEDBuffer(
            Constants.LED.SIGNAL_LIGHTS_LENGTH);

    public static int getBufferLength(AddressableLEDBuffer buffer) {
        return buffer.getLength();
    }

    public Command setColorCommand(int r, int g, int b) {
        return Commands.runOnce(() -> setColor(r, g, b));
    }

    public static void setColor(int r, int g, int b) {
        for (var i = 0; i < Led.getBufferLength(signalLightsBuffer); i++)
            signalLightsBuffer.setRGB(i, r, g, b);
        signalLights.setData(signalLightsBuffer);
    }
}
