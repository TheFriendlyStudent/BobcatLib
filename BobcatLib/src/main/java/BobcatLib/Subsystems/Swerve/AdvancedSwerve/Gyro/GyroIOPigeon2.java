package BobcatLib.Subsystems.Swerve.AdvancedSwerve.Gyro;


import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import BobcatLib.Subsystems.Swerve.AdvancedSwerve.Constants.SwerveConstants;
import edu.wpi.first.math.geometry.Rotation2d;


public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon;

    public GyroIOPigeon2(SwerveConstants constants) {
        pigeon = new Pigeon2(constants.pigeonID, constants.canbus);
        Pigeon2Configuration config = new Pigeon2Configuration();
        pigeon.getConfigurator().apply(config);

        pigeon.reset();
        pigeon.setYaw(0);
    }

    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = pigeon.isConnected();

        inputs.yawPosition = Rotation2d.fromDegrees(pigeon.getYaw().getValueAsDouble());
        inputs.pitchPosition = Rotation2d.fromDegrees(pigeon.getPitch().getValueAsDouble());
        inputs.rollPosition = Rotation2d.fromDegrees(pigeon.getRoll().getValueAsDouble());
    }

    /**
     * Sets the current Gyro yaw
     * @param yaw value to set yaw to, in degrees
     */
    public void setYaw(double yaw) {
        pigeon.setYaw(yaw);
    }
}