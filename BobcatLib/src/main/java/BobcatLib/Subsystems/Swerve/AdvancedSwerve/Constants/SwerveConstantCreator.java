package BobcatLib.Subsystems.Swerve.AdvancedSwerve.Constants;


import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import BobcatLib.Subsystems.Swerve.AdvancedSwerve.Constants.SwerveConstants.KinematicsConstants;
import BobcatLib.Subsystems.Swerve.AdvancedSwerve.Constants.SwerveConstants.ModuleConfig;
import BobcatLib.Subsystems.Swerve.AdvancedSwerve.Constants.SwerveConstants.OdometryConstants;
import BobcatLib.Subsystems.Swerve.AdvancedSwerve.Constants.SwerveConstants.PIDConfigs;
import BobcatLib.Subsystems.Swerve.AdvancedSwerve.Constants.SwerveConstants.SpeedLimit;
import BobcatLib.Subsystems.Swerve.AdvancedSwerve.Constants.SwerveConstants.SwerveModuleConfigs;
import BobcatLib.Subsystems.Swerve.AdvancedSwerve.Constants.SwerveConstants.SwerveMotorConfig;
import BobcatLib.Subsystems.Swerve.AdvancedSwerve.Constants.SwerveConstants.SwervePIDConfig;
import BobcatLib.Subsystems.Swerve.AdvancedSwerve.Constants.SwerveConstants.SwerveSpeedLimits;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;

import java.io.File;
import java.io.IOException;

public class SwerveConstantCreator {

  public static SwerveConstants parseConstants(File jsonFile) throws IOException {

    ObjectMapper mapper = new ObjectMapper();
    JsonNode node;
    // read json file
    node = mapper.readTree(jsonFile);


    Rotation2d holoAlignTolerance =
        Rotation2d.fromRadians(node.get(JsonElements.holoAlignToleranceRads).asDouble(-1));

    KinematicsConstants kinematicsConstants =
        new KinematicsConstants(
            node.get(JsonElements.wheelBaseMeters).asDouble(-1),
            node.get(JsonElements.trackWidthMeters).asDouble(-1),
            node.get(JsonElements.wheelCircumferenceMeters).asDouble(-1));

    SwerveSpeedLimits swerveSpeedLimits =
        new SwerveSpeedLimits(
            new SpeedLimit(
                node.get(JsonElements.chassisMaxVelocityMPS).asDouble(-1),
                node.get(JsonElements.chassisMaxAccelMPS2).asDouble(-1),
                Rotation2d.fromRadians(
                    node.get(JsonElements.chassisMaxAngularVelocityRPS).asDouble(-1)),
                Rotation2d.fromRadians(
                    node.get(JsonElements.chassisMaxAngularAccelRPS2).asDouble(-1))),
            new SpeedLimit(
                node.get(JsonElements.moduleMaxVelocityMPS).asDouble(-1),
                node.get(JsonElements.moduleMaxAccelMPS2).asDouble(-1),
                Rotation2d.fromRadians(
                    node.get(JsonElements.moduleMaxAngularVelocityRPS).asDouble(-1)),
                Rotation2d.fromRadians(
                    node.get(JsonElements.moduleMaxAngularAccelRPS2).asDouble(-1))));

    PIDConfigs pidConfigs =
        new PIDConfigs(
            new SwervePIDConfig( // auto
                node.get(JsonElements.autoRotKP).asDouble(-1),
                node.get(JsonElements.autoRotKI).asDouble(-1),
                node.get(JsonElements.autoRotKD).asDouble(-1),
                node.get(JsonElements.autoTransKP).asDouble(-1),
                node.get(JsonElements.autoTransKI).asDouble(-1),
                node.get(JsonElements.autoTransKD).asDouble(-1)),
            new SwervePIDConfig( // teleop
                node.get(JsonElements.teleopRotKP).asDouble(-1),
                node.get(JsonElements.teleopRotKI).asDouble(-1),
                node.get(JsonElements.teleopRotKD).asDouble(-1),
                node.get(JsonElements.teleopTransKP).asDouble(-1),
                node.get(JsonElements.teleopTransKI).asDouble(-1),
                node.get(JsonElements.teleopTransKD).asDouble(-1)),
            new SwervePIDConfig( // autoalign
                node.get(JsonElements.autoAlignKP).asDouble(-1),
                node.get(JsonElements.autoAlignKI).asDouble(-1),
                node.get(JsonElements.autoAlignKD).asDouble(-1),
                0,
                0,
                0),
            new SwerveMotorConfig( // angle
                node.get(JsonElements.angleKP).asDouble(-1),
                node.get(JsonElements.angleKI).asDouble(-1),
                node.get(JsonElements.angleKD).asDouble(-1),
                node.get(JsonElements.angleKS).asDouble(-1),
                node.get(JsonElements.angleKV).asDouble(-1),
                node.get(JsonElements.angleKA).asDouble(-1),
                node.get(JsonElements.angleSupplyCurrentLimitEnable).asBoolean(),
                node.get(JsonElements.angleSupplyCurrentLimit).asDouble(-1),
                node.get(JsonElements.angleSupplyCurrentLimitThreshold).asDouble(-1),
                node.get(JsonElements.angleSupplyCurrentLimitTimeThreshold).asDouble(-1),
                node.get(JsonElements.angleStatorLimitEnable).asBoolean(),
                node.get(JsonElements.angleStatorLimit).asDouble(-1),
                0,
                0,
                node.get(JsonElements.angleGearRatio).asDouble(-1)),
            new SwerveMotorConfig( // drive
                node.get(JsonElements.driveKP).asDouble(-1),
                node.get(JsonElements.driveKI).asDouble(-1),
                node.get(JsonElements.driveKD).asDouble(-1),
                node.get(JsonElements.driveKS).asDouble(-1),
                node.get(JsonElements.driveKV).asDouble(-1),
                node.get(JsonElements.driveKA).asDouble(-1),
                node.get(JsonElements.driveSupplyCurrentLimitEnable).asBoolean(),
                node.get(JsonElements.driveSupplyCurrentLimit).asDouble(-1),
                node.get(JsonElements.driveSupplyCurrentLimitThreshold).asDouble(-1),
                node.get(JsonElements.driveSupplyCurrentLimitTimeThreshold).asDouble(-1),
                node.get(JsonElements.driveStatorLimitEnable).asBoolean(),
                node.get(JsonElements.driveStatorLimit).asDouble(-1),
                node.get(JsonElements.driveOpenLoopRamp).asDouble(),
                node.get(JsonElements.driveClosedLoopRamp).asDouble(),
                node.get(JsonElements.driveGearRatio).asDouble(-1)));

    OdometryConstants odometryConstants =
        new OdometryConstants(
            VecBuilder.fill(
                node.get(JsonElements.stateTrustPositionStdDevMeters).asDouble(-1),
                node.get(JsonElements.stateTrustPositionStdDevMeters).asDouble(-1),
                node.get(JsonElements.stateTrustRotationStdDevRads).asDouble(-1)),
            VecBuilder.fill(
                node.get(JsonElements.stateDistrustPositionStdDevMeters).asDouble(-1),
                node.get(JsonElements.stateDistrustPositionStdDevMeters).asDouble(-1),
                node.get(JsonElements.stateDistrustRotationStdDevRads).asDouble(-1)),
            node.get(JsonElements.odometryThrowoutAccel).asDouble(-1),
            node.get(JsonElements.odometryDistrustAccel).asDouble(-1));

    SwerveModuleConfigs moduleConfigs =
        new SwerveModuleConfigs(
            new ModuleConfig(
                node.get(JsonElements.flCancoderID).asInt(-1),
                node.get(JsonElements.flAngleID).asInt(-1),
                node.get(JsonElements.flDriveID).asInt(-1),
                Rotation2d.fromRadians(node.get(JsonElements.flOffsetRads).asDouble(-1))),
            new ModuleConfig(
                node.get(JsonElements.frCancoderID).asInt(-1),
                node.get(JsonElements.frAngleID).asInt(-1),
                node.get(JsonElements.frDriveID).asInt(-1),
                Rotation2d.fromRadians(node.get(JsonElements.frOffsetRads).asDouble(-1))),
            new ModuleConfig(
                node.get(JsonElements.blCancoderID).asInt(-1),
                node.get(JsonElements.blAngleID).asInt(-1),
                node.get(JsonElements.blDriveID).asInt(-1),
                Rotation2d.fromRadians(node.get(JsonElements.blOffsetRads).asDouble(-1))),
            new ModuleConfig(
                node.get(JsonElements.brCancoderID).asInt(-1),
                node.get(JsonElements.brAngleID).asInt(-1),
                node.get(JsonElements.brDriveID).asInt(-1),
                Rotation2d.fromRadians(node.get(JsonElements.brOffsetRads).asDouble(-1))));

    return new SwerveConstants(
        holoAlignTolerance,
        kinematicsConstants,
        swerveSpeedLimits,
        pidConfigs,
        odometryConstants,
        moduleConfigs,
        node.get(JsonElements.pigeonID).asInt(-1),
        node.get(JsonElements.useFOC).asBoolean(),
        node.get(JsonElements.canbus).asText(),
        node.get(JsonElements.angleInverted).asBoolean() ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive, //TODO this is dumb
        node.get(JsonElements.angleInverted).asBoolean()? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive,
        node.get(JsonElements.angleNeutralMode).asBoolean() ? NeutralModeValue.Brake : NeutralModeValue.Coast,
        node.get(JsonElements.angleNeutralMode).asBoolean() ? NeutralModeValue.Brake : NeutralModeValue.Coast,
        SensorDirectionValue.CounterClockwise_Positive //TODO this is even dumber
        );
    }
  

  public class JsonElements {
    public static final String angleNeutralMode = "angleNeutralMode";
    public static final String driveNeutralMode = "driveNeutralMode";
    public static final String cancoderDirection = "cancoderDirection";
    public static final String enableInitialReplanning = "enableInitialReplanning";
    public static final String enableDynamicReplanning = "enableDynamicReplanning";
    public static final String holoAlignToleranceRads = "holoAlignToleranceRads";
    public static final String wheelBaseMeters = "wheelBaseMeters";
    public static final String trackWidthMeters = "trackWidthMeters";
    public static final String wheelCircumferenceMeters = "wheelCircumferenceMeters";
    public static final String pigeonID = "pigeonID";
    public static final String useFOC = "useFOC";
    public static final String canbus = "canbus";

    public static final String chassisMaxVelocityMPS = "chassisMaxVelocityMPS";
    public static final String chassisMaxAccelMPS2 = "chassisMaxAccelMPS2";
    public static final String chassisMaxAngularVelocityRPS = "chassisMaxAngularVelocityRPS";
    public static final String chassisMaxAngularAccelRPS2 = "chassisMaxAngularAccelRPS2";
    public static final String moduleMaxVelocityMPS = "moduleMaxVelocityMPS";
    public static final String moduleMaxAccelMPS2 = "moduleMaxAccelMPS2";
    public static final String moduleMaxAngularVelocityRPS = "moduleMaxAngularVelocityRPS";
    public static final String moduleMaxAngularAccelRPS2 = "moduleMaxAngularAccelRPS2";

    public static final String autoRotKP = "autoRotKP";
    public static final String autoRotKI = "autoRotKI";
    public static final String autoRotKD = "autoRotKD";
    public static final String autoTransKP = "autoTransKP";
    public static final String autoTransKI = "autoTransKI";
    public static final String autoTransKD = "autoTransKD";

    public static final String teleopRotKP = "teleopRotKP";
    public static final String teleopRotKI = "teleopRotKI";
    public static final String teleopRotKD = "teleopRotKD";
    public static final String teleopTransKP = "teleopTransKP";
    public static final String teleopTransKI = "teleopTransKI";
    public static final String teleopTransKD = "teleopTransKD";

    public static final String autoAlignKP = "autoAlignKP";
    public static final String autoAlignKI = "autoAlignKI";
    public static final String autoAlignKD = "autoAlignKD";

    public static final String angleKP = "angleKP";
    public static final String angleKI = "angleKI";
    public static final String angleKD = "angleKD";
    public static final String angleKS = "angleKS";
    public static final String angleKV = "angleKV";
    public static final String angleKA = "angleKA";
    public static final String angleInverted = "angleInverted";
    public static final String angleShouldCoast = "angleShouldCoast";
    public static final String angleSupplyCurrentLimitEnable = "angleSupplyCurrentLimitEnable";
    public static final String angleSupplyCurrentLimit = "angleSupplyCurrentLimit";
    public static final String angleSupplyCurrentLimitThreshold =
        "angleSupplyCurrentLimitThreshold";
    public static final String angleSupplyCurrentLimitTimeThreshold =
        "angleSupplyCurrentLimitTimeThreshold";
    public static final String angleStatorLimitEnable = "angleStatorLimitEnable";
    public static final String angleStatorLimit = "angleStatorLimit";
    public static final String angleGearRatio = "angleGearRatio";

    public static final String driveKP = "driveKP";
    public static final String driveKI = "driveKI";
    public static final String driveKD = "driveKD";
    public static final String driveKS = "driveKS";
    public static final String driveKV = "driveKV";
    public static final String driveKA = "driveKA";
    public static final String driveInverted = "driveInverted";
    public static final String driveShouldCoast = "driveShouldCoast";
    public static final String driveSupplyCurrentLimitEnable = "driveSupplyCurrentLimitEnable";
    public static final String driveSupplyCurrentLimit = "driveSupplyCurrentLimit";
    public static final String driveSupplyCurrentLimitThreshold =
        "driveSupplyCurrentLimitThreshold";
    public static final String driveSupplyCurrentLimitTimeThreshold =
        "driveSupplyCurrentLimitTimeThreshold";
    public static final String driveStatorLimitEnable = "driveStatorLimitEnable";
    public static final String driveStatorLimit = "driveStatorLimit";
    public static final String driveOpenLoopRamp = "driveOpenLoopRamp";
    public static final String driveClosedLoopRamp = "driveClosedLoopRamp";
    public static final String driveGearRatio = "driveGearRatio";

    public static final String stateTrustPositionStdDevMeters = "stateTrustPositionStdDevMeters";
    public static final String stateTrustRotationStdDevRads = "stateTrustRotationStdDevRads";
    public static final String stateDistrustPositionStdDevMeters =
        "stateDistrustPositionStdDevMeters";
    public static final String stateDistrustRotationStdDevRads = "stateDistrustRotationStdDevRads";
    public static final String odometryThrowoutAccel = "odometryThrowoutAccel";
    public static final String odometryDistrustAccel = "odometryDistrustAccel";

    public static final String flCancoderID = "flCancoderID";
    public static final String flDriveID = "flDriveID";
    public static final String flAngleID = "flAngleID";
    public static final String flOffsetRads = "flOffsetRads";

    public static final String frCancoderID = "frCancoderID";
    public static final String frDriveID = "frDriveID";
    public static final String frAngleID = "frAngleID";
    public static final String frOffsetRads = "frOffsetRads";

    public static final String blCancoderID = "blCancoderID";
    public static final String blDriveID = "blDriveID";
    public static final String blAngleID = "blAngleID";
    public static final String blOffsetRads = "blOffsetRads";

    public static final String brCancoderID = "brCancoderID";
    public static final String brDriveID = "brDriveID";
    public static final String brAngleID = "brAngleID";
    public static final String brOffsetRads = "brOffsetRads";
  }
}
