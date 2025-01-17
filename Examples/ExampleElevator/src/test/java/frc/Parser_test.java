package frc.robot;

import static edu.wpi.first.units.Units.Rotation;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.io.File;
import java.io.IOException;

import com.fasterxml.jackson.databind.ObjectMapper;

import BobcatLib.Subsystems.Elevators.ElevatorSubsystem;
import BobcatLib.Subsystems.Elevators.Modules.BaseElevator;
import BobcatLib.Subsystems.Elevators.Modules.ElevatorModuleReal;
import BobcatLib.Subsystems.Elevators.Parser.ElevatorModuleJson;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Parser.SwerveJson;
import BobcatLib.Utilities.SetPointWrapper;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import org.junit.jupiter.api.Test;

public class Parser_test {
    ElevatorSubsystem elevator;

    /**
     * @hidden
     */
    @Test
    void construct_elevatorSubsystem() {
        elevator = new ElevatorSubsystem("Elevator", new BaseElevator(new ElevatorModuleReal()),
                new SetPointWrapper("0.25,50"));
        assertNotNull(elevator);
    }

    /**
     * @hidden
     */
    @Test
    public void load_configurations_test() {
        ElevatorModuleJson module = new ElevatorModuleJson();
        File deployDirectory = Filesystem.getDeployDirectory();
        assert deployDirectory.exists();
        File directory = new File(deployDirectory, "configs/Elevator/");
        assert directory.exists();
        assert new File(directory, "elevator.json").exists();
        File swerveFile = new File(directory, "elevator.json");
        assert swerveFile.exists();
        try {
            module = new ObjectMapper().readValue(swerveFile, ElevatorModuleJson.class);
        } catch (IOException e) {
          e.printStackTrace();
        }    
        assertNotNull( module );
        assertNotNull(module.elevator);
        assertNotNull(module.elevatorPid);
        assertNotNull(module.motor);
        assertNotNull(module.limits);
    }
}