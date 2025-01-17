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
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import org.junit.jupiter.api.Test;

import frc.robot.ElevatorSubsystem;
import frc.robot.SetPointWrapper;
import frc.robot.Parser.ElevatorModuleJson;
import frc.robot.Parser.ElevatorModuleJson.ElevatorJson;
public class Parser_test {
    ElevatorSubsystem elevator;
    /**
     * @hidden
     */
    @Test
    void construct_elevatorSubsystem() {
        elevator = new ElevatorSubsystem();
        assertNotNull(elevator);
    }

    /**
     * @hidden
     */
    @Test
    public void load_configurations_test() {
      elevator = new ElevatorSubsystem();
      assertNotNull(elevator);
      elevator.loadConfigurationFromFile();
      assertNotNull(elevator.elevatorJson);
      assertNotNull(elevator.elevatorJson.elevator);
      assertNotNull(elevator.elevatorJson.elevatorPid);
      assertNotNull(elevator.elevatorJson.limits);
      assertNotNull(elevator.elevatorJson.motor);

    }
}