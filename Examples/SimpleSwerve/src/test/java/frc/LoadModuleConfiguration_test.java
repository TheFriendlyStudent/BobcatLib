package frc;

import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.fail;
import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;

import BobcatLib.Subsystems.Swerve.Utility.*;
import BobcatLib.Subsystems.Swerve.Utility.UnifiedModuleConfigurator.CotsModuleObject;

public class LoadModuleConfiguration_test {
    /**
     * @hidden
     */
    @Test
    void loadAndTestAll_Modules() {
        List<String> levels = new ArrayList<String>();
        List<String> motorTypes = new ArrayList<String>();
        List<String> moduleTypes = new ArrayList<String>();
        try {
            levels.add("L1");
            levels.add("L2");
            levels.add("L3");
            levels.add("L4");
            for (String level_element : levels) {
                motorTypes.add("Falcon500");
                motorTypes.add("KrakenX60");
                motorTypes.add("Neo");
                motorTypes.add("Vortex");
                for (String motor_element : motorTypes) {
                    moduleTypes.add("mk4");
                    moduleTypes.add("mk4i");
                    for (String module_element : moduleTypes) {
                        CotsModuleSwerveConstants cotsmodule = new CotsModuleObject()
                                .withConfiguration("sds", module_element, motor_element, level_element).to();
                        assertNotNull(cotsmodule.driveGearRatio);
                    }
                }
            }

            levels = new ArrayList<String>();
            levels.add("X1_10");
            levels.add("X1_11");
            levels.add("X1_12");
            levels.add("X2_10");
            levels.add("X2_11");
            levels.add("X2_12");
            levels.add("X3_10");
            levels.add("X3_11");
            levels.add("X3_12");
            levels.add("X4_10");
            levels.add("X4_11");
            levels.add("X4_12");
            for (String level_element : levels) {
                for (String motor_element : motorTypes) {
                    moduleTypes = new ArrayList<String>();
                    moduleTypes.add("SwerveX2i");
                    for (String module_element : moduleTypes) {
                        CotsModuleSwerveConstants cotsmodule = new CotsModuleObject()
                                .withConfiguration("wcp", module_element, motor_element, level_element).to();
                        assertNotNull(cotsmodule.driveGearRatio);
                    }
                }
            }
        } catch (Exception e) {
            assertNotNull(e.toString());
        }
    }
}
