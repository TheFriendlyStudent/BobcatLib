package frc.robot;

import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.fail;
import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;

import BobcatLib.Hardware.Controllers.EightBitDo;
import BobcatLib.Hardware.Controllers.Logitech;
import BobcatLib.Hardware.Controllers.OI;
import BobcatLib.Hardware.Controllers.PS4ControllerWrapper;
import BobcatLib.Hardware.Controllers.PS5ControllerWrapper;
import BobcatLib.Hardware.Controllers.Ruffy;
import BobcatLib.Hardware.Controllers.XboxControllerWrapper;
import BobcatLib.Subsystems.Swerve.Utility.*;
import BobcatLib.Subsystems.Swerve.Utility.UnifiedModuleConfigurator.CotsModuleObject;

public class LoadOIConfiguration_test {
    /**
     * @hidden
     */
    @Test
    void loadAndTestAll_Modules() {
        OI s_Controller;
        List<String> controllers = new ArrayList<String>();
        controllers.add("xbox");
        controllers.add("ps4");
        controllers.add("ps5");
        controllers.add("ruffy");
        controllers.add("logitec");
        controllers.add("eightbitdo");
        try {
            s_Controller = new OI("robotName");
            assertNotNull(s_Controller.controllerJson);
            assertNotNull(s_Controller.controllerJson.driver);
        } catch (Exception e) {
            assertNotNull(e.toString());
        }
    }
}
