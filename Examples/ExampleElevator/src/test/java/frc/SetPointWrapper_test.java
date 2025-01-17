package frc.robot;

import static edu.wpi.first.units.Units.Rotation;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;

public class SetPointWrapper_test {
    private SetPointWrapper setPointWrapper;

    /**
     * @hidden
     */
    @Test
    void construct_setpointwrapper() {
        setPointWrapper = new SetPointWrapper("0.0,10.0,20.0,30.0,40.0,50.0");
        assertNotNull(setPointWrapper.getPoints());
        assertTrue(setPointWrapper.getPoints().size() == 6);
        List<Double> points = Arrays.asList(new Double[] { 0.0, 10.0, 20.0, 30.0, 40.0, 50.0 });
        setPointWrapper = new SetPointWrapper(points);
        assertNotNull(setPointWrapper.getPoints());
        assertTrue(setPointWrapper.getPoints().size() == 6);
    }

    /**
     * @hidden
     */
    @Test
    void get_setpointwrapper() {
        setPointWrapper = new SetPointWrapper("0.0,10.0,20.0,30.0,40.0,50.0");
        assertNotNull(setPointWrapper.get(0));
        assertTrue(setPointWrapper.get(0) == 0.0);
        assertTrue(setPointWrapper.get(1) == 10.0);
    }

    /**
     * @hidden
     */
    @Test
    void getPoints_setpointwrapper() {
        setPointWrapper = new SetPointWrapper("0.0,10.0,20.0,30.0,40.0,50.0");
        assertNotNull(setPointWrapper.getPoints());
        assertTrue(setPointWrapper.getPoints().get(0) == 0.0);
        assertTrue(setPointWrapper.getPoints().get(1) == 10.0);
    }

    /**
     * @hidden
     */
    @Test
    void next_setpointwrapper() {
        setPointWrapper = new SetPointWrapper("0.0,10.0,20.0,30.0,40.0,50.0");
        assertNotNull(setPointWrapper.next(0));
        assertNotNull(setPointWrapper.next(2));
        assertNotNull(setPointWrapper.next(5));
        assertTrue(setPointWrapper.next(0) == 10.0);
        assertTrue(setPointWrapper.next(2) == 30.0);
        assertTrue(setPointWrapper.next(5) == 50.0);
    }

    /**
     * @hidden
     */
    @Test
    void previous_setpointwrapper() {
        setPointWrapper = new SetPointWrapper("0.0,10.0,20.0,30.0,40.0,50.0");
        assertNotNull(setPointWrapper.previous(0));
        assertNotNull(setPointWrapper.previous(2));
        assertNotNull(setPointWrapper.previous(5));
        assertTrue(setPointWrapper.previous(0) == 0.0);
        assertTrue(setPointWrapper.previous(2) == 10.0);
        assertTrue(setPointWrapper.previous(5) == 40.0);
    }

    /**
     * @hidden
     */
    @Test
    void findNearestSetPoint_setpointwrapper() {
        setPointWrapper = new SetPointWrapper("0.0,10.0,20.0,30.0,40.0,50.0");
        assertNotNull(setPointWrapper.findNearestSetPoint(0.1));
        assertNotNull(setPointWrapper.findNearestSetPoint(49.0));
        assertTrue(setPointWrapper.findNearestSetPoint(0.1) == 0.0);
        assertTrue(setPointWrapper.findNearestSetPoint(49.1) == 50.0);
    }

    /**
     * @hidden
     */
    @Test
    void getIndexFromValue_setpointwrapper() {
        setPointWrapper = new SetPointWrapper("0.0,10.0,20.0,30.0,40.0,50.0");
        assertNotNull(setPointWrapper.getIndexFromValue(0.0));
        assertNotNull(setPointWrapper.getIndexFromValue(20.0));
        assertNotNull(setPointWrapper.getIndexFromValue(50.0));
        assertTrue(setPointWrapper.getIndexFromValue(0.0) == 0);
        assertTrue(setPointWrapper.getIndexFromValue(20.0) == 2);
        assertTrue(setPointWrapper.getIndexFromValue(50.0) == 5);
    }

    /**
     * @hidden
     */
    @Test
    void getSurroundingPoints_setpointwrapper() {
        setPointWrapper = new SetPointWrapper("0.0,10.0,20.0,30.0,40.0,50.0");
        assertNotNull(setPointWrapper.getSurroundingPoints(new Rotation2d(0.0)).get(0));
        assertNotNull(setPointWrapper.getSurroundingPoints(new Rotation2d(0.0)).get(1));
        List<Double> points = setPointWrapper.getSurroundingPoints(new Rotation2d(0.0));
        assertTrue(points.get(0) == 0.0);
        assertTrue(points.get(1) == 10.0);
        points = setPointWrapper.getSurroundingPoints(Rotation2d.fromRotations(10.0));
        assertTrue(points.get(0) == 0.0);
        assertTrue(points.get(1) == 20.0);
        points = setPointWrapper.getSurroundingPoints(Rotation2d.fromRotations(50.0));
        assertTrue(points.get(0) == 40.0);
        assertTrue(points.get(1) == 50.0);
    }

}
