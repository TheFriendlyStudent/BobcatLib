package BobcatLib.Subsystems.Elevators.Modules;

import BobcatLib.Subsystems.Elevators.Modules.Motors.BaseElevatorMotor;
import BobcatLib.Utilities.SetPointWrapper;
import edu.wpi.first.math.geometry.Rotation2d;

public class ElevatorModuleReal implements ElevatorModuleIO {
    private BaseElevatorMotor motor;
    private double lowerLimit, upperLimit, currentSetPoint;
    private SetPointWrapper setPoints;

    public ElevatorModuleReal() {
        this.setPoints = setPoints;
    }

    /**
     * Moves the elevator to the specified position.
     *
     * @param position The desired position of the elevator as a {@link Rotation2d}
     *                 object.
     */
    public void moveElevator(Rotation2d position) {
        motor.setAngle(position.getRotations());
    }

    public void moveElevatorToNext() {
        Rotation2d currentPosition = Rotation2d.fromRotations(motor.getPosition());
        Rotation2d nextPosition = Rotation2d.fromRotations(setPoints.getSurroundingPoints(currentPosition).get(1));
        moveElevator(nextPosition);
    }

    public void moveElevatorToPrevious() {
        Rotation2d currentPosition = Rotation2d.fromRotations(motor.getPosition());
        Rotation2d previousPosition = Rotation2d.fromRotations(setPoints.getSurroundingPoints(currentPosition).get(0));
        moveElevator(previousPosition);
    }

    public void holdPosition() {
        Rotation2d currentPosition = Rotation2d.fromRotations(motor.getPosition());
        moveElevator(currentPosition);
    }

    public Rotation2d getPosition() {
        return Rotation2d.fromRotations(motor.getPosition());
    }

    public Rotation2d getCurrentSetPoint() {
        return Rotation2d.fromRotations(currentSetPoint);
    }

    /** Stops the elevator motor immediately. */
    public void stop() {
        motor.stopMotor();
    }
}
