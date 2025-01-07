package BobcatLib.Utilities;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

/**
 * A wrapper class for managing an immutable, sorted list of set points (doubles) and providing utility methods.
 */
public final class SetPointWrapper {
    private final List<Double> setPoints;

    /**
     * Constructs a SetPointWrapper from a comma-separated string of points.
     * The points are parsed, sorted, and stored in an immutable list.
     *
     * @param points a comma-separated string of double values.
     * @throws NumberFormatException if any value in the string cannot be parsed as a double.
     */
    public SetPointWrapper(String points) {
        this.setPoints = Collections.unmodifiableList(
            Arrays.stream(points.split(","))
                  .map(Double::parseDouble)
                  .sorted()
                  .collect(Collectors.toList())
        );
    }

    /**
     * Constructs a SetPointWrapper from a list of points.
     * The list is copied, sorted, and stored in an immutable list.
     *
     * @param points a list of double values.
     */
    public SetPointWrapper(List<Double> points) {
        this.setPoints = Collections.unmodifiableList(
            points.stream()
                  .sorted()
                  .collect(Collectors.toList())
        );
    }

    /**
     * Retrieves the set point at the specified index.
     *
     * @param index the index of the set point.
     * @return the set point at the specified index.
     * @throws IndexOutOfBoundsException if the index is out of bounds.
     */
    public double get(int index) {
        return setPoints.get(index);
    }

    /**
     * Retrieves the next set point relative to the given index.
     * If the index is the last one, it returns the same set point.
     *
     * @param index the current index.
     * @return the next set point or the current set point if at the end of the list.
     */
    public double next(int index) {
        return setPoints.get(Math.min(index + 1, setPoints.size() - 1));
    }

    /**
     * Retrieves the previous set point relative to the given index.
     * If the index is the first one, it returns the same set point.
     *
     * @param index the current index.
     * @return the previous set point or the current set point if at the beginning of the list.
     */
    public double previous(int index) {
        return setPoints.get(Math.max(index - 1, 0));
    }

    /**
     * Finds the nearest set point that is greater than or equal to the given value.
     * If no such set point exists, returns the last set point.
     *
     * @param current the value to compare against the set points.
     * @return the nearest set point that is greater than or equal to the given value.
     */
    public double findNearestSetPoint(double current) {
        return setPoints.stream()
                .filter(e -> e >= current)
                .findFirst()
                .orElse(setPoints.get(setPoints.size() - 1));
    }

    /**
     * Returns an immutable copy of the set points.
     *
     * @return an unmodifiable list of the set points.
     */
    public List<Double> getPoints() {
        return setPoints;
    }
}
