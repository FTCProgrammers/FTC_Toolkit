package org.firstinspires.ftc.toolkit.Utilities.Math.VectorClasses;

import static java.lang.Math.*;

/**
 * @author Nathaniel Glover
 * A vector containing two components
 */

public class Vector2D {
    /**
     * A constant for an empty vector
     */
    public static final Vector2D ZERO = new Vector2D();

    /**
     * A component of this vector
     */
    public double x, y;

    /**
     * Constructs an empty vector
     */
    private Vector2D() {
        this(0, 0);
    }

    /**
     * Constructs a vector with the desired components
     *
     * @param x The first component
     * @param y The second component
     */

    public Vector2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double dot(Vector2D multiplier) {
        return (x * multiplier.x) + (y * multiplier.y);
    }

    public Vector2D plus(Vector2D addition) {
        return new Vector2D(x + addition.x, y + addition.y);
    }

    public Vector2D minus(Vector2D subtraction) {
        return new Vector2D(x - subtraction.x, y - subtraction.y);
    }

    public Vector2D times(double scalar) {
        return new Vector2D(x * scalar, y * scalar);
    }

    public double magnitude() {
        return sqrt((x * x) + (y * y));
    }

    public Vector2D normalize() {
        double length = magnitude();
        return new Vector2D(x / length, y / length);
    }

    private boolean isEqual(Object vector) {
        Vector2D comparator = (Vector2D) vector;
        return (x == comparator.x) && (y == comparator.y);
    }

    @Override
    public String toString() {
        return "(\n" +
                "\tX: " + x + ",\n" +
                "\tY: " + y +
                "\n)";
    }

    @Override
    public boolean equals(Object object) {
        return object instanceof Vector2D && this.isEqual(object);
    }
}
