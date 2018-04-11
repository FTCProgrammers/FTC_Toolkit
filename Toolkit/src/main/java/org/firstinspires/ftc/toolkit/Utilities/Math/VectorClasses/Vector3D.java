package org.firstinspires.ftc.toolkit.Utilities.Math.VectorClasses;

import static java.lang.Math.*;

/**
 * @author Nathan Glover
 * A vector containing three components
 */
public class Vector3D {
    /**
     * A constant for an empty vector
     *
     */
    public static final Vector3D ZERO = new Vector3D();

    /**
     * A component of this vector
     */
    public double x, y, z;

    /**
     * Constructs an empty vector
     */
    public Vector3D() {
        this(0, 0, 0);
    }

    /**
     * Constructs a vector with the desired components
     *
     * @param x The first component
     * @param y The second component
     * @param z The third component
     */

    public Vector3D(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public double dot(Vector3D multiplier) {
        return (x * multiplier.x) + (y * multiplier.y) + (z * multiplier.z);
    }

    /**
     * Returns the cross product of this vector and another.
     * Note that cross product is only defined for 3 dimensional
     * vectors
     *
     * @param multiplier The vector with which this one will be multiplied
     * @return The cross product of the two vectors
     */

    public Vector3D cross(Vector3D multiplier) {
        return new Vector3D(
                (y * multiplier.z) - (z * multiplier.y),
                (z * multiplier.x) - (x * multiplier.z),
                (x * multiplier.y) - (y * multiplier.x)
        );
    }

    public Vector3D plus(Vector3D addition) {
        return new Vector3D(x + addition.x, y + addition.y, z + addition.z);
    }

    public Vector3D minus(Vector3D subtraction) {
        return new Vector3D(x - subtraction.x, y - subtraction.y, z - subtraction.z);
    }

    public Vector3D times(double scalar) {
        return new Vector3D(x * scalar, y * scalar, z * scalar);
    }

    public double magnitude() {
        return sqrt((x * x) + (y * y) + (z * z));
    }

    public Vector3D normalize() {
        double length = magnitude();
        return new Vector3D(x / length, y / length, z / length);
    }

    private boolean isEqual(Object vector) {
        Vector3D comparator = (Vector3D) vector;
        return (x == comparator.x) && (y == comparator.y) && (z == comparator.z);
    }

    @Override
    public String toString() {
        return "(\n" +
                "\tX: " + x + ",\n" +
                "\tY: " + y + ",\n" +
                "\tZ: " + z +
                "\n)";
    }

    @Override
    public boolean equals(Object object) {
        return object instanceof Vector3D && this.isEqual(object);
    }
}
