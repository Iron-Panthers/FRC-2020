package com.ironpanthers.util;

public class Util {
    private Util() {
        throw new RuntimeException("Util is a utility class! Don't try to construct an instance of Util");
    }

    private static double kEpsilon = 1e-9;

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, kEpsilon);
    }

    public static boolean epsilonEquals(int a, int b, int epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }
}