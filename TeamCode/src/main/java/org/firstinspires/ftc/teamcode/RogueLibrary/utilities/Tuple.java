package org.firstinspires.ftc.teamcode.RogueLibrary.utilities;

/**
 * Simple class implementing simple Tuple functionality.
 * @param <X> Class of the x member.
 * @param <Y> Class of the y member.
 */
public class Tuple<X,Y> {
    public final X x;
    public final Y y;

    public Tuple(X x, Y y) {
        this.x = x;
        this.y = y;
    }
}
