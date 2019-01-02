package com.roguemafia.rogueftc.robot.drive

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

interface DriveBase {

    /**
     * A simple drive function to move forwards and backwards
     * @param pow The power to apply to the motors. Negative means backwards movement.
     */
    open fun basicStandardDrive(pow: Double)

    /**
     * A simple drive function for Autonomous to move forward and backwards towards a certain distance.
     *
     * NOTE: This will probably require encoders, but a custom implementation of this may be able to
     * utilize time.
     *
     * @param pow The power to apply to the motors. Negative means backwards movement.
     * @param distance The distance to drive. Negative means backwards movement (doesn't negate to positive if power is also negative).
     */
    open fun basicAutoDrive(pow: Double, distance: DistanceUnit)

    /**
     * An advanced drive function that has directional functionality.
     *
     * @param pow The power to apply to the motors.
     * @param distance The distance to drive.
     * @param direction The direction in which to drive in.
     *
     * @throws InvalidDirectionException
     */
    open fun advAutoDrive(pow: Double, distance: DistanceUnit, direction: DriveDirection)
}

enum class DriveDirection {
    FORWARD, BACKWARDS, LEFT, RIGHT, ROTATE_CCW, ROTATE_CW, DIAGONAL_FL, DIAGONAL_FR, DIAGONAL_BL, DIAGONAL_BR
}

class InvalidDirectionException(message: String?) : Exception(message)