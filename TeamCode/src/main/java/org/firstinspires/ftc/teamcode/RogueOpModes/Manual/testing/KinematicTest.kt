package org.firstinspires.ftc.teamcode.RogueOpModes.Manual.testing

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.pow

@TeleOp(name = "Kinematic Test", group = "Testing")
class KinematicTest(): LinearOpMode() {

    private val wheelRadiusM = DistanceUnit.INCH.toMeters(2.0)

    private val wheelSeperation = (DistanceUnit.INCH.toMeters(13.0/2) + DistanceUnit.INCH.toMeters(11.5/2))

    private var wheelVelocities: Array<Double> = emptyArray()

    override fun runOpMode() {
        val BRMotor: DcMotorEx = hardwareMap.dcMotor.get("BR") as DcMotorEx
        val BLMotor: DcMotorEx = hardwareMap.dcMotor.get("BL") as DcMotorEx
        val FRMotor: DcMotorEx = hardwareMap.dcMotor.get("FR") as DcMotorEx
        val FLMotor: DcMotorEx = hardwareMap.dcMotor.get("FL") as DcMotorEx

        BRMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        BLMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        FRMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        FLMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        BRMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        BLMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        FRMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        FLMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER

        BRMotor.direction = DcMotorSimple.Direction.REVERSE
        FRMotor.direction = DcMotorSimple.Direction.REVERSE

        waitForStart()

        while (opModeIsActive()) {

            gamepad1.apply {
                left_stick_y = -joystickTransform(left_stick_y)
                left_stick_x = joystickTransform(left_stick_x)
                right_stick_x = joystickTransform(right_stick_x)
            }

            wheelVelocities = kinematicModel(-gamepad1.left_stick_y.toDouble(), gamepad1.left_stick_x.toDouble(), gamepad1.right_stick_x.toDouble() * (2 * PI))

            FLMotor.setVelocity(wheelVelocities[0] * 0.6, AngleUnit.RADIANS)
            FRMotor.setVelocity(wheelVelocities[1] * 0.6, AngleUnit.RADIANS)
            BLMotor.setVelocity(wheelVelocities[2] * 0.6, AngleUnit.RADIANS)
            BRMotor.setVelocity(wheelVelocities[3] * 0.6, AngleUnit.RADIANS)

            telemetry.addData("FLMotor angular velocity:", wheelVelocities[0])
            telemetry.update()
        }
    }

    private fun kinematicModel(vX: Double, vY: Double, vW: Double): Array<Double> {
        val FLVelocity = (1/wheelRadiusM) * (vX - vY - (wheelSeperation * vW))
        val FRVelocity = (1/wheelRadiusM) * (vX + vY + (wheelSeperation * vW))
        val BLVelocity = (1/wheelRadiusM) * (vX + vY - (wheelSeperation * vW))
        val BRVelocity = (1/wheelRadiusM) * (vX - vY + (wheelSeperation * vW))

        return arrayOf(FLVelocity, FRVelocity, BLVelocity, BRVelocity)
    }

    private fun joystickTransform(x: Float) =
            x.pow(3)/ abs(x)
}