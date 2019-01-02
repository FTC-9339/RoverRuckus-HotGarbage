package org.firstinspires.ftc.teamcode.RogueOpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Rammy Boi Autonomous", group = "Main Autonomous")
public class RammyOP extends LinearOpMode {

    private DcMotor FL,
                    FR,
                    BL,
                    BR = null;

    @Override
    public void runOpMode() {
        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BL = hardwareMap.dcMotor.get("BL");
        BR = hardwareMap.dcMotor.get("BR");

        telemetry.addLine("Initialized motors successfully.");
        telemetry.update();

        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("Init phase clear.");
        telemetry.update();

        waitForStart();

        telemetry.clear();
        telemetry.addLine("Starting AutonomousUtils");
        telemetry.update();

        double targetRuntime = getRuntime() + 4;

        while (opModeIsActive() && getRuntime() < targetRuntime) {
            FL.setPower(1.0);
            BL.setPower(1.0);
            FR.setPower(1.0);
            BR.setPower(1.0);
        }

        telemetry.clear();
        telemetry.addLine("R A M M E D");
    }

}
