package org.firstinspires.ftc.teamcode.RogueOpModes.Autonomous.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Telemetry Test", group = "Testing")
public class telemetrytest extends LinearOpMode {
    @Override
    public void runOpMode() {
        for (int i = 1; i < 21; i++) {
            telemetry.addLine("" + i);
        }
        telemetry.update();

        waitForStart();

        while (opModeIsActive());
    }
}
