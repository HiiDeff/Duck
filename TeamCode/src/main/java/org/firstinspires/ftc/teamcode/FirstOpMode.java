package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Arrays;

@TeleOp(name = "First Op-Mode", group = "Example")
public class FirstOpMode extends LinearOpMode {

    private GamePad gp1, gp2;
    private double armLiftPos = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumRobot robot = new MecanumRobot(hardwareMap);
        robot.setArmLiftPos(armLiftPos);
        gp1 = new GamePad(gamepad1);
        gp2 = new GamePad(gamepad2);
        telemetry.addData("arm lift pos", armLiftPos);
        telemetry.addData("color detected", Arrays.toString(robot.getRGB()));
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            if (gp1.onceDpadUp()) armLiftPos -= 0.03;
            else if (gp1.onceDpadDown()) armLiftPos += 0.03;
            robot.setArmLiftPos(armLiftPos);
            gp1.update();
            gp2.update();
            telemetry.addData("arm lift pos", armLiftPos);
            telemetry.addData("color detected", Arrays.toString(robot.getRGB()));
            telemetry.update();
        }
    }
}
