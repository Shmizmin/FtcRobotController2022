package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.MecanumBot;
import org.firstinspires.ftc.teamcode.util.Robot;

@TeleOp(name = "Mecanum Drive", group = "TeleOp")
public class MecanumDrive extends OpMode
{
    private Robot robot = null;
    private FtcDashboard dashboard = null;

    @Override
    public void init()
    {
        robot = new Robot(hardwareMap);
        dashboard = FtcDashboard.getInstance();

        telemetry.addLine("Robot Initialization Complete");
        telemetry.update();
    }

    @Override
    public void loop()
    {
        //transmit a packet containing robot pose to FTCDashboard
        {


        }

        robot.onUpdate(gamepad1, gamepad2);
    }

    @Override
    public void stop()
    {

    }
}
