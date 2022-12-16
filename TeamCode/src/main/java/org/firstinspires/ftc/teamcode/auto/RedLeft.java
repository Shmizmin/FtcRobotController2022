package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.CameraOpenCV.Position;

@Autonomous(name = "Red Left")
public class RedLeft extends AutoAbstract
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        onInit();

        turner.servo.setPosition(1.0);

        sleep(750);

        camera.webcam.stopStreaming();

        lifter.motor.setTargetPosition(-200);
        lifter.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.motor.setPower(0.6);
        sleep(250);

        forward(1.5, 2.0);

        right(11.5, 4.0);

        lifter.motor.setTargetPosition(-635);
        lifter.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.motor.setPower(0.6);
        sleep(500);

        forward(5.5, 3.0);

        turner.servo.setPosition(0.0);

        sleep(500);

        backward(5.5, 3.0);

        lifter.motor.setPower(0.0);
        lifter.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        left(11.5, 4.0);



        switch (camera.scanner.position)
        {
            case RED:
            {
                left(24.0, 5.0);
            } break;

            case GREEN:
            {
                // no movement for now
            } break;

            case BLUE:
            {
                right(24.0, 5.0);
            } break;
        }

        forward(30.0, 10.0);
    }
}
