package org.firstinspires.ftc.teamcode.prototypes;

import android.app.Activity;
import android.app.Application;
import android.content.Context;
import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.util.Objects;

public class Runner extends LinearOpMode
{
    private Runner()
    {

    }

    public static Runner getRunner()
    {
        return runner;
    }

    private static Runner runner;

    public static void main(String[] args)
    {
        runner = new Runner();
        try
        {
            runner.runOpMode();
        } catch (InterruptedException e)
        {
            e.printStackTrace();
        }

    }

    /**
     * Override this method and place your code here.
     * <p>
     * Please do not swallow the InterruptedException, as it is used in cases
     * where the op mode needs to be terminated early.
     *
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException
    {

    }

}
