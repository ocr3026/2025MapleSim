package frc.robot.subsystems.drive;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class Drive {
	static final Lock odometryLock = new ReentrantLock();
}
