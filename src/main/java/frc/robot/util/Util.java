package frc.robot.util;

public class Util {

	/**
	 * @param s
	 * @return String
	 */
	public static String removeFileExtention(String s) {
		int i = s.indexOf(".");
		return s.substring(0, i);
	}
}
