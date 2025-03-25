package frc.robot;
public class AngleModTest {

    public static void main(String args[]) {
        var angle = 0;
        var offset = -5.01234;

        var result = (angle + offset) % (2* Math.PI);

        System.out.println(result);
    }

}
