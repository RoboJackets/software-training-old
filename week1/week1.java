// to compile this file do
//        javac week1.java
// to run this file do
//        java week1
public class week1 {

    //////////Global Variables//////////

    //////////Primitive Data Types//////////
    public String myName = "George Burdell";
    public char middleInitial = 'P';
    //This is a comment!
    /*
    This is a block comment...
    By the way there are more number types like
    byte, short, long, and float
    but you'll run into int and double the most
    */
    public int yearsInRoboJackets = 3;
    public double myBirthday = 8.11;
    public boolean isRoboJacketsCool = true;

    //////////Arrays//////////
    //You can do this with any primitive data type
    public int[] membersInRoboJackets = new int[200];


    public static void main(String[] args) {
        //This is the main function that runs any logic!
        //////////Local Variables/////////
        int num = 0;
        int length = 0;
        int count = 0;
        String str = "";
        double[] gpas = {1.0, 2.6, 4.0, 3.6};


        //////////Operators//////////
        System.out.println((3 + 3 == 9));
        System.out.println((2 + 2 != 5));
        System.out.println(!true);
        System.out.println(true || false);
        System.out.println(true && false);

        //If-ElseIf-Else
        if (num == 0) {
            num = 3;
            System.out.println("I understand Loops!");
        } else if (num > 4) {
            num += 3;
            System.out.println("Like John Cena, you can't see me");
        } else {
            num--;
            System.out.println("Like George P. Burdell, I don't ever show up");
        }

        //////////Loops/////////
        //For
        for (int n = 0; n < gpas.length; n++) {
            System.out.println("GPA: " + gpas[n]);
        }

        //While
        while (length < gpas.length) {
            System.out.println("Adjusted GPA: " + (gpas[length] + 2));
            length++;
        }

        //Do-While
        do {
            System.out.println("Count is: " + count);
            count++;
        } while (count < 11);
    }

}
