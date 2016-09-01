#ifndef TOOLS_H
#define TOOLS_H

#include <ros/ros.h>

namespace moveit_simple_actions
{

  void printTutorial(const std::string robot_name)//, const int state
  {
    std::string key = "u";
    //if (state == 0)
    //{
      std::cout <<
                            "\n \n"
                            << "************************************************\n"
                            << "Tutorial for simple actions with Romeo, NAO, and Pepper robots \n"
                            << "You have choosen -" << robot_name << "- robot."
                            << std::endl;

      std::cout <<
                            "In your scene, you should see \n"
                            << "- a robot \n"
                            << "- a green cylinder -object-"
                            << std::endl;

      if (robot_name == "romeo")
      {
        key = "g";
        std::cout <<
                            "- a black table."
                            << std::endl;

        std::cout <<
                            "We inform you, that this tutorial works best with -Romeo- robot."
                            << std::endl;;
      }

    //}
    //else if (state == 1)
      std::cout <<
                            "\n The scenario is \n"
                            << "1) Try to grasp an object (for Romeo), by pressing -" << key << "- key and enter.\n"
                            << std::endl;
    //else if (state == 3)
      std::cout <<
                            "2) If your grasp is sucessful, try to place the object, by typing -place-"
                            << std::endl;

    //else if (state == 2)
       std::cout <<
                            "   If your grasp is unsuccessfull, try to grasp again.\n"
                             << "   Also, you can move the object right/left/top/down/farther/closer by pressing -s- -f- -e- -x- -r- -c- keys"
                             << std::endl;

    //else if (state == 4)
      std::cout <<
                            "3) Try to grasp with another hand"
                            << std::endl;

      std::cout <<
      "************************************************\n"
         << std::endl;

  }

  void printAllActions()
  {
    ROS_INFO_STREAM_NAMED("simple_actions:", "Possible actions are: \n"
                          << " g - pick an object, \n"
                          << " p - place the object, \n"

                          << " u - reach and grasp, \n"
                          //<< " pregrasp - reach the pregrasp pose, \n"
                          //<< " reachtop - reach the object from top, \n"
                          //<< " w - reach the init pose, \n"

                          //<< " plan - plan grasping the object, \n"
                          << " a - plan all possible grasps, \n"
                          //<< " execute - execute the planned action, \n"

                          //<< " v - show the pregrasp pose, \n"
                          << " i - go to init pose, \n" //(i0, i1, i2, i3)
                          //<< " z - move the hand to the zero pose, \n"
                          //<< " open - open hand, \n"
                          //<< " close - close hand, \n"
                          //<< " m - move the head to look down, \n"

                          << " lc - add a cylinder on the left, \n"
                          << " rc - add a cylinder on the right, \n"
                          << " t - add/remove the table/scene, \n"
                          << " d - detect objects, \n"
                          //<< " n - process the next object, \n"
                          << " e/s/f/x/r/c - move the object top/left/right/down/closer/farther \n"

                          << " test_pick - test the goal space for picking, \n"
                          << " test_reach - test the goal space for reaching, \n"
                          //<< " stat - print statistics on successfull grasps, \n"

                          << " q - exit, \n"
                          << " So, what do you choose?"
                          );
  }

  int promptUserAction()
  {
    printAllActions();
    char ascii;
    std::cin >> ascii;
    int in = (int) ascii;
    return in;
  }

  std::string promptUserQuestionString()
  {
    printAllActions();
    std::string input;
    std::cin >> input;
    return input;
  }

  bool promptUserQuestion(const std::string command)
  {
    ROS_INFO_STREAM_NAMED("pick_place",command);
    char input;
    std::cin >> input;
    if( input == 'n' ) // used for yes/no
      return false;

    return true;
  }

  double promptUserValue(const std::string command)
  {
    ROS_INFO_STREAM_NAMED("pick_place",command);
    double input;
    std::cin >> input;
    return input;
  }

  bool promptUser()
  {
    ROS_INFO_STREAM_NAMED("pick_place","Retry? (y/n)");
    char input; // used for prompting yes/no
    std::cin >> input;
    if( input == 'n' )
      return false;

    return true;
  }
}
#endif // TOOLS_H
