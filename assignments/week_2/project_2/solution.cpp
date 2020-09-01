#include <iostream>
#include <string>

struct LightOutputs {
  bool red_ns;
  bool yellow_ns;
  bool green_ns;
  bool red_ew;
  bool yellow_ew;
  bool green_ew;
};

struct LightInputs {
  int time;
  bool car_sensor_n;
  bool car_sensor_s;
};

class LightController {
public:
  LightOutputs update(const LightInputs &inputs) {
      checkForTransitions(inputs);
      return runState();
  }

  void checkForTransitions(const LightInputs &inputs) {
      switch (current_state_) {
          case RED_ALL:
              if (inputs.car_sensor_n || inputs.car_sensor_s) {
                  current_state_ = GREEN_NS;
              } else {
                  current_state_ = GREEN_EW;
              }
              next_transition_time_ = inputs.time + 5;
              break;
          case YELLOW_NS:
              if (inputs.time >= next_transition_time_) {
                  current_state_ = RED_ALL;
              }
              break;
          case GREEN_NS:
              if (inputs.time >= next_transition_time_) {
                  current_state_ = YELLOW_NS;
                  next_transition_time_ = inputs.time + 2;
              }
              break;
          case YELLOW_EW:
              if (inputs.time >= next_transition_time_) {
                  current_state_ = RED_ALL;
              }
              break;
          case GREEN_EW:
              if (inputs.time >= next_transition_time_) {
                  current_state_ = YELLOW_EW;
                  next_transition_time_ = inputs.time + 2;
              }
              break;
      }
  }

  LightOutputs runState() {
      switch (current_state_) {
          case RED_ALL:
              return {true, false, false, true, false, false};
          case YELLOW_NS:
              return {false, true, false, true, false, false};
          case GREEN_NS:
              return {false, false, true, true, false, false};
          case YELLOW_EW:
              return {true, false, false, false, true, false};
          case GREEN_EW:
              return {true, false, false, false, false, true};
      }
  }

private:
  enum State {
    RED_ALL,
    YELLOW_NS,
    GREEN_NS,
    YELLOW_EW,
    GREEN_EW
  };

  State current_state_{RED_ALL};

  int next_transition_time_{0};
};

int main() {
    int time = 0;
    LightController controller;
    while (true) {
        std::string input;
        std::getline(std::cin, input);
        if(input.size() != 2)
            break;

        const LightInputs inputs{
            time,
            input[0] == '1',
            input[1] == '1'
        };

        const auto output = controller.update(inputs);

        std::cout << output.red_ns << output.yellow_ns << output.green_ns
                  << output.red_ew << output.yellow_ew << output.green_ew
                  << "\n";

        time++;
    }
    return 0;
}
