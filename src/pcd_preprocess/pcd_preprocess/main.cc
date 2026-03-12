#include "pcd_preprocess/data_interface/data_interface.h"
#include "pcd_preprocess/pcd_preprocess.h"

using preprocess::DataInterface;
using preprocess::PcdPreprocess;
using preprocess::LogLevel;

const char kNodeName[] = "pcd_preprocess";

void TimeHandle() {
  enum class FiniteState { kInitState = 0, kWorkState };
  static FiniteState finite_state_machine_state = FiniteState::kInitState;
  static DataInterface& data_interface = DataInterface::GetSingleton();
  static PcdPreprocess& pcd_preprocess =
      PcdPreprocess::GetSingleton();

  switch (finite_state_machine_state) {
    case FiniteState::kInitState: {
      if (pcd_preprocess.Init()) {
        data_interface.Log(LogLevel::kInfo, "%s : Init Succeded", kNodeName);
        finite_state_machine_state = FiniteState::kWorkState;
      } else {
        data_interface.Log(LogLevel::kWarn, "%s : Init Failed", kNodeName);
        finite_state_machine_state = FiniteState::kInitState;
      }
      break;
    }
    case FiniteState::kWorkState: {
      if (pcd_preprocess.Work()) {
        // data_interface.Log(LogLevel::kInfo, "%s : Work Succeded",
        // kNodeName);
        finite_state_machine_state = FiniteState::kWorkState;
      } else {
        data_interface.Log(LogLevel::kWarn, "%s : Work Failed", kNodeName);
        finite_state_machine_state = FiniteState::kInitState;
      }
      break;
    }
    default: {
      data_interface.Log(LogLevel::kError, "%s : Unknown State", kNodeName);
      finite_state_machine_state = FiniteState::kInitState;
      break;
    }
  }
}

int main(int argc, char** argv) {
  DataInterface& data_interface = DataInterface::GetSingleton();
  data_interface.Init(argc, argv, kNodeName, 2.0, TimeHandle);

  data_interface.Work();

  data_interface.Deinit();
  return 0;
}
