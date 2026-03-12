#include "pcd_localization/pcd_localization.h"

using localization::DataInterface;
using localization::LogLevel;
using localization::PcdLocalization;

const char kNodeName[] = "pcd_localization";

void TimeHandle() {
  enum class FiniteState { kInitState = 0, kWorkState };
  static FiniteState finite_state_machine_state = FiniteState::kInitState;
  static PcdLocalization& pcd_localization = PcdLocalization::GetSingleton();
  static DataInterface& data_interface = DataInterface::GetSingleton();

  switch (finite_state_machine_state) {
    case FiniteState::kInitState: {
      if (pcd_localization.Init()) {
        data_interface.Log(LogLevel::kInfo, "%s : Init Succeded", kNodeName);
        finite_state_machine_state = FiniteState::kWorkState;
      } else {
        data_interface.Log(LogLevel::kWarn, "%s : Init Failed", kNodeName);
        finite_state_machine_state = FiniteState::kInitState;
      }
      break;
    }
    case FiniteState::kWorkState: {
      if (pcd_localization.Work()) {
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
  data_interface.Init(argc, argv, "pcd_localization", 50.0, TimeHandle);

  data_interface.Work();

  data_interface.Deinit();

  return 0;
}
