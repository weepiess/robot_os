#include "behavior_tree.h"

void bt::BehaviorTree::execute(){
    //必须在之前调用
    initBT();
    running_ = true;
    while (running_) {
        std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
        callBeforeRun();
        root_node_ptr->run();
        std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
        std::chrono::milliseconds execution_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        std::chrono::milliseconds sleep_time = cycle_duration_ - execution_duration;
        
        if (sleep_time > std::chrono::microseconds(0)) {
            std::this_thread::sleep_for(sleep_time);
            LOG_INFO << "sleep: " << sleep_time.count() << "ms";
        
        } else {
            LOG_INFO << "The time tick once is " << execution_duration.count() << " beyond the expected time "
                        << cycle_duration_.count();
        }
        LOG_WARNING << "----------------------------------";   
    }
}