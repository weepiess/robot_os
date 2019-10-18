include_directories(
    ./include
    ./include/controls
    ./include/decorators
    ./include/blackboard
)

aux_source_directory(./src BT_SRCS)
aux_source_directory(./src/controls BT_CONTROLS_SRCS)
aux_source_directory(./src/decorators BT_DECORATORS_SRCS)

add_library(BehaviorTree STATIC ${BT_SRCS} ${BT_CONTROLS_SRCS} ${BT_DECORATORS_SRCS})
