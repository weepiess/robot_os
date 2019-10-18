include_directories(./include
                    ./include/actions
                    ./include/test
                    ./include/proxy)

aux_source_directory(./src SENTRY_SRCS)
aux_source_directory(./src/actions SENTRY_ACTIONS)
aux_source_directory(./src/test SENTRY_TEST)
aux_source_directory(./src/proxy SENTRY_PROXY)

add_library(SentryDecision STATIC ${SENTRY_SRCS} ${SENTRY_ACTIONS} ${SENTRY_TEST} ${SENTRY_PROXY})

target_link_libraries(SentryDecision
    BehaviorTree
    Recognition
)