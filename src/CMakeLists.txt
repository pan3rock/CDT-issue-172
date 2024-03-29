add_executable(demo1
    demo1.cc
)

target_link_libraries(demo1
    PRIVATE
    fmt::fmt-header-only
    CDT::CDT
    predicates
)