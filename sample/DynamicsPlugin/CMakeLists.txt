option(BUILD_UTIL_SAMPLE "Building a Dynamics plugin" OFF)

if(BUILD_UTIL_SAMPLE)
  set(target CnoidDynamicsPlugin)
  add_cnoid_plugin(${target} SHARED DynamicsPlugin.cpp)
  target_link_libraries(${target} CnoidBase CnoidBody)
  apply_common_setting_for_plugin(${target})
endif()

install_sample_source(DynamicsPlugin.cpp)
