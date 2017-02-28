option(BUILD_Dynamics_PLUGIN "Building a Dynamics plugin" OFF)

if(BUILD_DYNAMICS_PLUGIN)
  set(target CnoidDynamicsPlugin)
  add_cnoid_plugin(${target} SHARED DynamicsPlugin.cpp)
  target_link_libraries(${target} CnoidBase CnoidBody)
  apply_common_setting_for_plugin(${target})
endif()

install_sample_source(DynamicsPlugin.cpp)
