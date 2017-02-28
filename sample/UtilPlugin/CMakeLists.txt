
option(BUILD_UTIL_PLUGIN "Building a Util plugin" OFF)

if(BUILD_UTIL_PLUGIN)
  set(target CnoidUtilPlugin)
  add_cnoid_plugin(${target} SHARED UtilPlugin.cpp)
  target_link_libraries(${target} CnoidBase CnoidBody)
  apply_common_setting_for_plugin(${target})
endif()

install_sample_source(UtilPlugin.cpp)
