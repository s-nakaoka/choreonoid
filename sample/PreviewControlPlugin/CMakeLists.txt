option(BUILD_PREVIEWCONTROL "Building a sample plugin \"PreviewControlPlugin\"" OFF)

if(BUILD_PREVIEWCONTROL)
  set(target CnoidPreviewControlPlugin)
	set(libs hrpModel-3.1 hrpCollision-3.1 hrpUtil-3.1 hrpsysBaseStub)

	find_package(PkgConfig)
	pkg_check_modules(OPENHRP3 REQUIRED openhrp3.1)

	include_directories(${OPENHRP3_PREFIX}/include/OpenHRP-3.1/ ${Choreonoid_SOURCE_DIR}/sample ${Choreonoid_SOURCE_DIR}/include)
	link_directories(${OPENHRP3_PREFIX}/lib/ ${hrpsys_PACKAGE_PATH}/lib)

  add_cnoid_plugin(${target} SHARED PreviewControlPlugin.cpp PreviewController.cpp)
  target_link_libraries(${target} CnoidBodyPlugin CnoidPoseSeqPlugin ${libs})
  apply_common_setting_for_plugin(${target})
endif()

install_sample_source(PreviewControlPlugin.cpp)
