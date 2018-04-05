file(GLOB SRCS *.cpp)

ADD_EXECUTABLE(testCylindricalModel ${SRCS})
TARGET_LINK_LIBRARIES(
    testCylindricalModel
    CYLINDRICAL_MODEL 
    ${PCL_LIBRARIES}
    libgtest
    libgmock
)
add_dependencies(testCylindricalModel cylinder_segmentation_reference)
add_custom_command(TARGET testCylindricalModel
 COMMAND cp ${CMAKE_BINARY_DIR}/src/cylinder_coefficients_ref ${CMAKE_CURRENT_BINARY_DIR}
 COMMAND cp ${CMAKE_BINARY_DIR}/src/plane_coefficients_ref ${CMAKE_CURRENT_BINARY_DIR}
 COMMAND cp ${CMAKE_BINARY_DIR}/src/table_scene_mug_stereo_textured.pcd ${CMAKE_CURRENT_BINARY_DIR}
 COMMAND cp ${CMAKE_BINARY_DIR}/src/table_scene_mug_stereo_textured_cylinder.pcd ${CMAKE_CURRENT_BINARY_DIR}
 COMMAND cp ${CMAKE_BINARY_DIR}/src/table_scene_mug_stereo_textured_plane.pcd ${CMAKE_CURRENT_BINARY_DIR}
  )


add_test(NAME testCylindricalModel
  COMMAND testCylindricalModel)
