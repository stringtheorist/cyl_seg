

add_subdirectory(cylindrical_model)

add_executable (cylinder_segmentation cylinder_segmentation.cpp)
add_executable (cylinder_segmentation_reference cylinder_segmentation_reference.cpp)

target_link_libraries (cylinder_segmentation CYLINDRICAL_MODEL ${PCL_LIBRARIES})
target_link_libraries (cylinder_segmentation_reference ${PCL_LIBRARIES})

#run the reference implementation after building it so that we have data to compare with
# for the tests
add_custom_command(TARGET cylinder_segmentation_reference
  POST_BUILD
  COMMAND cp ${CMAKE_SOURCE_DIR}/table_scene_mug_stereo_textured.pcd ${CMAKE_CURRENT_BINARY_DIR}
  COMMAND ${CMAKE_CURRENT_BINARY_DIR}/cylinder_segmentation_reference
  )


