
add_library(ogre_render ogre_application.cpp camera_render_application.cpp)
target_link_libraries(ogre_render  ${OGRE_LIBRARIES} ${OIS_LIBRARIES})

add_library(virtual_image_proc virtual_image_handler.cpp)
target_link_libraries(virtual_image_proc ogre_render ${OpenCV_LIBS})

