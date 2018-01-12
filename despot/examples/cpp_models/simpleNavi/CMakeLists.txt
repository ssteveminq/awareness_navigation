cmake_minimum_required(VERSION 2.8.3)

add_executable("${PROJECT_NAME}_simpleNavi"
  src/simpleNavi.cpp
  src/main.cpp
)
target_link_libraries("${PROJECT_NAME}_simpleNavi"
  "${PROJECT_NAME}"
)
install(TARGETS "${PROJECT_NAME}_simpleNavi"
  RUNTIME DESTINATION "${BINARY_INSTALL_PATH}"
)
