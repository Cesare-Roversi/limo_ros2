cmake_minimum_required(VERSION 3.8)
project(limo_car)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Non abbiamo dipendenze perchè questo pacchetto non contiene file c++


install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
