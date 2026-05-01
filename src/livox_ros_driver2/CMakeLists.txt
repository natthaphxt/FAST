# judge which cmake codes to use 
if(ROS_EDITION STREQUAL "ROS1")
  # (ส่วนของ ROS 1 คงเดิมไว้ตามที่คุณส่งมา)
  cmake_minimum_required(VERSION 3.0)
  include(cmake/version.cmake)
  project(livox_ros_driver2 VERSION ${LIVOX_ROS_DRIVER2_VERSION} LANGUAGES CXX)
  add_definitions(-DBUILDING_ROS1)
  # ... (โค้ด ROS1 เดิมของคุณ) ...
  # [ละไว้เพื่อความกระชับ แต่ในไฟล์จริงให้คงไว้ปกติครับ]

else() ################### ROS 2 Humble Section ###################

  cmake_minimum_required(VERSION 3.14)
  project(livox_ros_driver2)

  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

  # 1. ค้นหา Package พื้นฐานที่จำเป็นสำหรับ ROS 2
  find_package(ament_cmake_auto REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_components REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(builtin_interfaces REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(PCL REQUIRED)

  # ค้นหา APR (ต้องใช้สำหรับ Livox SDK)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(APR REQUIRED apr-1)

  # ใช้ ament_auto ในการจัดการ dependencies ที่ระบุใน package.xml
  ament_auto_find_build_dependencies()

  # 2. สร้าง Custom Message (livox_interfaces2)
  set(LIVOX_INTERFACES livox_interfaces2)
  rosidl_generate_interfaces(${LIVOX_INTERFACES}
    "msg/CustomPoint.msg"
    "msg/CustomMsg.msg"
    DEPENDENCIES builtin_interfaces std_msgs
    LIBRARY_NAME ${PROJECT_NAME}
  )

  # 3. ค้นหา Livox SDK Library ที่เราติดตั้งไว้ใน /usr/local
  find_library(LIVOX_LIDAR_SDK_LIBRARY liblivox_lidar_sdk_shared.so /usr/local/lib REQUIRED)
  find_path(LIVOX_LIDAR_SDK_INCLUDE_DIR NAMES "livox_lidar_api.h" REQUIRED)

  # 4. สร้าง Library สำหรับ Driver
  add_definitions(-DBUILDING_ROS2)
  ament_auto_add_library(${PROJECT_NAME} SHARED
    src/livox_ros_driver2.cpp
    src/lddc.cpp
    src/driver_node.cpp
    src/lds.cpp
    src/lds_lidar.cpp
    src/comm/comm.cpp
    src/comm/ldq.cpp
    src/comm/semaphore.cpp
    src/comm/lidar_imu_data_queue.cpp
    src/comm/cache_index.cpp
    src/comm/pub_handler.cpp
    src/parse_cfg_file/parse_cfg_file.cpp
    src/parse_cfg_file/parse_livox_lidar_cfg.cpp
    src/call_back/lidar_common_callback.cpp
    src/call_back/livox_lidar_callback.cpp
  )

  # 5. แก้จุด NOTFOUND: ใช้ท่ามาตรฐานของ Humble ในการดึง Typesupport
  rosidl_get_typesupport_target(cpp_typesupport_target
    ${LIVOX_INTERFACES} "rosidl_typesupport_cpp")
  
  # เชื่อมต่อ Message Target เข้ากับ Library หลัก
  target_link_libraries(${PROJECT_NAME} "${cpp_typesupport_target}")

  # 6. ตั้งค่า Include Directories (ตัดตัวแปรที่หาไม่เจอออก)
  target_include_directories(${PROJECT_NAME} PUBLIC
    ${PCL_INCLUDE_DIRS}
    ${APR_INCLUDE_DIRS}
    ${LIVOX_LIDAR_SDK_INCLUDE_DIR}
    3rdparty
    src
  )

  # 7. เชื่อมต่อ Libraries อื่นๆ
  target_link_libraries(${PROJECT_NAME}
    ${LIVOX_LIDAR_SDK_LIBRARY}
    ${PCL_LIBRARIES}
    ${APR_LIBRARIES}
  )

  # 8. ลงทะเบียน Node เป็น Component (เพื่อให้ใช้ใน Composable Nodes ได้)
  rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "livox_ros::DriverNode"
    EXECUTABLE ${PROJECT_NAME}_node
  )

  # สั่งติดตั้งไฟล์ต่างๆ
  ament_auto_package(INSTALL_TO_SHARE
    config
    launch_ROS2
  )

endif()