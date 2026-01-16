cmake_minimum_required(VERSION 2.8.3)

project(spark1_description)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  message_generation
)

 add_service_files(
  FILES
  Command.srv
)
 generate_messages(
   DEPENDENCIES
   std_msgs
  
 )

catkin_package(

  CATKIN_DEPENDS roscpp std_msgs std_srvs 

)
include_directories(include ${catkin_INCLUDE_DIRS} ) 


foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
