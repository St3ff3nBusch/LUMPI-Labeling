cmake_minimum_required(VERSION 3.7 FATAL_ERROR)
project(LUMPILabelingTool) 
############################################################################
#############################FIND PACKAGES##################################
############################################################################
find_package(PCL REQUIRED)
find_package(VTK REQUIRED)
find_package(Qt5 COMPONENTS Xml Widgets REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
############################################################################
##################SET FLAGES AND INCLUDE DIRECTORIES########################
############################################################################
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -Wno-all -Wno-comment")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif(OPENMP_FOUND)
set(CMAKE_VERBOSE_MAKEFILE On)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include_directories(include include/model include/model/geometries)
############################################################################
##################DEFINE QT OBJECTS SIGANL AND SLOTS########################
############################################################################
SET(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTOUIC ON)
SET(CMAKE_AUTORCC ON)
SET(CMAKE_INCLUDE_CURRENT_DIR YES)
SET(Qt_HDRS ${Qt_HDRS} include/presenter/LabelingPresenter.h include/model/Labeling.h )
QT5_WRAP_CPP(Qt_HDRS_MOC ${Qt_HDRS} ) 
set(CMAKE_BUILD_TYPE Release)
add_definitions(-std=c++17  -DDEBUG -fPIC -fopenmp ${Qt5Widgets_DEFINITIONS} )
############################################################################
###########EFFENCENTLY COMPILING  OBJECT TO SEPERATE LIBRARIES##############
############################################################################
add_library(ikgGeometries STATIC  src/model/geometries/BoundingBox.cpp   src/model/StructsAndMore.cpp  src/model/geometries/Pose.cpp src/model/geometries/Trajectory.cpp) 
add_library(ikgBase STATIC  src/model/LUMPIPipeline.cpp src/model/Labeling.cpp src/model/Frustum.cpp src/model/Plotter.cpp src/model/Parser.cpp)
add_library(ikg_view STATIC src/view/LabelingView.cpp)
add_library(ikg_presenter STATIC src/presenter/LabelingPresenter.cpp include/presenter/PresenterUtils.hpp)
target_link_libraries(ikgGeometries ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(ikgBase  ikgGeometries Qt5::Xml ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${OpenMP_LIBRARIES})
target_link_libraries(ikg_view ikgGeometries ikgBase Qt5::Widgets ${OpenCV_LIBRARIES}  ${OpenMP_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(ikg_presenter  ikgGeometries  ikgBase   Qt5::Widgets   ikg_view   ${OpenCV_LIBRARIES}  ${PCL_LIBRARIES} ${OpenMP_LIBRARIES})
############################################################################
###########COMPILE EXECUTABLE PROGRAM#######################################
############################################################################
add_executable(Labeling src/LabelingMain.cpp ${Qt_HDRS_MOC} ${MKL_LIBRARIES})
target_link_libraries(Labeling -Wl,--start-group ${VTK_LIBRARIES} Qt5::Widgets ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${OpenMP_LIBRARIES} ikgGeometries ikgBase ikg_presenter ikg_view -Wl,--end-group)
file(COPY ${CMAKE_SOURCE_DIR}/data DESTINATION ${CMAKE_BINARY_DIR})

