#Comment: or use AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR} I2C_BNO055)
set(header_path ${CMAKE_SOURCE_DIR}/include/${CMAKE_PROJECT_NAME})
set(header ${header_path}/getbno055.h)
#set(lib_src ${CMAKE_SOURCE_DIR}/src/i2c_bno055.c)
set(lib_src ${CMAKE_SOURCE_DIR}/src/i2c_bno055.c)


add_library(getbno055 ${header} ${lib_src})
#Comment: Rename libllibgetbno055.a to libgetbno055.a
#set_target_properties(libgetbno055 PROPERTIES OUTPUT_NAME "getbno055")
#Comment: reason for PUBLIC rather than PRIVATE - even though the directory of this lib to be built will be built in the same project as for the final executable, which leads to feasible use of PRIVATE. But we still need this library to be linked to in other projects, which leads to essential use of PUBLIC.
target_include_directories(getbno055 PUBLIC
                            $<BUILD_INTERFACE:${header_path}> # for headers when building
                            $<INSTALL_INTERFACE:${include_dest}>) # for client in install mode
#Explanation: math.h (belongs to libm & m stands for libm) is not a part of the standard C library, so you have to link to it
#Comment: append link flags 'm' or '-lm'
target_link_libraries(getbno055 PUBLIC m)


install(TARGETS getbno055
        EXPORT getbno055
        DESTINATION ${main_lib_dest})
install(FILES ${CMAKE_SOURCE_DIR}/include/${CMAKE_PROJECT_NAME}/getbno055.h
        DESTINATION ${include_dest})