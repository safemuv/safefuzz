include_directories(SYSTEM /usr/local/include/activemq-cpp-3.9.5 /usr/include/apr-1)
include_directories(SYSTEM /home/userfs/j/jrh512/source/activemq-cpp-install/include/activemq-cpp-3.9.5)
include_directories(SYSTEM /home/userfs/j/jrh512/source/apr-install/include/apr-1)
include_directories(SYSTEM /usr/include/activemq-cpp-3.9.5)

add_subdirectory(libMOOS)
add_subdirectory(tools)
add_subdirectory(MOOSDB)
