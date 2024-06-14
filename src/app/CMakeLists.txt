add_executable(orindectection_tracker orindectection_tracker.cpp
                        ${DET_SRC_DIR}
                        ${sourcefiles})

add_executable(rev_win rev_win.cpp ${DET_SRC_DIR})
target_link_libraries(rev_win ${dynamic_libs} ${OpenCV_LIBS} vps_pcap opencv_core)


target_link_libraries(orindectection_tracker MOT KCF ${dynamic_libs} vps_pcap img_codec)
target_link_libraries(orindectection_tracker ${OpenCV_LIBS} )
target_link_libraries(orindectection_tracker ${CUDA_LIBRARIES})
target_link_libraries(orindectection_tracker CUDA::cudart nvinfer nvinfer_plugin)


message("Add project win_mode")
install(TARGETS orindectection_tracker DESTINATION ./)
install(TARGETS rev_win DESTINATION ./)
install(TARGETS MOT  DESTINATION ./lib)
install(TARGETS KCF  DESTINATION ./lib)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/3rdparty/img_codec/lib DESTINATION ./)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/3rdparty/vps_pcap/lib DESTINATION ./)

