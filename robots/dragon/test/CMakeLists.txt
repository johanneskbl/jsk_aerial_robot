add_rostest(dragon_jacobian.test ARGS gui:=false)
add_rostest(dragon_control.test ARGS gui:=false) # old control method: hydrus-like LQI mode
add_rostest(dragon_control.test ARGS gui:=false full_vectoring_mode:=true) # new control method
