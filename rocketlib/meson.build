project('Rocket','c',
default_options : ['warning_level=3',
                     'c_std=c11']  )


src = files('src/logger.c', 'src/PID.c','src/rocket.c','src/utils.c', 'src/fparser.c')
include = include_directories('include')
cc = meson.get_compiler('c')
m_dep = cc.find_library('m', required: true)

shared_library('rocket',src,include_directories: include,dependencies: [m_dep], install: true)
install_headers('include/display.h','include/rocketlib.h')
install_headers('include/rocketlib/logger.h', 'include/rocketlib/PID.h','include/rocketlib/rocket.h','include/rocketlib/utils.h', 'include/rocketlib/fparser.h', 'include/rocketlib/events.h', subdir: 'rocketlib')