project('Solution1','c',
default_options : ['warning_level=3',
                     'c_std=c11']  )

cc = meson.get_compiler('c')
m_dep = cc.find_library('m', required: true)
src_hoverslam = files('src/common.c', 'src/hoverslam.c')
src_pid = files('src/common.c','src/pid.c')
include = include_directories('../../lib/include', 'include')
lib = cc.find_library('rocket',  required: true)

executable('hoverslam',src_hoverslam,include_directories: include,  dependencies: [m_dep, lib])
executable('pid',src_pid,include_directories: include, dependencies: [m_dep, lib])
