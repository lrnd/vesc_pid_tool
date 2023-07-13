project(
    'util-graph',
    'cpp',
    default_options : ['c_std=c++17']
)

cc = meson.get_compiler('cpp')
m_dep = cc.find_library('m', required: false)

deps = [
    dependency('gtk+-3.0'),
    m_dep
]

#gnome = import('gnome')

#asresources = gnome.compile_resources(
#    'util-draw.ui', 'data/util-draw.ui',
#    source_dir: 'data',
#    c_name: 'util-draw.ui'
#)

sources = [
    'util-draw.cpp'
]

executable('util', sources, dependencies: deps)

#executable(
#    meson.project_name(),
#    asresources,
#    dependencies: deps,
#    install: true
#)